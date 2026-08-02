#!/usr/bin/env python3
"""lerobot 遥操作 -> Isaac Sim 实时镜像桥接 / lerobot teleoperate -> Isaac Sim live-mirror bridge.

功能概述：
1. 复用 `lerobot-teleoperate` 完全相同的 CLI 配置（`--robot.type=...`
   `--teleop.type=...` 等），驱动 leader -> follower 遥操作循环。
2. 每次循环读取 follower 的 `get_observation()`（本来就会被遥操作循环读取，
   不会增加额外的 CAN 总线负载），转换为与 reBot-Isaacsim 的
   `isaacsim_joint_receiver.py` 一致的 UDP JSON 协议，发送出去。
3. 不修改真实机械臂的控制逻辑：动作处理管线与 `lerobot-teleoperate` 完全一致。

本包必须安装在装有 `lerobot` / `motorbridge` /
`lerobot_robot_seeed_b601` / `lerobot_teleoperator_rebot_arm_102`（或对应
硬件的机器人/遥操作插件）的 Python 环境中，通常是独立于任何仿真项目的
conda/venv 环境。

注意（重要，请务必核实，详见 README）：
- follower 的电机驱动栈（例如 motorbridge）与接收端所用的仿真资产可能来自
  两套完全独立的零位标定/正方向约定，不能假设直接可用。默认取反系数按关节
  区分（`DEFAULT_JOINT_SIGN`），依据是 reBot-Isaacsim 项目 USD 资产中
  joint2/joint3 的物理关节限位为 `[-179.90875, 0]`（只允许负值）：实测这两
  个关节的 follower 原始读数本身就是负值，因此不取反（+1）；其余关节取反
  （-1）。如果目标是别的仿真资产/URDF，请重新核实每个关节的符号。
- 关节顺序假定为 lerobot 标准命名
  `shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_yaw, wrist_roll`
  依次对应接收端仿真资产的 joint1..joint6；如果关节错位，调整本文件顶部
  `ARM_JOINT_NAMES` 的顺序。
- 夹爪：假定 follower 标定零位即"闭合"姿态，张开时角度向负方向增大，因此取
  绝对值换算，使 0=闭合。换算比例 `--gripper_position_scale` 未经标定，默认
  值只是占位，请对照接收端夹爪行程现场调整。

Overview:
1. Reuses the exact same CLI config surface as `lerobot-teleoperate`
   (`--robot.type=...`, `--teleop.type=...`, etc.) to drive the
   leader -> follower teleoperation loop.
2. Each loop iteration reads the follower's `get_observation()` (already
   read once per iteration by the stock teleop loop, so this adds no extra
   CAN bus traffic), converts it to the same UDP JSON wire format used by
   reBot-Isaacsim's `isaacsim_joint_receiver.py`, and sends it out.
3. Does not change real-arm control behavior: the action-processing
   pipeline is identical to `lerobot-teleoperate`.

This package must be installed in a Python environment that has `lerobot` /
`motorbridge` / `lerobot_robot_seeed_b601` /
`lerobot_teleoperator_rebot_arm_102` (or your hardware's robot/teleoperator
plugins) installed - typically a conda/venv environment kept separate from
any simulation project.

Caveats (important, please verify - see README for details):
- The follower's motor-driver stack (e.g. motorbridge) and the receiving
  simulation asset may come from two entirely independent zero
  calibrations/positive-direction conventions; don't assume it just works.
  The default sign is per-joint (`DEFAULT_JOINT_SIGN`), based on the
  reBot-Isaacsim project's USD asset having joint2/joint3 limited to
  `[-179.90875, 0]` (negative values only): the follower's raw readings for
  those two are already negative, so they are NOT flipped (+1); the rest
  are flipped (-1). If targeting a different simulation asset/URDF,
  re-verify every joint's sign.
- Joint order assumes lerobot's standard naming
  `shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_yaw, wrist_roll`
  maps in order to the receiving asset's joint1..joint6; adjust
  `ARM_JOINT_NAMES` at the top of this file if the joints are crossed.
- Gripper: assumes the follower's calibrated zero pose is "closed", and
  opening moves the angle further negative, so we take the absolute value,
  making 0=closed. `--gripper_position_scale` is an unverified placeholder;
  tune it against the receiver's finger joint limits.
"""

import json
import logging
import math
import signal
import socket
import time
from dataclasses import asdict, dataclass, field
from pprint import pformat

from lerobot.configs import parser
from lerobot.processor import make_default_processors
from lerobot.robots import Robot, RobotConfig, make_robot_from_config  # noqa: F401
from lerobot.teleoperators import Teleoperator, TeleoperatorConfig, make_teleoperator_from_config  # noqa: F401
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import init_logging

ARM_JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_yaw",
    "wrist_roll",
)
GRIPPER_MOTOR_NAME = "gripper"

# 每关节独立取反系数，而非全局统一取反。
# 依据：reBot-Isaacsim 项目 usd/RS-rebot-dev-arm/payloads/Physics/physics.usda
# 中 joint2/joint3 的物理关节限位为 physics:lowerLimit=-179.90875,
# physics:upperLimit=0 —— 只允许负值。实测 follower 的 shoulder_lift /
# elbow_flex 原始读数本身就是负值，如果再套用全局取反会变成正值，超出仿真
# 关节的合法范围，导致这两个关节在仿真里被钳制/卡住（其余关节限位对称，暂不
# 受影响）。因此 joint2/3 使用 +1（不取反），其余关节使用 -1。
# Per-joint sign, not one global flip.
# Reason: in the reBot-Isaacsim project's
# usd/RS-rebot-dev-arm/payloads/Physics/physics.usda, joint2/joint3 have
# physics:lowerLimit=-179.90875, physics:upperLimit=0 - negative values
# only. The follower's raw shoulder_lift / elbow_flex readings are already
# negative in their normal range; flipping them globally pushes them
# positive, outside the simulated joint's valid range, which clamps/breaks
# those two joints in sim (the other joints have symmetric limits and are
# unaffected). So joint2/3 use +1 (no flip); the rest use -1.
DEFAULT_JOINT_SIGN = (-1.0, 1.0, 1.0, -1.0, -1.0, -1.0)

DEFAULT_SIM_HOST = "127.0.0.1"
DEFAULT_SIM_PORT = 5005
DEFAULT_SEND_HZ = 60.0
DEFAULT_REPORT_EVERY = 30
DEFAULT_GRIPPER_POSITION_SCALE = 0.03  # 未标定占位值，需现场调整 / unverified placeholder, tune on-site

_running = True


def _sigint_handler(signum, frame) -> None:
    del signum, frame
    global _running
    print("\n[bridge] 收到 Ctrl+C，准备退出... / received Ctrl+C, preparing to exit...")
    _running = False


signal.signal(signal.SIGINT, _sigint_handler)


@dataclass
class TeleopSimBridgeConfig:
    teleop: TeleoperatorConfig
    robot: RobotConfig
    fps: int = 60
    teleop_time_s: float | None = None

    sim_host: str = DEFAULT_SIM_HOST
    sim_port: int = DEFAULT_SIM_PORT
    send_hz: float = DEFAULT_SEND_HZ
    report_every: int = DEFAULT_REPORT_EVERY
    # 每关节独立取反系数（对应 ARM_JOINT_NAMES 顺序），默认见 DEFAULT_JOINT_SIGN
    # 上方注释。如果仿真中某个关节方向反了，把对应位置的 1/-1 互换。
    # Per-joint sign (matching ARM_JOINT_NAMES order); see the
    # DEFAULT_JOINT_SIGN comment above for the default rationale. If a joint
    # moves the wrong way in sim, flip the corresponding 1/-1 entry.
    joint_sign: list[float] = field(default_factory=lambda: list(DEFAULT_JOINT_SIGN))
    # follower 的夹爪标定零位假定为"闭合"，张开时角度向负方向增大，因此这里
    # 取绝对值，使 0=闭合、正值=张开，与接收端"闭合=0"的裁剪约定一致。
    # Assumes the follower's calibrated zero pose is "closed"; opening moves
    # the angle further negative. We take the absolute value so 0=closed,
    # positive=open, matching the receiver's closed=0 clipping convention.
    gripper_position_scale: float = DEFAULT_GRIPPER_POSITION_SCALE


def teleop_loop_with_sim_bridge(
    teleop: Teleoperator,
    robot: Robot,
    cfg: TeleopSimBridgeConfig,
    teleop_action_processor,
    robot_action_processor,
) -> None:
    sim_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if len(cfg.joint_sign) != len(ARM_JOINT_NAMES):
        raise ValueError(
            f"joint_sign 长度必须为 {len(ARM_JOINT_NAMES)} / "
            f"joint_sign must have length {len(ARM_JOINT_NAMES)}, got {len(cfg.joint_sign)}"
        )
    send_period = 1.0 / cfg.send_hz if cfg.send_hz > 0 else 0.0

    sequence = 0
    last_send_time = 0.0
    start = time.perf_counter()

    try:
        while _running:
            loop_start = time.perf_counter()

            obs = robot.get_observation()
            raw_action = teleop.get_action()
            teleop_action = teleop_action_processor((raw_action, obs))
            robot_action_to_send = robot_action_processor((teleop_action, obs))
            robot.send_action(robot_action_to_send)

            now = time.perf_counter()
            if send_period <= 0 or now - last_send_time >= send_period:
                joint_positions = [
                    sign * math.radians(obs.get(f"{name}.pos", 0.0))
                    for name, sign in zip(ARM_JOINT_NAMES, cfg.joint_sign)
                ]
                gripper_deg = obs.get(f"{GRIPPER_MOTOR_NAME}.pos", 0.0)
                gripper_position = abs(math.radians(gripper_deg)) * cfg.gripper_position_scale

                payload = {
                    "sequence": sequence,
                    "timestamp": time.time(),
                    "joint_positions": joint_positions,
                    "gripper_position": gripper_position,
                }
                packet = json.dumps(payload, separators=(",", ":")).encode("utf-8")
                sim_socket.sendto(packet, (cfg.sim_host, cfg.sim_port))

                if sequence % cfg.report_every == 0:
                    deg_str = "  ".join(f"{math.degrees(v):+7.2f}" for v in joint_positions)
                    print(
                        f"[bridge] sim deg=[{deg_str}]  "
                        f"gripper_deg={gripper_deg:+.2f}  gripper_m={gripper_position:+.4f}"
                    )

                sequence += 1
                last_send_time = now

            dt_s = time.perf_counter() - loop_start
            precise_sleep(max(1 / cfg.fps - dt_s, 0.0))

            if cfg.teleop_time_s is not None and time.perf_counter() - start >= cfg.teleop_time_s:
                return
    finally:
        sim_socket.close()


@parser.wrap()
def bridge(cfg: TeleopSimBridgeConfig) -> None:
    init_logging()
    logging.info(pformat(asdict(cfg)))

    teleop = make_teleoperator_from_config(cfg.teleop)
    robot = make_robot_from_config(cfg.robot)
    teleop_action_processor, robot_action_processor, _ = make_default_processors()

    teleop.connect()
    robot.connect()

    print("=" * 72)
    print("  lerobot 遥操作 -> Isaac Sim 实时镜像桥接")
    print(f"  仿真接收端: udp://{cfg.sim_host}:{cfg.sim_port}")
    print(f"  关节取反 joint_sign={cfg.joint_sign}  夹爪比例={cfg.gripper_position_scale}")
    print("  停止方式: Ctrl+C")
    print("=" * 72)
    print()
    print("=" * 72)
    print("  lerobot teleoperate -> Isaac Sim live-mirror bridge")
    print(f"  sim receiver: udp://{cfg.sim_host}:{cfg.sim_port}")
    print(f"  joint_sign={cfg.joint_sign}  gripper_position_scale={cfg.gripper_position_scale}")
    print("  To stop: press Ctrl+C")
    print("=" * 72)

    try:
        teleop_loop_with_sim_bridge(
            teleop=teleop,
            robot=robot,
            cfg=cfg,
            teleop_action_processor=teleop_action_processor,
            robot_action_processor=robot_action_processor,
        )
    except KeyboardInterrupt:
        pass
    finally:
        print("[停止] 正在断开连接... / [stopping] disconnecting...")
        teleop.disconnect()
        robot.disconnect()
        print("[完成] 已安全退出 / [done] exited safely")


def main() -> None:
    register_third_party_plugins()
    bridge()


if __name__ == "__main__":
    main()
