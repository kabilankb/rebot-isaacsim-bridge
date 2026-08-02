#!/usr/bin/env python3
"""lerobot teleoperate -> Isaac Sim live-mirror bridge.

Overview:
1. Reuses the exact same CLI config surface as `lerobot-teleoperate`
   (`--robot.type=...`, `--teleop.type=...`, etc.) to drive the
   leader -> follower teleoperation loop.
2. Each loop iteration reads the follower's `get_observation()` (already
   read once per iteration by the stock teleop loop, so this adds no extra
   CAN bus traffic), converts it to the same UDP JSON wire format used by
   this repo's `isaacsim/isaacsim_joint_receiver.py`, and sends it out.
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
DEFAULT_GRIPPER_POSITION_SCALE = 0.03  # unverified placeholder, tune on-site

_running = True


def _sigint_handler(signum, frame) -> None:
    del signum, frame
    global _running
    print("\n[bridge] received Ctrl+C, preparing to exit...")
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
    # Per-joint sign (matching ARM_JOINT_NAMES order); see the
    # DEFAULT_JOINT_SIGN comment above for the default rationale. If a joint
    # moves the wrong way in sim, flip the corresponding 1/-1 entry.
    joint_sign: list[float] = field(default_factory=lambda: list(DEFAULT_JOINT_SIGN))
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
        print("[stopping] disconnecting...")
        teleop.disconnect()
        robot.disconnect()
        print("[done] exited safely")


def main() -> None:
    register_third_party_plugins()
    bridge()


if __name__ == "__main__":
    main()
