# rebot-isaacsim-bridge

Mirror a [lerobot](https://github.com/huggingface/lerobot) `lerobot-teleoperate` leader/follower session's live joint state into an Isaac Sim UDP receiver, in real time — while leaving the real-arm control behavior untouched.

https://github.com/user-attachments/assets/ab85519c-0284-4ccd-aac0-dd4f9524fce2

This package reuses `lerobot-teleoperate`'s exact CLI/config surface and action-processing pipeline. It adds one thing: each loop iteration, it takes the follower's `get_observation()` (already read once per iteration by the stock teleop loop — no extra CAN bus traffic) and forwards it as UDP JSON to an Isaac Sim receiver.

The repo has two independent halves that run in two different Python runtimes:

```
rebot-isaacsim-bridge/
├── src/rebot_isaacsim_bridge/   # pip-installable: the teleop -> UDP sender
│   └── bridge.py                # -> `lerobot-teleop-sim-bridge` console command
└── isaacsim/                    # NOT pip-installed: run with Isaac Sim's own python.sh
    ├── isaacsim_joint_receiver.py
    └── run_isaacsim_receiver.sh
```

`isaacsim/isaacsim_joint_receiver.py` imports `isaacsim`, which only exists inside Isaac Sim's bundled Python runtime — it can't be pip-installed into a normal venv, so it's kept out of `src/` and launched via Isaac's official `python.sh` instead (same as the rest of the reBot-Isaacsim project). The robot USD asset itself is **not** bundled in this repo (it's ~28MB of binary/USD); point `REBOT_ASSET_ROOT` at a directory that has it, e.g. a reBot-Isaacsim checkout.

## Install

This package declares its own complete dependency chain (`lerobot`, pinned to the exact fork commit it's verified against) and an optional `seeed-b601` extra for the Seeed reBotArm B601 follower + 102 leader plugins — no need to reuse or hand-tweak an existing `lerobot` environment.

Self-contained venv, in this directory:

```bash
python3.12 -m venv .venv
.venv/bin/pip install --upgrade pip
.venv/bin/pip install -e ".[seeed-b601]"
```

(Drop `[seeed-b601]` and just `pip install -e .` if you're targeting different hardware — bring your own robot/teleoperator plugin package instead.)

This pulls in `torch` and the rest of `lerobot`'s dependency tree, so the first install takes a while and a few GB of disk.

## Usage

**Terminal 0 — optional, first-time hardware check** (verify the motors respond before bringing up the full pipeline; see "Optional: motorbridge-studio live monitoring & debugging" below for details):

```bash
motorbridge-gateway -- --bind 127.0.0.1:9002 --vendor damiao --transport dm-serial \
    --serial-port /dev/ttyACM0 --serial-baud 921600 \
    --model 4340P --dt-ms 20
```

Open <https://motorbridge.github.io/motorbridge-studio/>, connect to `ws://127.0.0.1:9002`, and confirm the motors on `/dev/ttyACM0` show up and respond. **Stop this (Ctrl+C) before moving on to Terminal 2** — it holds the same serial port `lerobot-teleop-sim-bridge` needs, and the two can't run at once.

Two terminals for the actual pipeline. Start the Isaac Sim receiver first, then this bridge with the exact same flags you'd pass to `lerobot-teleoperate`.

**Terminal 1 — Isaac Sim receiver** (Isaac Sim's own `python.sh`, not this package's venv):

```bash
cd ~/Documents/rebot/rebot-isaacsim-bridge/isaacsim
export ISAACSIM_ROOT=/path/to/your/IsaacSim/_build/linux-x86_64/release
export REBOT_ASSET_ROOT=~/Documents/rebot/reBot-Isaacsim   # any dir containing usd/RS-rebot-dev-arm/
./run_isaacsim_receiver.sh
```

**Terminal 2 — this bridge** (from this package's own venv):

```bash
cd ~/Documents/rebot/rebot-isaacsim-bridge
.venv/bin/lerobot-teleop-sim-bridge \
  --robot.type=seeed_b601_dm_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.id=follower1 \
  --robot.can_adapter=damiao \
  --teleop.type=rebot_arm_102_leader \
  --teleop.port=/dev/ttyUSB0 \
  --teleop.id=rebot_arm_102_leader
```

(Or `source .venv/bin/activate` first and drop the `.venv/bin/` prefix.)

Both `ISAACSIM_ROOT` and `REBOT_ASSET_ROOT` are required; `run_isaacsim_receiver.sh` fails fast with a clear message if either is missing.

## Bridge-specific flags

On top of `lerobot-teleoperate`'s existing `--robot.*` / `--teleop.*` / `--fps` / `--teleop_time_s`:

| Flag | Default | Description |
|---|---|---|
| `--sim_host` | `127.0.0.1` | Isaac Sim receiver host |
| `--sim_port` | `5005` | Isaac Sim receiver port |
| `--send_hz` | `60.0` | UDP send rate |
| `--report_every` | `30` | print a debug log line every N frames |
| `--joint_sign` | `[-1,1,1,-1,-1,-1]` | per-joint sign, see "Joint sign convention" below |
| `--gripper_position_scale` | `0.03` | gripper degrees-to-meters scale, see "Gripper convention" below |

## Wire protocol

UDP JSON, one packet per send:

```json
{
  "sequence": 123,
  "timestamp": 1718000000.123,
  "joint_positions": [0.0, 0.1, 0.2, -0.1, 0.0, -0.02],
  "gripper_position": 0.05
}
```

| Field | Type | Description |
|---|---|---|
| `sequence` | int | increasing counter |
| `timestamp` | float | Unix timestamp (seconds) |
| `joint_positions` | float[6] | the 6 arm joint angles (radians) |
| `gripper_position` | float | gripper finger position target (meters), closed = 0 |

This matches `isaacsim/isaacsim_joint_receiver.py` in this repo (originally from the reBot-Isaacsim project), but any UDP JSON receiver speaking this format works.

## Joint sign convention

**This defaults are tuned for the reBot-Isaacsim `RS-rebot-dev-arm` USD asset. If you're targeting a different simulation asset/URDF, re-verify every joint's sign before trusting the defaults.**

The follower's motor-driver stack (e.g. `motorbridge`) and the target simulation asset can come from independent zero calibrations and positive-direction conventions, so a single global sign flip is not safe to assume.

For `RS-rebot-dev-arm`: its USD defines `joint2` (shoulder_lift) and `joint3` (elbow_flex) with:

```
physics:lowerLimit = -179.90875
physics:upperLimit = 0
```

i.e. negative values only. The follower's raw `shoulder_lift` / `elbow_flex` readings are already negative in their normal range, so flipping them globally would push them positive — outside the simulated joint's valid range, clamping those two joints to 0 (frozen) in sim.

Hence the default `joint_sign = [-1, 1, 1, -1, -1, -1]` (matching `shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_yaw, wrist_roll`): joint2/joint3 are not flipped (+1); the rest are flipped (-1).

If a joint moves the wrong way in your sim, override its sign, e.g. flipping wrist_yaw:

```bash
--joint_sign='[-1,1,1,-1,1,-1]'
```

## Gripper convention

Assumes the follower's calibrated zero pose is "closed" (true for the Seeed B601 driver — see the "close its gripper" zero-reference note in `SeeedB601FollowerBase.calibrate()`), and that opening the gripper moves the angle further negative. The bridge takes the absolute value so `0 = closed`, matching a receiver convention where closed=0 and any negative target gets clipped to 0.

`--gripper_position_scale` (default `0.03`) is an unverified placeholder. Slowly open/close the real gripper while watching your receiver's logged target positions move sensibly between `0` and the finger travel limit, and tune the scale from there.

## Optional: motorbridge-studio live monitoring & debugging

[`motorbridge-gateway`](https://motorbridge.github.io/motorbridge-studio/) is a standalone WebSocket gateway (`ws://127.0.0.1:9002`) that exposes the CAN/serial motor bus to the browser-based motorbridge-studio for live motor-state inspection and manual single-motor testing. It is a separate path from the Python `motorbridge.Controller` used internally by `lerobot-teleoperate` / this bridge (which opens the serial port directly) — it is **not** a required part of the teleop pipeline.

```bash
motorbridge-gateway -- --bind 127.0.0.1:9002 --vendor damiao --transport dm-serial \
    --serial-port /dev/ttyACM0 --serial-baud 921600 \
    --model 4340P --dt-ms 20
```

Then open <https://motorbridge.github.io/motorbridge-studio/> and connect to `ws://127.0.0.1:9002`.

**Serial-port conflict warning:** `motorbridge-gateway` and `lerobot-teleop-sim-bridge` both exclusively open the same serial port given by `--robot.port`. Do not run both against the same port at the same time — stop one before starting the other, or you'll get a port-in-use error or bus contention. Use `motorbridge-gateway` + `motorbridge-studio` for standalone calibration/single-motor debugging outside of a teleoperation session.

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Gripper's logged target position changes but the sim never opens/closes | `gripper_position` went negative and got clipped to the receiver's lower bound | Confirm your follower's zero pose really is "closed" (see Gripper convention above); this bridge already takes the absolute value by default |
| One arm joint is stuck or behaves oddly while the others are fine | The commanded value for that joint exceeds its simulated joint's physical limit | Check that joint's `lowerLimit`/`upperLimit` in your target asset, adjust its sign via `--joint_sign` |
| The whole arm moves the mirrored/opposite direction in sim | `joint_sign` doesn't match your actual wiring/calibration | Flip signs one joint at a time and observe |
| `TypeError: must be called with a dataclass type or instance` on startup | Something added `from __future__ import annotations` to `bridge.py`, which is incompatible with lerobot's `parser.wrap()` runtime type introspection | Do not add that import to `bridge.py` |
| `Missing required field(s) teleop, robot` | `--robot.type=...` / `--teleop.type=...` not provided | Supply the required flags, same as `lerobot-teleoperate` |
| `[error] REBOT_ASSET_ROOT is not set` | The receiver couldn't find the robot USD asset | `export REBOT_ASSET_ROOT=/path/to/reBot-Isaacsim` (or any dir containing `usd/RS-rebot-dev-arm/`) before launching |
| `[error] Isaac Sim python.sh not found` | `ISAACSIM_ROOT` isn't set or doesn't point at a real Isaac Sim install | `export ISAACSIM_ROOT=/path/to/IsaacSim/_build/linux-x86_64/release` |

## License

Apache-2.0 (matches `lerobot`'s license; change if you'd prefer something else).
