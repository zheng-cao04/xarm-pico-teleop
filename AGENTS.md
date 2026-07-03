# AGENTS.md

Guidance for coding agents working in this repository.

## Project Focus

This repo contains teleoperation code for UFactory xArm robots. The current active work is in `pico_teleop/`, especially PICO 4 Ultra controller input through XRoboToolkit and dual-arm xArm simulation.

## PICO/XRoboToolkit Simulation Workflow

Use simulation first unless the user explicitly asks to run real xArm hardware.

```bash
cd pico_teleop
pip install -r requirements-sim.txt

python inspect_xrobotoolkit_stream.py --hz 5

python uf_robot_pico_teleop_dual.py \
  --config config/xarm7_xrobotoolkit_teleop_dual.yaml \
  --sim true \
  --sim-viewer mujoco \
  --sim-mujoco-arm both
```

Expected stream health:

- `ts` is nonzero and increasing.
- Left/right controller positions are finite, not all zero.
- `grip`, `trigger`, `A/B`, and `X/Y` values change when operated.

## Teleop Semantics

- XRoboToolkit poses are converted to robot world convention `x=front, y=left, z=up`.
- The active xArm7 config maps controller translation in the headset-yaw frame with `position_delta_frame: "head_yaw"`. Each reference reset captures current HMD yaw, so the operator's facing direction at calibration maps to robot `+x`; operator-left maps to robot `+y`; up stays robot `+z`.
- `action_scale` scales final robot translation around the calibrated base pose. Keep it small, for example `0.2`, for first real-arm tests.
- Wrist rotation remains a relative orientation delta from the calibrated controller pose.
- Grip is both deadman and clutch. Releasing grip pauses output; the next grip press re-anchors that arm to the current controller pose, current HMD yaw, and current TCP pose before sending motion, so inactive controller motion cannot accumulate into a jump.
- Trigger controls the gripper value independently of the grip deadman; releasing grip pauses TCP motion commands only.
- Right-controller `A` recalibrates both arms and resets simulation robot state to the configured `robot_base_pose`, not the current TCP.
- Right-controller `B` is the default software stop. With `stop_robot_on_stop_button: true`, real mode calls xArm SDK `emergency_stop()` on selected arm(s), clears references, and exits the teleop loop. It is not a replacement for the physical E-stop.

## Real xArm Bring-Up

Do not jump straight from simulation to live teleop. Use the no-motion connection checker first:

```bash
cd pico_teleop
python check_xarm_connection.py --config config/xarm7_xrobotoolkit_teleop_dual.yaml
```

For the first live run, prefer:

```bash
python check_xarm_connection.py --config config/xarm7_xrobotoolkit_teleop_dual.yaml --arm left

python uf_robot_pico_teleop_dual.py \
  --config config/xarm7_xrobotoolkit_teleop_dual.yaml \
  --sim false \
  --arm left \
  --no-start-move \
  --no-gripper-init
```

Real mode waits for a valid XR frame before constructing `UFRobot` objects. `--arm left|right|both` selects which arm(s) to connect and command, so single-arm bring-up can leave the other arm disconnected. `--no-start-move` skips the automatic move to `start_joints` / `start_tcp_pose`; initial teleop references then use current TCP poses. Right-controller `A` calls `UFRobot.reset_pose()` only for selected real arms, moving them to `robot_base_pose` with `reset_speed` / `reset_acc`, then recalibrates references.

The active xArm7 XRoboToolkit config uses xArm Studio's "Initial Position" as all-zero `start_joints`, with a measured TCP near `[205.05, -0.055, 119.202, -3.1415, 0.0001, 0.0]`. Use `--initial-position-only` to test that joint-space move without XR teleop, and `--start-at-initial` to do it before teleop starts. `start_joint_speed` / `start_joint_acc` control that joint-space move.

Use `--start-at-base` when the selected real arm should softly return to `TeleoperatorConfig.robot_base_pose` before teleop starts. This uses `reset_speed` / `reset_acc` and automatically disables the older startup move to `start_joints`.

Use `--reset-to-base-only --reset-speed <mm/s> --reset-acc <mm/s^2>` to test the reset motion without starting XR teleop. The active xArm7 XRoboToolkit config intentionally keeps `reset_speed` / `reset_acc` low for first real tests.

## Default TCP Pose

The example default TCP pose is:

```text
[350, 0, 250, 3.141593, 0, -1.570796]
```

This points the TCP/tool Z axis vertically down toward the ground. Keep xArm6/xArm7 PICO and XRoboToolkit config defaults consistent unless changing the pose convention intentionally.

## MuJoCo Viewer

`--sim-viewer mujoco` uses `pico_teleop/mujoco_sim_viewer.py`.

- It builds a dual xArm7 scene from `robot_descriptions.xarm7_mj_description`.
- Both arms are displayed by default with small visual Y offsets to avoid overlap.
- `--sim-mujoco-arm left|right|both` is currently a camera/focus hint; it should not hide either arm.
- The viewer uses pose IK, not position-only IK, so visible TCP orientation should follow the target frame.

## Verification

At minimum, run:

```bash
python -m py_compile \
  pico_teleop/uf_robot_pico_teleop_dual.py \
  pico_teleop/xrobotoolkit_xr_client.py \
  pico_teleop/mujoco_sim_viewer.py
```

When changing MuJoCo logic, also run a headless smoke test that loads the generated dual model and confirms both TCP Z axes are approximately world `-Z` at the default pose.
