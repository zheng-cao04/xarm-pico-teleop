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
  --sim \
  --sim-viewer mujoco \
  --sim-mujoco-arm both
```

Expected stream health:

- `ts` is nonzero and increasing.
- Left/right controller positions are finite, not all zero.
- `grip`, `trigger`, `A/B`, and `X/Y` values change when operated.

## Teleop Semantics

- XRoboToolkit poses are converted to robot world convention `x=front, y=left, z=up`.
- Controller translation is mapped as world-frame delta by default with `position_delta_frame: "world"`.
- Wrist rotation remains a relative orientation delta from the calibrated controller pose.
- Grip is the deadman input. Releasing grip pauses output but keeps the calibration reference.
- Trigger controls the gripper value independently of the grip deadman; releasing grip pauses TCP motion commands only.
- Right-controller `A` recalibrates both arms and resets simulation robot state to the configured `robot_base_pose`, not the current TCP.
- Right-controller `B` stops the teleop loop.

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
