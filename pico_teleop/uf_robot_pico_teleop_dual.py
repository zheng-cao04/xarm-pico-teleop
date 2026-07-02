#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import logging
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ufactory_devices", "umi", "vive_tracker"))
from pico_xr_client import PicoXRClient, XRFrame
from transformations import Transformations


logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s")
logger = logging.getLogger("pico_teleop")


@dataclass
class PicoXRConfig:
    backend: str = "udp"
    udp_host: str = "0.0.0.0"
    udp_port: int = 5005
    xrobotoolkit_poll_hz: int = 120
    fps: int = 50
    use_deadman: bool = True
    deadman_threshold: float = 0.5
    recalibrate_button: str = "right_a"
    stop_button: str = "right_b"
    max_frame_age_s: float = 0.25


@dataclass
class RobotConfig:
    robot_ip: str = "192.168.1.127"
    robot_mode: int = 7
    robot_speed: int = 100
    robot_acc: int = 500
    gripper_type: int = 0
    gripper_port: str | None = None
    gripper_speed: int = -1
    gripper_force: int = -1
    start_joints: Tuple[float, ...] = (0, 0, 0, np.pi / 2, 0, np.pi / 2, 0)
    start_tcp_pose: Tuple[float, ...] | None = None


@dataclass
class PicoArmTeleopConfig:
    controller: str
    use_gripper: bool = True
    gripper_input: str = "trigger"
    deadman_input: str = "grip"
    position_scale: float = 1.0
    controller_to_robot_eef: Tuple[float, ...] = (0, 0, 0, 0, 0, 0)
    robot_base_pose: Tuple[float, ...] = (350, 0, 250, -np.pi / 2, 0, -np.pi / 2)
    use_current_robot_pose_as_base: bool = True


@dataclass
class SimConfig:
    viewer: str = "plot"
    print_hz: float = 5.0
    trail_size: int = 200
    axis_length: float = 60.0


def _normalize_arm_config(raw_config):
    config = dict(raw_config)
    if "tracker_to_robot_eef" in config and "controller_to_robot_eef" not in config:
        config["controller_to_robot_eef"] = config.pop("tracker_to_robot_eef")
    return config


def _button_value(button_states, name, controller):
    if not name:
        return 0.0
    key = name
    if name == "trigger":
        key = f"{controller}_trigger"
    elif name == "grip":
        key = f"{controller}_grip"
    elif name == "primary":
        key = "left_x" if controller == "left" else "right_a"
    elif name == "secondary":
        key = "left_y" if controller == "left" else "right_b"
    elif name == "click":
        key = f"{controller}_click"
    value = button_states.get(key, 0.0)
    if isinstance(value, (tuple, list)):
        return float(np.linalg.norm(value))
    return float(value)


def _make_xr_client(config: PicoXRConfig):
    backend = config.backend.lower()
    if backend == "udp":
        return PicoXRClient(config.udp_host, config.udp_port)
    if backend in ("xrobotoolkit", "xrt"):
        from xrobotoolkit_xr_client import XRoboToolkitXRClient

        return XRoboToolkitXRClient(config.xrobotoolkit_poll_hz)
    raise ValueError("XRConfig.backend must be one of: udp, xrobotoolkit")


class SimulatedUFRobot:
    def __init__(self, name, config: RobotConfig):
        self.name = name
        self.config = config
        self.pose_rpy = list(config.start_tcp_pose or (350, 0, 250, -np.pi / 2, 0, -np.pi / 2))
        self.pose_aa = Transformations.rotation_matrix_to_xyzrxryrz(
            Transformations.xyzrpy_to_rotation_matrix(*self.pose_rpy)
        )
        self.gripper_norm = 0.0
        self.last_action = None
        logger.info("%s sim robot initialized at TCP pose %s", self.name, self.pose_rpy)

    def get_position(self, is_axis_angle=False):
        if is_axis_angle:
            return 0, list(self.pose_aa)
        return 0, list(self.pose_rpy)

    def send_action(self, action):
        self.last_action = list(action)
        self.pose_aa = list(action[:6])
        self.pose_rpy = Transformations.rotation_matrix_to_xyzrpy(
            Transformations.xyzrxryrz_to_rotation_matrix(*self.pose_aa)
        )
        if len(action) > 6:
            self.gripper_norm = float(action[6])
        return 0


class ConsoleSimViewer:
    def __init__(self, config: SimConfig):
        self.config = config
        self._last_print = 0.0

    def update(self, frame, arms):
        now = time.time()
        period = 1.0 / max(self.config.print_hz, 0.1)
        if now - self._last_print < period:
            return
        self._last_print = now
        parts = [f"frame={frame.frame_id}"]
        for arm in arms:
            ctrl_pos_mm = frame.link_pos[arm.controller_index] * 1000.0
            ctrl_xyz = ", ".join(f"{v:7.1f}" for v in ctrl_pos_mm)
            deadman = _button_value(frame.button_states, arm.config.deadman_input, arm.config.controller)
            trigger = _button_value(frame.button_states, arm.config.gripper_input, arm.config.controller)
            prefix = (
                f"{arm.name}: ctrl=[{ctrl_xyz}] "
                f"{arm.config.deadman_input}={deadman:.2f} "
                f"{arm.config.gripper_input}={trigger:.2f}"
            )
            action = arm.robot.last_action
            if action is None:
                parts.append(f"{prefix} target=inactive")
                continue
            xyz = ", ".join(f"{v:7.1f}" for v in action[:3])
            aa = ", ".join(f"{v:+.2f}" for v in action[3:6])
            grip = arm.robot.gripper_norm
            parts.append(f"{prefix} target=[{xyz}] aa=[{aa}] gripper={grip:.2f}")
        print(" | ".join(parts), flush=True)

    def close(self):
        pass


class MatplotlibSimViewer(ConsoleSimViewer):
    def __init__(self, config: SimConfig, arms):
        super().__init__(config)
        try:
            import matplotlib.pyplot as plt
        except ImportError as exc:
            raise RuntimeError("matplotlib is not installed") from exc

        self.plt = plt
        self.trails = {arm.name: [] for arm in arms}
        self.colors = {"L": "tab:blue", "R": "tab:orange"}
        plt.ion()
        self.fig = plt.figure("PICO xArm teleop sim")
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.fig.show()

    def update(self, frame, arms):
        super().update(frame, arms)
        self.ax.clear()
        self.ax.set_title("PICO dual xArm teleop sim")
        self.ax.set_xlabel("x front (mm)")
        self.ax.set_ylabel("y left (mm)")
        self.ax.set_zlabel("z up (mm)")
        self.ax.set_xlim(0, 800)
        self.ax.set_ylim(-500, 500)
        self.ax.set_zlim(0, 800)

        for arm in arms:
            action = arm.robot.last_action
            base = np.asarray(Transformations.rotation_matrix_to_xyzrpy(arm.robot_base_matrix)[:3])
            color = self.colors.get(arm.name, "black")
            self.ax.scatter([base[0]], [base[1]], [base[2]], c=color, marker="x", s=50)
            self.ax.text(base[0], base[1], base[2], f"{arm.name} base")
            if action is None:
                continue

            pos = np.asarray(action[:3], dtype=float)
            self.trails[arm.name].append(pos)
            self.trails[arm.name] = self.trails[arm.name][-self.config.trail_size :]
            trail = np.asarray(self.trails[arm.name])
            if len(trail) > 1:
                self.ax.plot(trail[:, 0], trail[:, 1], trail[:, 2], color=color, alpha=0.45)

            self.ax.plot([base[0], pos[0]], [base[1], pos[1]], [base[2], pos[2]], color=color, alpha=0.25)
            self.ax.scatter([pos[0]], [pos[1]], [pos[2]], c=color, s=45)
            self.ax.text(pos[0], pos[1], pos[2], f"{arm.name} TCP {arm.robot.gripper_norm:.2f}")
            self._draw_axes(pos, action[3:6], self.config.axis_length)

        self.fig.canvas.draw_idle()
        self.plt.pause(0.001)

    def _draw_axes(self, pos, axis_angle, length):
        rot = Transformations.rxryrz_to_rotation_matrix(*axis_angle)
        colors = ("r", "g", "b")
        for idx, color in enumerate(colors):
            endpoint = pos + rot[:, idx] * length
            self.ax.plot(
                [pos[0], endpoint[0]],
                [pos[1], endpoint[1]],
                [pos[2], endpoint[2]],
                color=color,
                linewidth=2,
            )

    def close(self):
        self.plt.ioff()


class PicoArmTeleop:
    def __init__(self, name, config: PicoArmTeleopConfig, robot_config: RobotConfig, robot=None):
        if config.controller not in ("left", "right"):
            raise ValueError(f"{name}: controller must be 'left' or 'right'")
        self.name = name
        self.config = config
        self.controller_index = 1 if config.controller == "left" else 2
        if robot is not None:
            self.robot = robot
        else:
            from ufactory_devices.robot import UFRobot

            self.robot = UFRobot(robot_config)

        self.controller_to_robot_matrix = Transformations.xyzrpy_to_rotation_matrix(
            *self.config.controller_to_robot_eef
        )
        self.robot_base_matrix = Transformations.xyzrpy_to_rotation_matrix(*self.config.robot_base_pose)
        self.begin_controller_robot_matrix = None

    def clear_reference(self):
        self.begin_controller_robot_matrix = None
        if hasattr(self.robot, "last_action"):
            self.robot.last_action = None

    def reset_reference(self, frame: XRFrame):
        if self.config.use_current_robot_pose_as_base:
            code, pose = self.robot.get_position(is_axis_angle=False)
            if code == 0 and pose is not None and len(pose) >= 6:
                self.robot_base_matrix = Transformations.xyzrpy_to_rotation_matrix(*pose[:6])
            else:
                logger.warning("%s: failed to read current robot pose, using configured robot_base_pose", self.name)
        self.begin_controller_robot_matrix = self._controller_robot_matrix(frame)
        logger.info("%s: teleop reference reset from %s controller", self.name, self.config.controller)

    def enabled(self, frame: XRFrame, xr_config: PicoXRConfig):
        if not xr_config.use_deadman:
            return True
        return _button_value(
            frame.button_states,
            self.config.deadman_input,
            self.config.controller,
        ) >= xr_config.deadman_threshold

    def action_from_frame(self, frame: XRFrame):
        controller_robot_matrix = self._controller_robot_matrix(frame)
        if self.begin_controller_robot_matrix is None:
            self.begin_controller_robot_matrix = controller_robot_matrix

        robot_target_pose = Transformations.tracker_robot_matrix_to_robot_pose(
            self.begin_controller_robot_matrix,
            controller_robot_matrix,
            self.robot_base_matrix,
            is_axis_angle=True,
        )
        action = list(robot_target_pose[:6])
        if self.config.use_gripper:
            gripper_norm = _button_value(
                frame.button_states,
                self.config.gripper_input,
                self.config.controller,
            )
            action.append(min(max(gripper_norm, 0.0), 1.0))
        return action

    def send_action(self, action):
        return self.robot.send_action(action)

    def _controller_robot_matrix(self, frame: XRFrame):
        pos_m = frame.link_pos[self.controller_index]
        quat_xyzw = frame.link_quat_xyzw[self.controller_index]
        x, y, z = (pos_m * 1000.0 * self.config.position_scale).tolist()
        return Transformations.tracker_pose_to_robot_matrix(
            x,
            y,
            z,
            quat_xyzw,
            self.controller_to_robot_matrix,
        )


class PicoDualArmTeleop:
    def __init__(
        self,
        xr_config: PicoXRConfig,
        left_config: PicoArmTeleopConfig,
        left_robot_config: RobotConfig,
        right_config: PicoArmTeleopConfig,
        right_robot_config: RobotConfig,
        sim_config: SimConfig | None = None,
    ):
        self.xr_config = xr_config
        self.xr_client = _make_xr_client(xr_config)
        left_robot = SimulatedUFRobot("L", left_robot_config) if sim_config is not None else None
        right_robot = SimulatedUFRobot("R", right_robot_config) if sim_config is not None else None
        self.left = PicoArmTeleop("L", left_config, left_robot_config, robot=left_robot)
        self.right = PicoArmTeleop("R", right_config, right_robot_config, robot=right_robot)
        self.sim_viewer = self._make_sim_viewer(sim_config) if sim_config is not None else None
        self._last_recalibrate_pressed = False
        self._last_stop_pressed = False

    def shutdown(self):
        self.xr_client.shutdown()
        if self.sim_viewer is not None:
            self.sim_viewer.close()

    def _make_sim_viewer(self, sim_config):
        if sim_config.viewer == "none":
            return None
        if sim_config.viewer == "console":
            return ConsoleSimViewer(sim_config)
        if sim_config.viewer == "plot":
            try:
                return MatplotlibSimViewer(sim_config, (self.left, self.right))
            except RuntimeError as exc:
                logger.warning("%s; falling back to console sim viewer", exc)
                return ConsoleSimViewer(sim_config)
        raise ValueError("sim viewer must be one of: plot, console, none")

    def run(self):
        sleep_time = 1.0 / self.xr_config.fps
        logger.info("waiting for first XR frame from %s backend", self.xr_config.backend)
        first_frame = self.xr_client.get_frame(timeout=10.0)
        if first_frame is None:
            if self.xr_config.backend.lower() == "udp":
                raise TimeoutError(
                    "No XR frame received. Start the PICO/Unity streamer and check UDP target IP/port."
                )
            raise TimeoutError(
                "No XR frame received. Start the XRoboToolkit PICO app, connect it to PC Service, "
                "and check xrobotoolkit_sdk installation."
            )

        self.left.reset_reference(first_frame)
        self.right.reset_reference(first_frame)
        logger.info("teleop loop started")

        while True:
            loop_start = time.time()
            frame = self.xr_client.get_frame(timeout=1.0)
            if frame is None:
                logger.warning("XR frame timeout")
                self.left.clear_reference()
                self.right.clear_reference()
                continue

            if time.time() - frame.recv_time > self.xr_config.max_frame_age_s:
                self.left.clear_reference()
                self.right.clear_reference()
                time.sleep(sleep_time)
                continue

            if self._button_rising(frame, self.xr_config.stop_button, "_last_stop_pressed"):
                logger.info("stop button pressed")
                break

            if self._button_rising(frame, self.xr_config.recalibrate_button, "_last_recalibrate_pressed"):
                self.left.reset_reference(frame)
                self.right.reset_reference(frame)

            for arm in (self.left, self.right):
                if not arm.enabled(frame, self.xr_config):
                    arm.clear_reference()
                    continue
                if arm.begin_controller_robot_matrix is None:
                    arm.reset_reference(frame)
                action = arm.action_from_frame(frame)
                code = arm.send_action(action)
                if code != 0:
                    logger.error("%s robot command failed with code %s", arm.name, code)
                    return

            if self.sim_viewer is not None:
                self.sim_viewer.update(frame, (self.left, self.right))

            elapsed = time.time() - loop_start
            if elapsed < sleep_time:
                time.sleep(sleep_time - elapsed)

    def _button_rising(self, frame, button_name, state_attr):
        if not button_name:
            return False
        pressed = _button_value(frame.button_states, button_name, "right") > 0.5
        last_pressed = getattr(self, state_attr)
        setattr(self, state_attr, pressed)
        return pressed and not last_pressed


def _load_config(path):
    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError("PyYAML is required to load config files. Install it with: pip install pyyaml") from exc
    with open(Path(path).expanduser(), "r") as f:
        return yaml.safe_load(f)


def main():
    parser = argparse.ArgumentParser(description="PICO/OpenXR dual xArm teleoperation")
    parser.add_argument("-c", "--config", type=str, required=True, help="configuration file path")
    parser.add_argument("--sim", action="store_true", help="run without connecting to real xArm")
    parser.add_argument(
        "--sim-viewer",
        choices=["plot", "console", "none"],
        default="plot",
        help="simulation output mode",
    )
    parser.add_argument("--sim-print-hz", type=float, default=5.0, help="console sim print rate")
    args = parser.parse_args()

    config = _load_config(args.config)
    xr_config = PicoXRConfig(**config.get("XRConfig", {}))

    left_config_raw = config["L"]
    right_config_raw = config["R"]
    left_robot_config = RobotConfig(**left_config_raw["RobotConfig"])
    right_robot_config = RobotConfig(**right_config_raw["RobotConfig"])
    left_teleop_config = PicoArmTeleopConfig(
        **_normalize_arm_config(left_config_raw["TeleoperatorConfig"])
    )
    right_teleop_config = PicoArmTeleopConfig(
        **_normalize_arm_config(right_config_raw["TeleoperatorConfig"])
    )

    teleop = PicoDualArmTeleop(
        xr_config,
        left_teleop_config,
        left_robot_config,
        right_teleop_config,
        right_robot_config,
        sim_config=SimConfig(viewer=args.sim_viewer, print_hz=args.sim_print_hz) if args.sim else None,
    )

    try:
        mode = "SIM" if args.sim else "REAL ROBOT"
        print(f"\n********** Test PICO Teleop With Dual xArm ({mode}) **********")
        input("Enter to control robot with PICO teleop >>> ")
        print("\n********** Teleop Control Loop Start **********")
        teleop.run()
    finally:
        teleop.shutdown()


if __name__ == "__main__":
    main()
