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
from hand_bridge import GripperBridge, GripperBridgeConfig
from pico_xr_client import PicoXRClient, XRFrame
from transformations import Transformations


logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s")
logger = logging.getLogger("pico_teleop")

DEFAULT_TCP_POSE = (350, 0, 250, np.pi, 0, -np.pi / 2)


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
    stop_robot_on_stop_button: bool = True
    max_frame_age_s: float = 0.25


@dataclass
class RobotConfig:
    robot_ip: str = "192.168.1.127"
    robot_mode: int = 7
    robot_speed: int = 100
    robot_acc: int = 500
    reset_speed: int = 80
    reset_acc: int = 300
    gripper_type: int = 0
    gripper_port: str | None = None
    gripper_speed: int = -1
    gripper_force: int = -1
    start_joints: Tuple[float, ...] = (0, 0, 0, np.pi / 2, 0, np.pi / 2, 0)
    start_joint_speed: float = 0.25
    start_joint_acc: float = 0.5
    start_tcp_pose: Tuple[float, ...] | None = None
    move_to_start: bool = True
    init_gripper_pose: bool = True


@dataclass
class PicoArmTeleopConfig:
    controller: str
    use_gripper: bool = True
    gripper_input: str = "trigger"
    deadman_input: str = "grip"
    position_scale: float = 1.0
    action_scale: float = 1.0
    position_delta_frame: str = "world"
    controller_to_robot_eef: Tuple[float, ...] = (0, 0, 0, 0, 0, 0)
    robot_base_pose: Tuple[float, ...] = DEFAULT_TCP_POSE
    use_current_robot_pose_as_base: bool = True


@dataclass
class SimConfig:
    viewer: str = "plot"
    print_hz: float = 5.0
    trail_size: int = 200
    axis_length: float = 60.0
    mujoco_arm: str = "both"


def _normalize_arm_config(raw_config):
    config = dict(raw_config)
    if "tracker_to_robot_eef" in config and "controller_to_robot_eef" not in config:
        config["controller_to_robot_eef"] = config.pop("tracker_to_robot_eef")
    return config


def _clamp01(value):
    return min(max(float(value), 0.0), 1.0)


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
        self.pose_rpy = list(config.start_tcp_pose or DEFAULT_TCP_POSE)
        self.pose_aa = Transformations.rotation_matrix_to_xyzrxryrz(
            Transformations.xyzrpy_to_rotation_matrix(*self.pose_rpy)
        )
        self.gripper_norm = 0.0
        self.last_action = None
        self.reset_count = 0
        logger.info("%s sim robot initialized at TCP pose %s", self.name, self.pose_rpy)

    def reset_pose(self, pose_rpy):
        self.pose_rpy = list(pose_rpy)
        self.pose_aa = Transformations.rotation_matrix_to_xyzrxryrz(
            Transformations.xyzrpy_to_rotation_matrix(*self.pose_rpy)
        )
        self.last_action = None
        self.reset_count += 1
        logger.info("%s sim robot reset to TCP pose %s", self.name, self.pose_rpy)

    def move_to_start_pose(self):
        self.reset_pose(self.config.start_tcp_pose or DEFAULT_TCP_POSE)
        return 0

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
            self.gripper_norm = _clamp01(action[6])
        return 0

    def send_gripper(self, gripper_norm):
        self.gripper_norm = _clamp01(gripper_norm)
        if self.last_action is not None and len(self.last_action) > 6:
            self.last_action[6] = self.gripper_norm
        return 0

    def emergency_stop(self):
        self.last_action = None
        logger.info("%s sim robot software stop", self.name)
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
                grip = getattr(arm.robot, "gripper_norm", 0.0)
                parts.append(f"{prefix} target=inactive gripper={grip:.2f}")
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
        self.robot_config = robot_config
        self.controller_index = 1 if config.controller == "left" else 2
        self.robot = robot

        self.controller_to_robot_matrix = Transformations.xyzrpy_to_rotation_matrix(
            *self.config.controller_to_robot_eef
        )
        self.robot_base_matrix = Transformations.xyzrpy_to_rotation_matrix(*self.config.robot_base_pose)
        self.begin_controller_robot_matrix = None
        self.begin_controller_pos_mm = None
        self.begin_head_yaw_matrix = None
        if self.config.position_delta_frame == "head":
            self.config.position_delta_frame = "head_yaw"
        if self.config.position_delta_frame not in ("world", "controller", "head_yaw"):
            raise ValueError(f"{name}: position_delta_frame must be 'world', 'controller', or 'head_yaw'")
        if self.config.action_scale < 0:
            raise ValueError(f"{name}: action_scale must be >= 0")
        if self.config.action_scale > 1.0:
            logger.warning("%s: action_scale %.3f amplifies controller translation", self.name, self.config.action_scale)

    def connect_robot(self):
        if self.robot is not None:
            return
        from ufactory_devices.robot import UFRobot

        logger.info("%s: connecting xArm at %s", self.name, self.robot_config.robot_ip)
        self.robot = UFRobot(self.robot_config)
        logger.info("%s: xArm connected", self.name)

    def clear_reference(self):
        self.begin_controller_robot_matrix = None
        self.begin_controller_pos_mm = None
        self.begin_head_yaw_matrix = None
        if self.robot is not None and hasattr(self.robot, "last_action"):
            self.robot.last_action = None

    def pause_output(self):
        if self.robot is not None and hasattr(self.robot, "last_action"):
            self.robot.last_action = None

    def move_robot_to_base_pose(self):
        self.connect_robot()
        if not hasattr(self.robot, "reset_pose"):
            return
        code = self.robot.reset_pose(self.config.robot_base_pose)
        if code not in (None, 0):
            raise RuntimeError(f"{self.name}: failed to reset robot to configured base pose, code={code}")

    def move_robot_to_start_pose(self):
        self.connect_robot()
        move_to_start_pose = getattr(self.robot, "move_to_start_pose", None)
        if not callable(move_to_start_pose):
            raise RuntimeError(f"{self.name}: robot does not expose move_to_start_pose()")
        code = move_to_start_pose()
        if code not in (None, 0):
            raise RuntimeError(f"{self.name}: failed to move robot to start pose, code={code}")

    def reset_reference(self, frame: XRFrame, use_current_robot_pose: bool | None = None, reset_robot_to_base=False):
        self.connect_robot()
        if use_current_robot_pose is None:
            use_current_robot_pose = self.config.use_current_robot_pose_as_base

        if reset_robot_to_base:
            self.move_robot_to_base_pose()

        if use_current_robot_pose:
            code, pose = self.robot.get_position(is_axis_angle=False)
            if code == 0 and pose is not None and len(pose) >= 6:
                self.robot_base_matrix = Transformations.xyzrpy_to_rotation_matrix(*pose[:6])
            else:
                logger.warning("%s: failed to read current robot pose, using configured robot_base_pose", self.name)
                self.robot_base_matrix = Transformations.xyzrpy_to_rotation_matrix(*self.config.robot_base_pose)
        else:
            self.robot_base_matrix = Transformations.xyzrpy_to_rotation_matrix(*self.config.robot_base_pose)
        self.begin_controller_robot_matrix = self._controller_robot_matrix(frame)
        self.begin_controller_pos_mm = self._controller_pos_mm(frame)
        self.begin_head_yaw_matrix, head_yaw, head_yaw_valid = self._head_yaw_matrix(frame)
        base_mode = "current robot pose" if use_current_robot_pose else "configured base pose"
        if self.config.position_delta_frame == "head_yaw":
            if head_yaw_valid:
                logger.info(
                    "%s: teleop reference reset from %s controller using %s, head yaw %.1f deg",
                    self.name,
                    self.config.controller,
                    base_mode,
                    np.degrees(head_yaw),
                )
            else:
                logger.warning(
                    "%s: teleop reference reset with head_yaw mode but no valid HMD pose; using world-aligned yaw",
                    self.name,
                )
        else:
            logger.info(
                "%s: teleop reference reset from %s controller using %s",
                self.name,
                self.config.controller,
                base_mode,
            )

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
        if self.begin_controller_pos_mm is None:
            self.begin_controller_pos_mm = self._controller_pos_mm(frame)

        robot_target_pose = Transformations.tracker_robot_matrix_to_robot_pose(
            self.begin_controller_robot_matrix,
            controller_robot_matrix,
            self.robot_base_matrix,
            is_axis_angle=True,
        )
        action = list(robot_target_pose[:6])
        if self.config.position_delta_frame == "world":
            base_pos = self.robot_base_matrix[:3, 3]
            pos_delta = self._controller_pos_mm(frame) - self.begin_controller_pos_mm
            action[:3] = (base_pos + pos_delta).tolist()
        elif self.config.position_delta_frame == "head_yaw":
            base_pos = self.robot_base_matrix[:3, 3]
            pos_delta = self._controller_pos_mm(frame) - self.begin_controller_pos_mm
            if self.begin_head_yaw_matrix is None:
                self.begin_head_yaw_matrix = self._head_yaw_matrix(frame)[0]
            action[:3] = (base_pos + self.begin_head_yaw_matrix.T @ pos_delta).tolist()
        action[:3] = self._scale_action_position(action[:3]).tolist()
        if self.config.use_gripper:
            action.append(self.gripper_norm_from_frame(frame))
        return action

    def send_action(self, action):
        self.connect_robot()
        return self.robot.send_action(action)

    def gripper_norm_from_frame(self, frame: XRFrame):
        if not self.config.use_gripper:
            return 0.0
        return _clamp01(
            _button_value(
                frame.button_states,
                self.config.gripper_input,
                self.config.controller,
            )
        )

    def send_gripper(self, gripper_norm):
        if not self.config.use_gripper:
            return 0
        if hasattr(self.robot, "send_gripper"):
            return self.robot.send_gripper(gripper_norm)
        return 0

    def current_gripper_norm(self):
        return _clamp01(getattr(self.robot, "gripper_norm", 0.0))

    def emergency_stop(self):
        if self.robot is None:
            return
        self.clear_reference()
        emergency_stop = getattr(self.robot, "emergency_stop", None)
        if not callable(emergency_stop):
            logger.warning("%s: robot does not expose emergency_stop()", self.name)
            return
        code = emergency_stop()
        if code not in (None, 0):
            logger.error("%s: software stop returned code %s", self.name, code)

    def _controller_robot_matrix(self, frame: XRFrame):
        x, y, z = self._controller_pos_mm(frame).tolist()
        quat_xyzw = frame.link_quat_xyzw[self.controller_index]
        return Transformations.tracker_pose_to_robot_matrix(
            x,
            y,
            z,
            quat_xyzw,
            self.controller_to_robot_matrix,
        )

    def _controller_pos_mm(self, frame: XRFrame):
        pos_m = frame.link_pos[self.controller_index]
        return pos_m * 1000.0 * self.config.position_scale

    def _head_yaw_matrix(self, frame: XRFrame):
        hmd_valid = bool(frame.button_states.get("hmd_valid", True))
        quat_xyzw = np.asarray(frame.link_quat_xyzw[0], dtype=np.float64)
        if not hmd_valid or np.linalg.norm(quat_xyzw) < 0.5:
            return np.eye(3), 0.0, False
        rot = Transformations.quaternion_to_rotation_matrix(quat_xyzw)
        yaw = float(np.arctan2(rot[1, 0], rot[0, 0]))
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        yaw_matrix = np.array(
            [
                [cos_yaw, -sin_yaw, 0.0],
                [sin_yaw, cos_yaw, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        return yaw_matrix, yaw, True

    def _scale_action_position(self, target_xyz):
        base_pos = self.robot_base_matrix[:3, 3]
        target_pos = np.asarray(target_xyz, dtype=float)
        return base_pos + (target_pos - base_pos) * self.config.action_scale


class PicoDualArmTeleop:
    def __init__(
        self,
        xr_config: PicoXRConfig,
        left_config: PicoArmTeleopConfig,
        left_robot_config: RobotConfig,
        right_config: PicoArmTeleopConfig,
        right_robot_config: RobotConfig,
        active_arm: str = "both",
        start_at_base: bool = False,
        start_at_initial: bool = False,
        sim_config: SimConfig | None = None,
        gripper_bridge_config: GripperBridgeConfig | None = None,
    ):
        self.xr_config = xr_config
        self.xr_client = None
        left_robot = SimulatedUFRobot("L", left_robot_config) if sim_config is not None else None
        right_robot = SimulatedUFRobot("R", right_robot_config) if sim_config is not None else None
        self.left = PicoArmTeleop("L", left_config, left_robot_config, robot=left_robot)
        self.right = PicoArmTeleop("R", right_config, right_robot_config, robot=right_robot)
        self.active_arm = active_arm
        self.active_arms = self._select_active_arms(active_arm)
        self.start_at_base = start_at_base
        self.start_at_initial = start_at_initial
        self.sim_config = sim_config
        self.sim_viewer = None
        self.gripper_bridge = GripperBridge(gripper_bridge_config or GripperBridgeConfig())
        self._arm_was_enabled = {arm.name: False for arm in (self.left, self.right)}
        self._last_recalibrate_pressed = False
        self._last_stop_pressed = False

    def _select_active_arms(self, active_arm):
        if active_arm == "left":
            return (self.left,)
        if active_arm == "right":
            return (self.right,)
        if active_arm == "both":
            return (self.left, self.right)
        raise ValueError("active_arm must be one of: left, right, both")

    def shutdown(self):
        if self.xr_client is not None:
            self.xr_client.shutdown()
        if self.sim_viewer is not None:
            self.sim_viewer.close()
        self.gripper_bridge.close()

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
        if sim_config.viewer == "mujoco":
            from mujoco_sim_viewer import MujocoSimViewer

            return MujocoSimViewer(sim_config, (self.left, self.right))
        raise ValueError("sim viewer must be one of: plot, console, mujoco, none")

    def _connect_real_robots_if_needed(self):
        if self.sim_config is not None:
            return
        for arm in self.active_arms:
            arm.connect_robot()

    def _software_stop_active_arms(self):
        for arm in self.active_arms:
            try:
                arm.emergency_stop()
            except Exception as exc:
                logger.exception("%s: software stop failed: %s", arm.name, exc)

    def reset_to_base_only(self):
        if self.sim_config is not None:
            raise RuntimeError("--reset-to-base-only is intended for real xArm mode")
        logger.warning("reset-only mode: connecting active arm(s) and moving to configured robot_base_pose")
        self._connect_real_robots_if_needed()
        for arm in self.active_arms:
            arm.move_robot_to_base_pose()
        logger.info("reset-only mode complete")

    def initial_position_only(self):
        if self.sim_config is not None:
            raise RuntimeError("--initial-position-only is intended for real xArm mode")
        logger.warning("initial-position-only mode: connecting active arm(s) and moving to configured start pose")
        self._connect_real_robots_if_needed()
        for arm in self.active_arms:
            arm.move_robot_to_start_pose()
        logger.info("initial-position-only mode complete")

    def run(self):
        if self.xr_client is None:
            self.xr_client = _make_xr_client(self.xr_config)
        sleep_time = 1.0 / self.xr_config.fps
        logger.info("waiting for first XR frame from %s backend", self.xr_config.backend)
        first_frame = self.xr_client.get_frame(timeout=90.0)
        if first_frame is None:
            if self.xr_config.backend.lower() == "udp":
                raise TimeoutError(
                    "No XR frame received. Start the PICO/Unity streamer and check UDP target IP/port."
                )
            raise TimeoutError(
                "No valid XR frame received after 90 seconds. XRoboToolkit returned zero or invalid "
                "controller poses. Restart the XRoboToolkit PICO app / PC Service if this persists."
            )

        if self.sim_config is None:
            logger.info("valid XR frame received; connecting active real xArm(s): %s", self.active_arm)
            self._connect_real_robots_if_needed()
            logger.info("waiting for fresh XR frame after xArm initialization")
            fresh_frame = self.xr_client.get_frame(timeout=5.0)
            if fresh_frame is not None:
                first_frame = fresh_frame

        if self.start_at_base:
            logger.warning("moving active arm(s) to configured robot_base_pose before teleop starts")
            for arm in self.active_arms:
                arm.move_robot_to_base_pose()
            fresh_frame = self.xr_client.get_frame(timeout=5.0)
            if fresh_frame is not None:
                first_frame = fresh_frame
        elif self.start_at_initial:
            logger.warning("moving active arm(s) to configured start pose before teleop starts")
            for arm in self.active_arms:
                arm.move_robot_to_start_pose()
            fresh_frame = self.xr_client.get_frame(timeout=5.0)
            if fresh_frame is not None:
                first_frame = fresh_frame

        for arm in self.active_arms:
            arm.reset_reference(first_frame)
        if self.sim_config is not None and self.sim_viewer is None:
            self.sim_viewer = self._make_sim_viewer(self.sim_config)
        self.gripper_bridge.start()
        logger.info("teleop loop started for active arm(s): %s", self.active_arm)

        while True:
            loop_start = time.time()
            frame = self.xr_client.get_frame(timeout=1.0)
            if frame is None:
                logger.warning("XR frame timeout")
                for arm in self.active_arms:
                    arm.clear_reference()
                    self._arm_was_enabled[arm.name] = False
                continue

            if time.time() - frame.recv_time > self.xr_config.max_frame_age_s:
                for arm in self.active_arms:
                    arm.clear_reference()
                    self._arm_was_enabled[arm.name] = False
                time.sleep(sleep_time)
                continue

            if self._button_rising(frame, self.xr_config.stop_button, "_last_stop_pressed"):
                logger.warning("PICO stop button pressed: stopping selected arm(s) and exiting teleop")
                if self.xr_config.stop_robot_on_stop_button:
                    self._software_stop_active_arms()
                break

            if self._button_rising(frame, self.xr_config.recalibrate_button, "_last_recalibrate_pressed"):
                for arm in self.active_arms:
                    arm.move_robot_to_base_pose()
                fresh_frame = self.xr_client.get_frame(timeout=1.0)
                if fresh_frame is not None:
                    frame = fresh_frame
                for arm in self.active_arms:
                    arm.reset_reference(frame, use_current_robot_pose=False)
                time.sleep(sleep_time)
                continue

            gripper_actions = {
                self.left.name: self.left.gripper_norm_from_frame(frame),
                self.right.name: self.right.gripper_norm_from_frame(frame),
            }

            for arm in self.active_arms:
                gripper_action = gripper_actions[arm.name]
                enabled = arm.enabled(frame, self.xr_config)
                if not enabled:
                    arm.pause_output()
                    self._arm_was_enabled[arm.name] = False
                    code = arm.send_gripper(gripper_action)
                    if code != 0:
                        logger.error("%s gripper command failed with code %s", arm.name, code)
                        return
                    continue

                if not self._arm_was_enabled[arm.name]:
                    # Treat grip as a clutch: controller motion while inactive must not
                    # accumulate into a jump when the user re-enables the arm.
                    arm.reset_reference(frame)
                    self._arm_was_enabled[arm.name] = True
                    code = arm.send_gripper(gripper_action)
                    if code != 0:
                        logger.error("%s gripper command failed with code %s", arm.name, code)
                        return
                    continue

                if arm.begin_controller_robot_matrix is None:
                    arm.reset_reference(frame)
                    code = arm.send_gripper(gripper_action)
                    if code != 0:
                        logger.error("%s gripper command failed with code %s", arm.name, code)
                        return
                    continue
                action = arm.action_from_frame(frame)
                code = arm.send_action(action)
                if code != 0:
                    logger.error("%s robot command failed with code %s", arm.name, code)
                    return

            self.gripper_bridge.update(
                gripper_actions[self.left.name],
                gripper_actions[self.right.name],
                self.left.current_gripper_norm(),
                self.right.current_gripper_norm(),
            )

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


def _parse_bool(value):
    if isinstance(value, bool):
        return value
    normalized = value.strip().lower()
    if normalized in ("true", "1", "yes", "y"):
        return True
    if normalized in ("false", "0", "no", "n"):
        return False
    raise argparse.ArgumentTypeError("expected true or false")


def main():
    parser = argparse.ArgumentParser(description="PICO/OpenXR dual xArm teleoperation")
    parser.add_argument("-c", "--config", type=str, required=True, help="configuration file path")
    parser.add_argument(
        "--sim",
        type=_parse_bool,
        required=True,
        metavar="{true,false}",
        help="required safety mode: true runs simulation, false connects real xArm hardware",
    )
    parser.add_argument(
        "--sim-viewer",
        choices=["plot", "console", "mujoco", "none"],
        default="plot",
        help="simulation output mode",
    )
    parser.add_argument("--sim-print-hz", type=float, default=5.0, help="console sim print rate")
    parser.add_argument(
        "--sim-mujoco-arm",
        choices=["left", "right", "both"],
        default="both",
        help="MuJoCo camera/focus hint; both xArm models are displayed",
    )
    parser.add_argument(
        "--arm",
        choices=["left", "right", "both"],
        default="both",
        help="select which arm(s) to connect and teleoperate",
    )
    parser.add_argument(
        "--no-start-move",
        action="store_true",
        help="real xArm mode only: connect/init arms but skip moving to configured start_joints/start_tcp_pose",
    )
    parser.add_argument(
        "--start-at-base",
        action="store_true",
        help="move selected arm(s) softly to TeleoperatorConfig.robot_base_pose before teleop starts",
    )
    parser.add_argument(
        "--start-at-initial",
        action="store_true",
        help="move selected arm(s) to RobotConfig.start_joints/start_tcp_pose before teleop starts",
    )
    parser.add_argument(
        "--reset-to-base-only",
        action="store_true",
        help="real xArm mode only: move selected arm(s) to robot_base_pose and exit without starting XR teleop",
    )
    parser.add_argument(
        "--initial-position-only",
        action="store_true",
        help="real xArm mode only: move selected arm(s) to RobotConfig.start_joints/start_tcp_pose and exit",
    )
    parser.add_argument("--reset-speed", type=int, default=None, help="override RobotConfig.reset_speed")
    parser.add_argument("--reset-acc", type=int, default=None, help="override RobotConfig.reset_acc")
    parser.add_argument("--start-joint-speed", type=float, default=None, help="override RobotConfig.start_joint_speed")
    parser.add_argument("--start-joint-acc", type=float, default=None, help="override RobotConfig.start_joint_acc")
    parser.add_argument(
        "--no-gripper-init",
        action="store_true",
        help="real xArm mode only: do not move grippers to their configured open pose during initialization",
    )
    parser.add_argument(
        "--hand-driver",
        choices=["none", "wuji_hand", "changingtek"],
        default=None,
        help="override GripperBridge.driver for a standalone external hand",
    )
    parser.add_argument(
        "--hand-side",
        choices=["left", "right", "both"],
        default=None,
        help="override GripperBridge.side",
    )
    parser.add_argument("--hand-serial-number", type=str, default=None, help="single external hand serial number")
    parser.add_argument("--hand-left-serial-number", type=str, default=None, help="left external hand serial number")
    parser.add_argument("--hand-right-serial-number", type=str, default=None, help="right external hand serial number")
    parser.add_argument(
        "--hand-close-scale",
        type=float,
        default=None,
        help="override external hand closure range scale in [0, 1]",
    )
    parser.add_argument(
        "--hand-dummy",
        type=_parse_bool,
        default=None,
        metavar="{true,false}",
        help="run external hand bridge in dummy mode",
    )
    args = parser.parse_args()

    real_motion_modes = [
        args.start_at_base,
        args.start_at_initial,
        args.reset_to_base_only,
        args.initial_position_only,
    ]
    if sum(bool(v) for v in real_motion_modes) > 1:
        parser.error("choose only one of --start-at-base, --start-at-initial, --reset-to-base-only, --initial-position-only")
    if args.sim and any(real_motion_modes):
        parser.error("real-arm startup/reset motion options require --sim false")
    for name in ("reset_speed", "reset_acc", "start_joint_speed", "start_joint_acc"):
        value = getattr(args, name)
        if value is not None and value <= 0:
            parser.error(f"--{name.replace('_', '-')} must be positive")
    if args.hand_close_scale is not None and not 0.0 <= args.hand_close_scale <= 1.0:
        parser.error("--hand-close-scale must be in [0, 1]")

    config = _load_config(args.config)
    xr_config = PicoXRConfig(**config.get("XRConfig", {}))

    left_config_raw = config["L"]
    right_config_raw = config["R"]
    left_robot_config = RobotConfig(**left_config_raw["RobotConfig"])
    right_robot_config = RobotConfig(**right_config_raw["RobotConfig"])
    if (
        args.no_start_move
        or args.start_at_base
        or args.start_at_initial
        or args.reset_to_base_only
        or args.initial_position_only
    ):
        left_robot_config.move_to_start = False
        right_robot_config.move_to_start = False
    if args.no_gripper_init:
        left_robot_config.init_gripper_pose = False
        right_robot_config.init_gripper_pose = False
    if args.reset_speed is not None:
        left_robot_config.reset_speed = args.reset_speed
        right_robot_config.reset_speed = args.reset_speed
    if args.reset_acc is not None:
        left_robot_config.reset_acc = args.reset_acc
        right_robot_config.reset_acc = args.reset_acc
    if args.start_joint_speed is not None:
        left_robot_config.start_joint_speed = args.start_joint_speed
        right_robot_config.start_joint_speed = args.start_joint_speed
    if args.start_joint_acc is not None:
        left_robot_config.start_joint_acc = args.start_joint_acc
        right_robot_config.start_joint_acc = args.start_joint_acc
    left_teleop_config = PicoArmTeleopConfig(
        **_normalize_arm_config(left_config_raw["TeleoperatorConfig"])
    )
    right_teleop_config = PicoArmTeleopConfig(
        **_normalize_arm_config(right_config_raw["TeleoperatorConfig"])
    )
    gripper_bridge_config = GripperBridgeConfig(**config.get("GripperBridge", {}))
    if args.hand_driver is not None:
        gripper_bridge_config.driver = args.hand_driver
    if args.hand_side is not None:
        gripper_bridge_config.side = args.hand_side
    if args.hand_serial_number is not None:
        gripper_bridge_config.serial_number = args.hand_serial_number
    if args.hand_left_serial_number is not None:
        gripper_bridge_config.left_serial_number = args.hand_left_serial_number
    if args.hand_right_serial_number is not None:
        gripper_bridge_config.right_serial_number = args.hand_right_serial_number
    if args.hand_close_scale is not None:
        gripper_bridge_config.close_scale = args.hand_close_scale
    if args.hand_dummy is not None:
        gripper_bridge_config.is_dummy = args.hand_dummy

    teleop = PicoDualArmTeleop(
        xr_config,
        left_teleop_config,
        left_robot_config,
        right_teleop_config,
        right_robot_config,
        active_arm=args.arm,
        start_at_base=args.start_at_base,
        start_at_initial=args.start_at_initial,
        sim_config=SimConfig(
            viewer=args.sim_viewer,
            print_hz=args.sim_print_hz,
            mujoco_arm=args.sim_mujoco_arm,
        )
        if args.sim
        else None,
        gripper_bridge_config=gripper_bridge_config,
    )

    try:
        mode = "SIM" if args.sim else "REAL ROBOT"
        print(f"\n********** Test PICO Teleop With xArm ({mode}, arm={args.arm}) **********")
        if not args.sim:
            print(f"Real mode will connect selected xArm(s) after Enter and after a valid XR frame is available: {args.arm}.")
            if args.start_at_base:
                print("Startup base move is enabled; selected arm(s) will move softly to robot_base_pose before teleop.")
            elif args.start_at_initial:
                print("Startup initial-position move is enabled; selected arm(s) will move to start_joints before teleop.")
            elif args.reset_to_base_only:
                print("Reset-only mode is enabled; selected arm(s) will move softly to robot_base_pose, then the program exits.")
            elif args.initial_position_only:
                print("Initial-position-only mode is enabled; selected arm(s) will move to start_joints, then the program exits.")
            elif args.no_start_move:
                print("Start motion is disabled for this run; selected arm(s) will stay at their current poses on connect.")
            else:
                print("Start motion is enabled; selected arm(s) may move to configured start_joints/start_tcp_pose.")
            print("Keep physical E-stop reachable. Right-controller B sends software stop to selected arm(s) and exits.")
            print("Right-controller A moves selected arm(s) to robot_base_pose and recalibrates.")
        if args.reset_speed is not None or args.reset_acc is not None:
            print(f"Reset override: speed={left_robot_config.reset_speed}, acc={left_robot_config.reset_acc}")
        if args.start_joint_speed is not None or args.start_joint_acc is not None:
            print(
                "Initial joint override: "
                f"speed={left_robot_config.start_joint_speed}, acc={left_robot_config.start_joint_acc}"
            )
        if gripper_bridge_config.driver != "none":
            print(
                "External hand bridge: "
                f"driver={gripper_bridge_config.driver}, side={gripper_bridge_config.side}, "
                f"close_scale={gripper_bridge_config.close_scale}"
            )
        input("Enter to control robot with PICO teleop >>> ")
        print("\n********** Teleop Control Loop Start **********")
        if args.reset_to_base_only:
            teleop.reset_to_base_only()
        elif args.initial_position_only:
            teleop.initial_position_only()
        else:
            teleop.run()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        teleop.shutdown()


if __name__ == "__main__":
    main()
