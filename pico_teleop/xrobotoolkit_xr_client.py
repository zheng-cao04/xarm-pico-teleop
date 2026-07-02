from __future__ import annotations

import copy
import logging
import threading
import time
from typing import Any

import numpy as np

from pico_xr_client import XRFrame, _matrix_to_quat_xyzw, _quat_xyzw_to_matrix


logger = logging.getLogger("pico_teleop.xrobotoolkit")


class XRoboToolkitXRClient:
    """
    Reads PICO controller state through XRoboToolkit PC Service Python bindings.

    Output poses match PicoXRClient:
        link order: HMD, left controller, right controller
        position frame: x = front, y = left, z = up
        position unit: meters
        quaternion order: xyzw
    """

    _OPENXR_TO_ROBOT = np.array(
        [
            [0.0, 0.0, -1.0],
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ],
        dtype=np.float64,
    )

    def __init__(self, poll_hz: int = 120):
        try:
            import xrobotoolkit_sdk as xrt
        except ImportError as exc:
            raise RuntimeError(
                "XRoboToolkit backend requires xrobotoolkit_sdk. Install and run "
                "XRoboToolkit PC Service / PC-Service-Pybind first."
            ) from exc

        self.xrt = xrt
        self.poll_hz = max(int(poll_hz), 1)
        self._lock = threading.Lock()
        self._has_frame = threading.Event()
        self._latest_frame = None
        self._stop = threading.Event()
        self._frame_id = 0
        self._last_timestamp_ns = None

        self.xrt.init()
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()
        logger.info("XRoboToolkit SDK initialized")

    def shutdown(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        try:
            self.xrt.close()
        except Exception as exc:
            logger.warning("XRoboToolkit SDK close failed: %s", exc)

    def get_frame(self, timeout=None):
        if not self._has_frame.wait(timeout=timeout):
            return None
        with self._lock:
            return copy.deepcopy(self._latest_frame)

    def _poll_loop(self):
        period = 1.0 / self.poll_hz
        last_error_log = 0.0
        while not self._stop.is_set():
            loop_start = time.time()
            try:
                frame = self._read_frame()
                if frame is not None:
                    with self._lock:
                        self._latest_frame = frame
                    self._has_frame.set()
            except Exception as exc:
                now = time.time()
                if now - last_error_log > 2.0:
                    logger.warning("XRoboToolkit read failed: %s", exc)
                    last_error_log = now

            elapsed = time.time() - loop_start
            if elapsed < period:
                time.sleep(period - elapsed)

    def _read_frame(self):
        timestamp_ns = int(self._safe_call(self.xrt.get_time_stamp_ns, 0))
        if timestamp_ns != 0 and timestamp_ns == self._last_timestamp_ns:
            return None

        headset_pose = self._pose_array(self._safe_call(self.xrt.get_headset_pose, None))
        left_pose = self._pose_array(self._safe_call(self.xrt.get_left_controller_pose, None))
        right_pose = self._pose_array(self._safe_call(self.xrt.get_right_controller_pose, None))

        if not self._valid_pose(left_pose) or not self._valid_pose(right_pose):
            return None

        hmd_pos, hmd_quat = self._convert_pose(headset_pose, allow_identity=True)
        left_pos, left_quat = self._convert_pose(left_pose)
        right_pos, right_quat = self._convert_pose(right_pose)

        self._frame_id += 1
        self._last_timestamp_ns = timestamp_ns
        button_states = self._read_buttons(timestamp_ns)

        return XRFrame(
            frame_id=self._frame_id,
            recv_time=time.time(),
            link_pos=np.stack([hmd_pos, left_pos, right_pos], axis=0),
            link_quat_xyzw=np.stack([hmd_quat, left_quat, right_quat], axis=0),
            button_states=button_states,
            body_imu=np.zeros(3, dtype=np.float64),
        )

    def _read_buttons(self, timestamp_ns):
        return {
            "left_stick": tuple(self._safe_call(self.xrt.get_left_axis, [0.0, 0.0])),
            "left_trigger": self._clamp01(self._safe_call(self.xrt.get_left_trigger, 0.0)),
            "left_grip": self._clamp01(self._safe_call(self.xrt.get_left_grip, 0.0)),
            "left_x": int(bool(self._safe_call(self.xrt.get_X_button, False))),
            "left_y": int(bool(self._safe_call(self.xrt.get_Y_button, False))),
            "left_click": int(bool(self._safe_call(self.xrt.get_left_axis_click, False))),
            "left_menu": int(bool(self._safe_call(self.xrt.get_left_menu_button, False))),
            "right_stick": tuple(self._safe_call(self.xrt.get_right_axis, [0.0, 0.0])),
            "right_trigger": self._clamp01(self._safe_call(self.xrt.get_right_trigger, 0.0)),
            "right_grip": self._clamp01(self._safe_call(self.xrt.get_right_grip, 0.0)),
            "right_a": int(bool(self._safe_call(self.xrt.get_A_button, False))),
            "right_b": int(bool(self._safe_call(self.xrt.get_B_button, False))),
            "right_click": int(bool(self._safe_call(self.xrt.get_right_axis_click, False))),
            "right_menu": int(bool(self._safe_call(self.xrt.get_right_menu_button, False))),
            "sdk_timestamp_ns": timestamp_ns,
        }

    @classmethod
    def _convert_pose(cls, pose, allow_identity=False):
        if not cls._valid_pose(pose):
            if allow_identity:
                return np.zeros(3, dtype=np.float64), np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
            raise ValueError(f"invalid XRoboToolkit pose: {pose}")

        pos = cls._OPENXR_TO_ROBOT @ pose[:3]
        rot = _quat_xyzw_to_matrix(pose[3:7])
        rot_robot = cls._OPENXR_TO_ROBOT @ rot @ cls._OPENXR_TO_ROBOT.T
        quat_robot = _matrix_to_quat_xyzw(rot_robot)
        return pos, quat_robot

    @staticmethod
    def _pose_array(raw):
        if raw is None:
            return np.zeros(7, dtype=np.float64)
        pose = np.asarray(raw, dtype=np.float64).reshape(-1)
        if pose.size < 7:
            return np.zeros(7, dtype=np.float64)
        return pose[:7]

    @staticmethod
    def _valid_pose(pose):
        pose = np.asarray(pose, dtype=np.float64)
        return pose.shape == (7,) and np.all(np.isfinite(pose)) and np.linalg.norm(pose[3:7]) > 0.5

    @staticmethod
    def _safe_call(fn, default: Any):
        try:
            return fn()
        except Exception:
            return default

    @staticmethod
    def _clamp01(value):
        return float(min(max(float(value), 0.0), 1.0))
