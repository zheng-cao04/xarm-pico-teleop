from __future__ import annotations

import copy
import socket
import threading
import time
from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass
class XRFrame:
    frame_id: int
    recv_time: float
    link_pos: np.ndarray
    link_quat_xyzw: np.ndarray
    button_states: dict[str, Any]
    body_imu: np.ndarray


def _quat_xyzw_to_matrix(q):
    q = np.asarray(q, dtype=np.float64)
    norm = np.linalg.norm(q)
    if norm < 1e-8:
        return np.eye(3)
    x, y, z, w = q / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _matrix_to_quat_xyzw(matrix):
    m = np.asarray(matrix, dtype=np.float64)
    trace = np.trace(m)
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(max(1.0 + m[0, 0] - m[1, 1] - m[2, 2], 1e-12))
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(max(1.0 + m[1, 1] - m[0, 0] - m[2, 2], 1e-12))
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(max(1.0 + m[2, 2] - m[0, 0] - m[1, 1], 1e-12))
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w], dtype=np.float64)
    return q / max(np.linalg.norm(q), 1e-8)


def pos_unity_to_robot(pos):
    pos = np.asarray(pos, dtype=np.float64)
    x_u, y_u, z_u = pos[..., 0], pos[..., 1], pos[..., 2]
    return np.stack([z_u, -x_u, y_u], axis=-1)


def quat_unity_to_robot_xyzw(quat_xyzw):
    quat_xyzw = np.asarray(quat_xyzw, dtype=np.float64)
    axis_change = np.array(
        [
            [0.0, -1.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
        ],
        dtype=np.float64,
    )
    flat = quat_xyzw.reshape(-1, 4)
    converted = []
    for q in flat:
        rot_unity = _quat_xyzw_to_matrix(q)
        rot_robot = axis_change.T @ rot_unity @ axis_change
        converted.append(_matrix_to_quat_xyzw(rot_robot))
    return np.asarray(converted, dtype=np.float64).reshape(quat_xyzw.shape)


class PicoXRClient:
    """
    Receives the UDP packet format produced by XRStreamer/PoseUdpSender.

    Output poses are in a right-handed robot-friendly frame:
        x = front, y = left, z = up
    Positions are meters. Quaternions are xyzw.
    """

    def __init__(self, udp_host="0.0.0.0", udp_port=5005):
        self.udp_host = udp_host
        self.udp_port = udp_port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(1.0)
        self._sock.bind((udp_host, udp_port))

        self._lock = threading.Lock()
        self._has_frame = threading.Event()
        self._latest_frame = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()

    def shutdown(self):
        self._stop.set()
        try:
            self._sock.close()
        except Exception:
            pass

    def get_frame(self, timeout=None):
        if not self._has_frame.wait(timeout=timeout):
            return None
        with self._lock:
            return copy.deepcopy(self._latest_frame)

    def _recv_loop(self):
        while not self._stop.is_set():
            try:
                data, _ = self._sock.recvfrom(8192)
            except socket.timeout:
                continue
            except OSError:
                break

            msg = data.decode("utf-8", errors="ignore").strip()
            frame = self._parse_packet(msg, time.time())
            if frame is None:
                continue

            with self._lock:
                self._latest_frame = frame
            self._has_frame.set()

    def _parse_packet(self, msg, recv_time):
        parts = msg.split(",")
        frame_id = self._parse_int_block(parts, "FRAME", 1)[0]

        hmd_pose = np.asarray(self._parse_float_block(parts, "HMD", 7), dtype=np.float64)
        left_pose = np.asarray(self._parse_float_block(parts, "LEFTHAND", 7), dtype=np.float64)
        right_pose = np.asarray(self._parse_float_block(parts, "RIGHTHAND", 7), dtype=np.float64)
        body_imu = np.asarray(self._parse_float_block(parts, "BODYIMU", 3), dtype=np.float64)

        link_pos_unity = np.stack([hmd_pose[:3], left_pose[:3], right_pose[:3]], axis=0)
        link_quat_unity = np.stack([hmd_pose[3:], left_pose[3:], right_pose[3:]], axis=0)
        link_pos = pos_unity_to_robot(link_pos_unity)
        link_quat_xyzw = quat_unity_to_robot_xyzw(link_quat_unity)

        left_stick = tuple(self._parse_float_block(parts, "LEFTSTICK", 2))
        left_trigger = self._parse_float_block(parts, "LEFTTRIGGER", 1)[0]
        left_grip = self._parse_float_block(parts, "LEFTGRIP", 1)[0]
        left_x, left_y, left_click = self._parse_int_block(parts, "LEFTKEYS", 3)
        right_stick = tuple(self._parse_float_block(parts, "RIGHTSTICK", 2))
        right_trigger = self._parse_float_block(parts, "RIGHTTRIGGER", 1)[0]
        right_grip = self._parse_float_block(parts, "RIGHTGRIP", 1)[0]
        right_a, right_b, right_click = self._parse_int_block(parts, "RIGHTKEYS", 3)

        button_states = {
            "left_stick": left_stick,
            "left_trigger": left_trigger,
            "left_grip": left_grip,
            "left_x": left_x,
            "left_y": left_y,
            "left_click": left_click,
            "right_stick": right_stick,
            "right_trigger": right_trigger,
            "right_grip": right_grip,
            "right_a": right_a,
            "right_b": right_b,
            "right_click": right_click,
            "hmd_valid": bool(np.linalg.norm(hmd_pose[3:]) > 0.5),
        }
        return XRFrame(frame_id, recv_time, link_pos, link_quat_xyzw, button_states, body_imu)

    @staticmethod
    def _parse_float_block(parts, label, count):
        default = [0.0] * count
        try:
            index = parts.index(label)
        except ValueError:
            return default
        if index + count >= len(parts):
            return default
        try:
            return [float(parts[index + i]) for i in range(1, count + 1)]
        except ValueError:
            return default

    @staticmethod
    def _parse_int_block(parts, label, count):
        default = [0] * count
        try:
            index = parts.index(label)
        except ValueError:
            return default
        if index + count >= len(parts):
            return default
        try:
            return [int(parts[index + i]) for i in range(1, count + 1)]
        except ValueError:
            return default
