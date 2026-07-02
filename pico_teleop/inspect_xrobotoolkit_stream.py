#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import time

import numpy as np

from xrobotoolkit_xr_client import XRoboToolkitXRClient


def _pose_array(raw):
    if raw is None:
        return np.zeros(7, dtype=np.float64)
    pose = np.asarray(raw, dtype=np.float64).reshape(-1)
    if pose.size < 7:
        return np.zeros(7, dtype=np.float64)
    return pose[:7]


def _valid_pose(pose):
    pose = np.asarray(pose, dtype=np.float64)
    return pose.shape == (7,) and np.all(np.isfinite(pose)) and np.linalg.norm(pose[3:7]) > 0.5


def _fmt(values, width=7, precision=3):
    return "[" + ", ".join(f"{float(v):{width}.{precision}f}" for v in values) + "]"


def _read_controller(xrt, name):
    if name == "left":
        raw_pose = _pose_array(xrt.get_left_controller_pose())
        grip = float(xrt.get_left_grip())
        trigger = float(xrt.get_left_trigger())
        primary = int(bool(xrt.get_X_button()))
        secondary = int(bool(xrt.get_Y_button()))
    else:
        raw_pose = _pose_array(xrt.get_right_controller_pose())
        grip = float(xrt.get_right_grip())
        trigger = float(xrt.get_right_trigger())
        primary = int(bool(xrt.get_A_button()))
        secondary = int(bool(xrt.get_B_button()))

    if _valid_pose(raw_pose):
        robot_pos_m, robot_quat_xyzw = XRoboToolkitXRClient._convert_pose(raw_pose)
    else:
        robot_pos_m = np.full(3, np.nan, dtype=np.float64)
        robot_quat_xyzw = np.array([np.nan, np.nan, np.nan, np.nan], dtype=np.float64)

    return {
        "raw_pose": raw_pose,
        "robot_pos_m": robot_pos_m,
        "robot_quat_xyzw": robot_quat_xyzw,
        "grip": min(max(grip, 0.0), 1.0),
        "trigger": min(max(trigger, 0.0), 1.0),
        "primary": primary,
        "secondary": secondary,
    }


def main():
    parser = argparse.ArgumentParser(description="Inspect XRoboToolkit PICO controller stream")
    parser.add_argument("--hz", type=float, default=5.0, help="print frequency")
    args = parser.parse_args()

    try:
        import xrobotoolkit_sdk as xrt
    except ImportError as exc:
        raise RuntimeError(
            "xrobotoolkit_sdk is not installed. Install XRoboToolkit-PC-Service-Pybind first."
        ) from exc

    period = 1.0 / max(args.hz, 0.1)
    left_ref = None
    right_ref = None

    print("Initializing XRoboToolkit SDK...")
    xrt.init()
    print("Streaming. Move one controller at a time; press Ctrl-C to stop.")
    print("robot_pos_m is converted to x=front, y=left, z=up. delta_mm is relative to script start.")

    try:
        while True:
            timestamp_ns = int(xrt.get_time_stamp_ns())
            left = _read_controller(xrt, "left")
            right = _read_controller(xrt, "right")

            if left_ref is None and np.all(np.isfinite(left["robot_pos_m"])):
                left_ref = left["robot_pos_m"].copy()
            if right_ref is None and np.all(np.isfinite(right["robot_pos_m"])):
                right_ref = right["robot_pos_m"].copy()

            left_delta = (left["robot_pos_m"] - left_ref) * 1000.0 if left_ref is not None else np.full(3, np.nan)
            right_delta = (
                (right["robot_pos_m"] - right_ref) * 1000.0 if right_ref is not None else np.full(3, np.nan)
            )

            print(
                "ts={ts} | "
                "L raw_xyz_m={lraw} robot_xyz_m={lrobot} delta_mm={ldelta} "
                "grip={lgrip:.2f} trig={ltrig:.2f} X={lx} Y={ly} | "
                "R raw_xyz_m={rraw} robot_xyz_m={rrobot} delta_mm={rdelta} "
                "grip={rgrip:.2f} trig={rtrig:.2f} A={ra} B={rb}".format(
                    ts=timestamp_ns,
                    lraw=_fmt(left["raw_pose"][:3]),
                    lrobot=_fmt(left["robot_pos_m"]),
                    ldelta=_fmt(left_delta, width=7, precision=1),
                    lgrip=left["grip"],
                    ltrig=left["trigger"],
                    lx=left["primary"],
                    ly=left["secondary"],
                    rraw=_fmt(right["raw_pose"][:3]),
                    rrobot=_fmt(right["robot_pos_m"]),
                    rdelta=_fmt(right_delta, width=7, precision=1),
                    rgrip=right["grip"],
                    rtrig=right["trigger"],
                    ra=right["primary"],
                    rb=right["secondary"],
                ),
                flush=True,
            )
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        xrt.close()


if __name__ == "__main__":
    main()
