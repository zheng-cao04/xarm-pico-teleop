#!/usr/bin/env python3
"""Check configured xArm connections without enabling motion or moving hardware."""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path


def _load_config(path):
    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError("PyYAML is required. Install it with: pip install pyyaml") from exc
    with open(Path(path).expanduser(), "r") as f:
        return yaml.safe_load(f)


def _ping(ip):
    result = subprocess.run(
        ["ping", "-c", "1", "-W", "1", ip],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return result.returncode == 0


def _safe_call(obj, method_name, *args, **kwargs):
    try:
        return getattr(obj, method_name)(*args, **kwargs)
    except Exception as exc:
        return f"{type(exc).__name__}: {exc}"


def _read_mode(arm):
    get_mode = getattr(arm, "get_mode", None)
    if callable(get_mode):
        return get_mode()
    return getattr(arm, "mode", "unknown")


def _short(value, max_items=8):
    if isinstance(value, tuple) and len(value) == 2:
        code, payload = value
        if isinstance(payload, (list, tuple)):
            payload = [round(float(v), 4) if isinstance(v, (float, int)) else v for v in payload[:max_items]]
        return f"code={code}, value={payload}"
    return str(value)


def _check_arm(name, robot_config, connect_wait_s, skip_ping):
    ip = robot_config["robot_ip"]
    print(f"\n{name} xArm: {ip}")
    if not skip_ping:
        print(f"  ping: {'ok' if _ping(ip) else 'failed'}")

    from xarm.wrapper import XArmAPI

    try:
        arm = XArmAPI(ip)
    except Exception as exc:
        print(f"  sdk exception: {type(exc).__name__}: {exc}")
        return False
    time.sleep(connect_wait_s)
    connected = bool(getattr(arm, "connected", False))
    print(f"  sdk connected: {connected}")
    if not connected:
        print("  result: failed to connect with xArm SDK")
        return False

    print(f"  state: {_short(_safe_call(arm, 'get_state'))}")
    print(f"  mode: {_short(_read_mode(arm))}")
    print(f"  err/warn: {_short(_safe_call(arm, 'get_err_warn_code'))}")
    print(f"  tcp pose rpy: {_short(_safe_call(arm, 'get_position', is_radian=True), max_items=6)}")
    print(f"  joints: {_short(_safe_call(arm, 'get_servo_angle', is_radian=True), max_items=7)}")

    disconnect = getattr(arm, "disconnect", None)
    if callable(disconnect):
        disconnect()
    return True


def main():
    parser = argparse.ArgumentParser(description="Check xArm SDK connectivity without moving the robots")
    parser.add_argument("-c", "--config", required=True, help="PICO dual-arm teleop config path")
    parser.add_argument(
        "--arm",
        choices=["left", "right", "both"],
        default="both",
        help="select which configured arm(s) to check",
    )
    parser.add_argument("--connect-wait-s", type=float, default=0.5, help="seconds to wait after SDK construction")
    parser.add_argument("--skip-ping", action="store_true", help="skip ICMP ping and only use xArm SDK connection")
    args = parser.parse_args()

    config = _load_config(args.config)
    ok = True
    names = {"left": ("L",), "right": ("R",), "both": ("L", "R")}[args.arm]
    for name in names:
        ok = _check_arm(
            name,
            config[name]["RobotConfig"],
            connect_wait_s=args.connect_wait_s,
            skip_ping=args.skip_ping,
        ) and ok
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
