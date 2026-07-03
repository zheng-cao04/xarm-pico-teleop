#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import os
import sys
import time
from dataclasses import asdict
from pathlib import Path

sys.path.insert(0, os.path.dirname(__file__))
from hand_bridge import GripperBridge, GripperBridgeConfig


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


def _config_from_args(args):
    raw = {}
    if args.config is not None:
        raw = _load_config(args.config).get("GripperBridge", {})
    config = GripperBridgeConfig(**raw)

    for field in (
        "driver",
        "side",
        "serial_number",
        "left_serial_number",
        "right_serial_number",
        "ema_alpha",
        "close_scale",
        "command_deadband",
        "min_send_interval_s",
        "redis_url",
        "redis_key",
    ):
        value = getattr(args, field)
        if value is not None:
            setattr(config, field, value)
    if args.dummy is not None:
        config.is_dummy = args.dummy
    if args.publish_redis is not None:
        config.publish_redis = args.publish_redis
    if args.config is None and args.side is None:
        config.side = "right"
    return config


def _print_joint_positions(joint_positions):
    if not joint_positions:
        print("No joint-position state is exposed by this driver.")
        return
    for side, positions in joint_positions.items():
        preview = ", ".join(f"{value:.3f}" for value in positions[:4])
        print(f"{side}: {len(positions)} joints, first four = [{preview}]")


def main():
    parser = argparse.ArgumentParser(description="No-motion hand/gripper connection checker")
    parser.add_argument("--config", type=str, default=None, help="teleop config path with a GripperBridge section")
    parser.add_argument("--driver", choices=["none", "wuji_hand", "changingtek"], default=None)
    parser.add_argument("--side", choices=["left", "right", "both"], default=None)
    parser.add_argument("--serial-number", type=str, default=None)
    parser.add_argument("--left-serial-number", type=str, default=None)
    parser.add_argument("--right-serial-number", type=str, default=None)
    parser.add_argument("--dummy", type=_parse_bool, default=None, metavar="{true,false}")
    parser.add_argument("--ema-alpha", type=float, default=None)
    parser.add_argument("--close-scale", type=float, default=None)
    parser.add_argument("--command-deadband", type=float, default=None)
    parser.add_argument("--min-send-interval-s", type=float, default=None)
    parser.add_argument("--publish-redis", type=_parse_bool, default=None, metavar="{true,false}")
    parser.add_argument("--redis-url", type=str, default=None)
    parser.add_argument("--redis-key", type=str, default=None)
    parser.add_argument(
        "--command-closure",
        type=float,
        default=None,
        help="optional motion test: command normalized closure 0..1 for --duration seconds",
    )
    parser.add_argument("--duration", type=float, default=2.0, help="duration for --command-closure")
    args = parser.parse_args()

    config = _config_from_args(args)
    if config.driver == "none":
        parser.error("set --driver wuji_hand/changingtek or provide a config with GripperBridge.driver")
    if not 0.0 <= config.ema_alpha <= 1.0:
        parser.error("--ema-alpha must be in [0, 1]")
    if not 0.0 <= config.close_scale <= 1.0:
        parser.error("--close-scale must be in [0, 1]")

    bridge = GripperBridge(config)
    try:
        printable = asdict(config)
        printable["open_joint_positions"] = "<default>" if config.open_joint_positions is None else "<custom>"
        printable["closed_joint_positions"] = "<default>" if config.closed_joint_positions is None else "<custom>"
        print("Hand bridge config:")
        for key, value in printable.items():
            print(f"  {key}: {value}")

        bridge.start()
        print("Driver started. Reading state...")
        _print_joint_positions(bridge.joint_positions())

        if args.command_closure is not None:
            closure = min(max(float(args.command_closure), 0.0), 1.0)
            print(f"Commanding closure={closure:.3f} for {args.duration:.1f}s")
            deadline = time.monotonic() + max(0.0, args.duration)
            while time.monotonic() < deadline:
                if config.side == "left":
                    bridge.update(closure, 0.0)
                elif config.side == "right":
                    bridge.update(0.0, closure)
                else:
                    bridge.update(closure, closure)
                time.sleep(0.02)
            _print_joint_positions(bridge.joint_positions())
        else:
            print("No closure command sent. Use --command-closure for an explicit motion test.")
    finally:
        bridge.close()


if __name__ == "__main__":
    main()
