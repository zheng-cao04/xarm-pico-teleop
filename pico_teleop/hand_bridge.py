#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import json
import logging
import threading
import time
from dataclasses import dataclass
from typing import Any, Tuple

import numpy as np


logger = logging.getLogger("pico_teleop.hand_bridge")


# Joint limits copied from the WujiHand bring-up scripts in GenesisPlayground.
# The bridge maps closure=0 to the lower/open pose and closure=1 to the
# upper/closed pose, optionally scaled by close_scale for conservative tests.
WUJI_OPEN_JOINT_POSITIONS = (
    -0.0766,
    -0.2504,
    -0.5395,
    -0.5673,
    -0.2532,
    -0.4818,
    -0.5400,
    -0.5470,
    -0.2497,
    -0.4692,
    -0.5821,
    -0.5099,
    -0.2421,
    -0.4727,
    -0.5564,
    -0.5531,
    -0.2235,
    -0.4634,
    -0.5505,
    -0.5525,
)

WUJI_CLOSED_JOINT_POSITIONS = (
    1.6688,
    0.9165,
    1.6773,
    1.6535,
    1.6420,
    0.3211,
    1.6768,
    1.6794,
    1.6586,
    0.3422,
    1.6512,
    1.7132,
    1.6797,
    0.2895,
    1.6550,
    1.6807,
    1.6727,
    0.2939,
    1.6572,
    1.6878,
)


@dataclass
class GripperBridgeConfig:
    driver: str = "none"
    side: str = "both"  # left, right, or both controller/hand channels
    is_dummy: bool = False
    serial_number: str | None = None
    left_serial_number: str | None = None
    right_serial_number: str | None = None
    ema_alpha: float = 1.0
    close_scale: float = 1.0
    command_deadband: float = 0.005
    min_send_interval_s: float = 0.02
    open_joint_positions: Tuple[float, ...] | None = None
    closed_joint_positions: Tuple[float, ...] | None = None
    publish_redis: bool = False
    redis_url: str = "redis://localhost:6379/0"
    redis_key: str = "motion:ref:latest"


def _clamp01(value):
    return min(max(float(value), 0.0), 1.0)


def _joint_positions(value, fallback):
    if value is None:
        return np.asarray(fallback, dtype=float)
    array = np.asarray(list(value), dtype=float).reshape(-1)
    if array.size != 20:
        raise ValueError("WujiHand joint position profiles must contain 20 values")
    return array


def _selected_sides(side):
    normalized = side.strip().lower()
    if normalized == "both":
        return ("left", "right")
    if normalized in ("left", "right"):
        return (normalized,)
    raise ValueError("GripperBridge.side must be one of: left, right, both")


class _WujiHandDevice:
    def __init__(
        self,
        side: str,
        serial_number: str | None,
        is_dummy: bool,
        open_positions: np.ndarray,
        closed_positions: np.ndarray,
        close_scale: float,
        ema_alpha: float,
        command_deadband: float,
        min_send_interval_s: float,
    ):
        self.side = side
        self.serial_number = serial_number
        self.is_dummy = is_dummy
        self.open_positions = open_positions
        self.closed_positions = closed_positions
        self.close_scale = _clamp01(close_scale)
        self.ema_alpha = _clamp01(ema_alpha)
        self.command_deadband = max(0.0, float(command_deadband))
        self.min_send_interval_s = max(0.0, float(min_send_interval_s))
        self.closure = 0.0
        self._hand: Any = None
        self._rt_ctx: Any = None
        self._ctrl: Any = None
        self._lock = threading.Lock()
        self._last_command_time = 0.0
        self._target_positions: np.ndarray | None = None
        self._joint_positions = np.array(self.open_positions, dtype=float)

    def start(self):
        if self.is_dummy:
            logger.info("WujiHand %s bridge running in dummy mode", self.side)
            return

        try:
            import wujihandpy
        except ImportError as exc:
            raise RuntimeError(
                "GripperBridge.driver=wuji_hand requires wujihandpy in the active Python environment"
            ) from exc

        kwargs: dict[str, Any] = {}
        if self.serial_number is not None:
            kwargs["serial_number"] = self.serial_number
        self._hand = wujihandpy.Hand(**kwargs)
        self._hand.disable_thread_safe_check()
        self._hand.write_joint_enabled(True)
        self._rt_ctx = self._hand.realtime_controller(
            enable_upstream=True,
            filter=wujihandpy.filter.LowPass(cutoff_freq=5.0),
        )
        self._ctrl = self._rt_ctx.__enter__()
        self._joint_positions = self.read_joint_positions()
        logger.info("WujiHand %s bridge started", self.side)

    def read_joint_positions(self):
        if self.is_dummy:
            return self._joint_positions.copy()
        if self._ctrl is None:
            return self._joint_positions.copy()
        with self._lock:
            positions = self._ctrl.get_joint_actual_position()
        self._joint_positions = np.asarray(positions, dtype=float).reshape(-1)
        return self._joint_positions.copy()

    def set_closure(self, closure):
        requested = _clamp01(closure)
        effective = _clamp01(requested * self.close_scale)
        target = self.open_positions + effective * (self.closed_positions - self.open_positions)
        if self._target_positions is not None and self.ema_alpha < 1.0:
            target = self.ema_alpha * target + (1.0 - self.ema_alpha) * self._target_positions

        now = time.monotonic()
        if self._target_positions is not None:
            max_delta = float(np.max(np.abs(target - self._target_positions)))
            if max_delta < self.command_deadband and now - self._last_command_time < self.min_send_interval_s:
                return self.closure

        self._target_positions = target
        self._last_command_time = now
        self.closure = effective
        self._joint_positions = target.copy()
        if self.is_dummy:
            return self.closure

        if self._ctrl is None:
            raise RuntimeError(f"WujiHand {self.side} bridge is not started")
        with self._lock:
            self._ctrl.set_joint_target_position(target.reshape(5, 4))
        return self.closure

    def stop(self):
        if self._hand is None or self.is_dummy:
            self._hand = None
            return
        try:
            with self._lock:
                if self._rt_ctx is not None:
                    self._rt_ctx.__exit__(None, None, None)
                    self._rt_ctx = None
                    self._ctrl = None
        except Exception:
            logger.warning("WujiHand %s realtime controller exit failed", self.side, exc_info=True)
        try:
            self._hand.write_joint_enabled(False)
        except Exception:
            logger.warning("WujiHand %s disable failed", self.side, exc_info=True)
        self._hand = None
        logger.info("WujiHand %s bridge stopped", self.side)


class _WujiHandAdapter:
    def __init__(self, config: GripperBridgeConfig):
        self.config = config
        self._hands: dict[str, _WujiHandDevice] = {}
        open_positions = _joint_positions(config.open_joint_positions, WUJI_OPEN_JOINT_POSITIONS)
        closed_positions = _joint_positions(config.closed_joint_positions, WUJI_CLOSED_JOINT_POSITIONS)

        sides = _selected_sides(config.side)
        for side in sides:
            if len(sides) == 1:
                serial_number = config.serial_number or getattr(config, f"{side}_serial_number")
            else:
                serial_number = getattr(config, f"{side}_serial_number")
            self._hands[side] = _WujiHandDevice(
                side=side,
                serial_number=serial_number,
                is_dummy=config.is_dummy,
                open_positions=open_positions,
                closed_positions=closed_positions,
                close_scale=config.close_scale,
                ema_alpha=config.ema_alpha,
                command_deadband=config.command_deadband,
                min_send_interval_s=config.min_send_interval_s,
            )

    def start(self):
        for hand in self._hands.values():
            hand.start()

    def command(self, left_action, right_action):
        left_closure = _clamp01(left_action)
        right_closure = _clamp01(right_action)
        if "left" in self._hands:
            left_closure = self._hands["left"].set_closure(left_action)
        if "right" in self._hands:
            right_closure = self._hands["right"].set_closure(right_action)
        return left_closure, right_closure

    def joint_positions(self):
        return {side: hand.read_joint_positions().tolist() for side, hand in self._hands.items()}

    def close(self):
        for hand in self._hands.values():
            hand.stop()


class _ChangingTekAdapter:
    def __init__(self):
        self._driver = None

    def start(self):
        try:
            from gs_env.real.changingtek.gripper import Gripper
        except ImportError as exc:
            raise RuntimeError(
                "GripperBridge.driver=changingtek requires gs_env.real.changingtek.gripper"
            ) from exc
        self._driver = Gripper()
        logger.info("ChangingTek gripper bridge started")

    def command(self, left_action, right_action):
        self._driver.set_closure(_clamp01(left_action), _clamp01(right_action))
        self._driver.update_state()
        left_closure = float(getattr(self._driver, "closure1", left_action))
        right_closure = float(getattr(self._driver, "closure2", right_action))
        return _clamp01(left_closure), _clamp01(right_closure)

    def joint_positions(self):
        return {}

    def close(self):
        for method_name in ("close", "stop", "shutdown"):
            method = getattr(self._driver, method_name, None)
            if callable(method):
                method()
                break


class GripperBridge:
    """Normalized hand/gripper side channel independent from the robot body."""

    def __init__(self, config: GripperBridgeConfig):
        self.config = config
        self._redis = None
        self._adapter = None
        self._started = False

    def start(self):
        if self._started:
            return

        driver = self.config.driver.strip().lower()
        if driver in ("none", ""):
            self._adapter = None
        elif driver in ("wuji", "wuji_hand"):
            self._adapter = _WujiHandAdapter(self.config)
            self._adapter.start()
        elif driver == "changingtek":
            self._adapter = _ChangingTekAdapter()
            self._adapter.start()
        else:
            raise ValueError("GripperBridge.driver must be one of: none, wuji_hand, changingtek")

        if self.config.publish_redis:
            try:
                import redis
            except ImportError as exc:
                raise RuntimeError(
                    "GripperBridge.publish_redis requires redis. Install it with: pip install redis"
                ) from exc
            self._redis = redis.from_url(self.config.redis_url)
            logger.info("Publishing hand state to Redis key prefix %s", self.config.redis_key)

        self._started = True

    def update(self, left_action, right_action, left_closure=None, right_closure=None):
        left_action = _clamp01(left_action)
        right_action = _clamp01(right_action)
        if left_closure is None:
            left_closure = left_action
        if right_closure is None:
            right_closure = right_action

        if not self._started:
            self.start()

        if self._adapter is not None:
            left_closure, right_closure = self._adapter.command(left_action, right_action)

        if self._redis is not None:
            key = self.config.redis_key
            self._redis.set(f"{key}:gripper:action", json.dumps([left_action, right_action]))
            self._redis.set(f"{key}:gripper:closure", json.dumps([left_closure, right_closure]))
            self._redis.set(f"{key}:hand:closure", json.dumps([left_closure, right_closure]))
            joint_positions = self.joint_positions()
            if joint_positions:
                self._redis.set(f"{key}:hand:joint_positions", json.dumps(joint_positions))

        return left_closure, right_closure

    def joint_positions(self):
        if self._adapter is None:
            return {}
        return self._adapter.joint_positions()

    def close(self):
        if self._adapter is not None:
            self._adapter.close()
        self._adapter = None
        self._started = False
