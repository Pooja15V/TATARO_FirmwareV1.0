"""Android/WebXR phone teleoperator that reports the phone pose in *robot base axes*.

Why not LeRobot's `Phone`: it reads the `teleop` package's accumulated pose, whose position is in
the WebXR world frame. That frame's "forward" is wherever the phone camera pointed when the AR
session started, i.e. an arbitrary yaw relative to the robot, so +X ends up in a random direction.

Here we read the raw WebXR pose from each message and calibrate the heading once:
hold the phone flat, screen up, TOP EDGE POINTING AT ROBOT +X (away from the base, toward the
work area), and press Move. The top edge's horizontal direction at that moment defines robot +X.

Output (absolute, robot axes): phone.pos (3,) [m], phone.rot (scipy Rotation),
phone.enabled (Move held), phone.gripper (0 open / 1 closed, or None if unchanged).
"""

import logging
import threading
import time
from dataclasses import dataclass
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from lerobot.teleoperators.config import TeleoperatorConfig
from lerobot.teleoperators.teleoperator import Teleoperator

logger = logging.getLogger(__name__)

# WebXR world (x right, y up, z back) -> forward-left-up
_RUB2FLU = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])


@TeleoperatorConfig.register_subclass("lite6_webxr_phone")
@dataclass
class Lite6WebXRPhoneConfig(TeleoperatorConfig):
    host: str = "0.0.0.0"
    port: int = 4443


class Lite6WebXRPhone(Teleoperator):
    config_class = Lite6WebXRPhoneConfig
    name = "lite6_webxr_phone"

    def __init__(self, config: Lite6WebXRPhoneConfig):
        super().__init__(config)
        self.config = config
        self._teleop = None
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._msg: dict | None = None
        self._msg_t = 0.0
        self._yaw_inv: Rotation | None = None
        self._grip: float | None = None
        self._prev_gripper_field = None

    # -------------------------------------------------------------- Teleoperator API
    @property
    def action_features(self) -> dict:
        return {"phone.pos": np.ndarray, "phone.rot": Rotation, "phone.enabled": bool, "phone.gripper": float}

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._teleop is not None

    @property
    def is_calibrated(self) -> bool:
        return self._yaw_inv is not None

    def connect(self, calibrate: bool = True) -> None:
        from teleop import Teleop

        self._teleop = Teleop(host=self.config.host, port=self.config.port)
        self._teleop.subscribe(self._on_message)
        self._thread = threading.Thread(target=self._teleop.run, daemon=True)
        self._thread.start()
        if calibrate:
            self.calibrate()

    def calibrate(self) -> None:
        print(
            "\n  Open the https URL above in Chrome on the phone, accept the certificate, tap Start.\n"
            "  CALIBRATE: hold the phone FLAT, SCREEN UP, TOP EDGE POINTING AT ROBOT +X\n"
            "             (away from the robot base, toward the work area), then press and hold Move.\n"
        )
        while True:
            with self._lock:
                msg = self._msg
            if msg is not None and msg.get("move"):
                _, R = self._raw_pose_flu(msg)
                top = R.apply([0.0, 1.0, 0.0])  # phone top edge in world FLU
                if np.hypot(top[0], top[1]) > 0.3:
                    yaw = np.arctan2(top[1], top[0])
                    self._yaw_inv = Rotation.from_euler("z", -yaw)
                    break
                print("  phone top edge is pointing too far up/down -- hold it flat")
                time.sleep(1.0)
            time.sleep(0.02)
        print("  Calibrated. Release Move. Hold Move to drive; A = open, B = close (or the gripper button).\n")
        while True:  # wait for release so the first drive starts cleanly
            with self._lock:
                if not (self._msg or {}).get("move"):
                    break
            time.sleep(0.02)

    def configure(self) -> None:
        pass

    def get_action(self) -> dict[str, Any]:
        with self._lock:
            msg = self._msg
            age = time.monotonic() - self._msg_t
        if msg is None or not self.is_calibrated:
            return {}
        p, R = self._raw_pose_flu(msg)
        p_r = self._yaw_inv.apply(p)
        R_r = self._yaw_inv * R
        # stale stream (phone locked / tab in background) -> treat as released
        enabled = bool(msg.get("move")) and age < 0.3
        self._update_gripper(msg)
        return {"phone.pos": p_r, "phone.rot": R_r, "phone.enabled": enabled, "phone.gripper": self._grip}

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        pass

    def disconnect(self) -> None:
        if self._teleop is not None:
            try:
                self._teleop.stop()
            except Exception:  # noqa: BLE001
                pass
        self._teleop = None
        self._thread = None

    # -------------------------------------------------------------- internals
    def _on_message(self, _pose, message: dict) -> None:
        with self._lock:
            self._msg = message
            self._msg_t = time.monotonic()

    @staticmethod
    def _raw_pose_flu(msg: dict) -> tuple[np.ndarray, Rotation]:
        pos, q = msg["position"], msg["orientation"]
        p = _RUB2FLU @ np.array([pos["x"], pos["y"], pos["z"]], dtype=float)
        R_rub = Rotation.from_quat([q["x"], q["y"], q["z"], q["w"]])  # device axes in RUB world
        R = Rotation.from_matrix(_RUB2FLU @ R_rub.as_matrix())  # device axes in FLU world
        return p, R

    def _update_gripper(self, msg: dict) -> None:
        if msg.get("reservedButtonA"):
            self._grip = 0.0
        elif msg.get("reservedButtonB"):
            self._grip = 1.0
        g = msg.get("gripper")
        if g is not None and g != self._prev_gripper_field:
            if self._prev_gripper_field is not None:
                self._grip = 1.0 if str(g).lower() in ("close", "closed", "true", "1") else 0.0
            self._prev_gripper_field = g
