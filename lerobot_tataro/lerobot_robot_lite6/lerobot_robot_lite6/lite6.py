"""LeRobot Robot plugin for the TATARO UFactory Lite6 + Gripper Lite.

The arm is driven in Cartesian space: send_action() takes an absolute TCP pose target and a
100 Hz background thread walks the servo setpoint toward it with linear/angular speed limits
(set_servo_cartesian, mode 1). The controller does the IK, so no URDF is needed.
"""

import logging
import threading
import time
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from lerobot.cameras import make_cameras_from_configs
from lerobot.robots.robot import Robot
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from .config_lite6 import Lite6Config
from .pose_utils import (
    EE_KEYS,
    dict_to_pR,
    pR_to_dict,
    pR_to_xarm_pose,
    tilt_from_down,
    xarm_pose_to_pR,
)

logger = logging.getLogger(__name__)

JOINT_KEYS = tuple(f"joint{i}.pos" for i in range(1, 7))


class Lite6(Robot):
    config_class = Lite6Config
    name = "lite6"

    def __init__(self, config: Lite6Config):
        super().__init__(config)
        self.config = config
        self.cameras = make_cameras_from_configs(config.cameras)
        self._arm = None
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._paused = threading.Event()
        self._fault: str | None = None
        # servo setpoint actually streamed, and latest accepted target
        self._cmd_p = self._cmd_R = None
        self._tgt_p = self._tgt_R = None
        self._vel, self._omega = np.zeros(3), np.zeros(3)
        self._gripper_target = 0.0  # 0 open, 1 closed
        self._gripper_applied: float | None = None
        self._gripper_open_t = 0.0
        self._gripper_stopped = True
        self._last_reject_log = 0.0

    # ------------------------------------------------------------------ features
    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {k: (c.height, c.width, 3) for k, c in self.config.cameras.items()}

    @property
    def observation_features(self) -> dict:
        ft = {k: float for k in JOINT_KEYS}
        ft.update({k: float for k in EE_KEYS})
        ft["gripper.pos"] = float
        ft.update(self._cameras_ft)
        return ft

    @property
    def action_features(self) -> dict:
        ft = {k: float for k in EE_KEYS}
        ft["gripper.pos"] = float
        return ft

    # ------------------------------------------------------------------ connection
    @property
    def is_connected(self) -> bool:
        return self._arm is not None and all(c.is_connected for c in self.cameras.values())

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass  # absolute encoders, nothing to calibrate

    def connect(self, calibrate: bool = True) -> None:
        if self._arm is not None:
            raise DeviceAlreadyConnectedError(f"{self} already connected")
        from xarm.wrapper import XArmAPI

        # 'devlop' = real-time report port (~100 Hz) -> arm.angles / arm.position are fresh.
        arm = XArmAPI(self.config.ip, is_radian=True, report_type="devlop")
        if not arm.connected:
            raise DeviceNotConnectedError(f"Could not connect to Lite6 at {self.config.ip}")
        self._arm = arm
        try:
            for cam in self.cameras.values():
                cam.connect()
            self.configure()
        except Exception:
            self.disconnect()
            raise
        logger.info(f"{self} connected.")

    def configure(self) -> None:
        arm = self._arm
        arm.clean_warn()
        arm.clean_error()
        arm.set_collision_sensitivity(self.config.collision_sensitivity)
        arm.motion_enable(enable=True)
        time.sleep(0.5)
        if self.config.go_home_on_connect:
            self._move_home_blocking()
        if not self.config.servo_on_connect:
            return  # passive: position mode, no streaming
        self._enter_servo_mode()
        self._open_gripper_now()
        self._stop.clear()
        self._thread = threading.Thread(target=self._servo_loop, name="lite6-servo", daemon=True)
        self._thread.start()

    def disconnect(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._arm is not None:
            try:
                self._arm.stop_lite6_gripper()
                self._arm.set_mode(0)
                self._arm.set_state(0)
            except Exception as e:  # noqa: BLE001
                logger.warning(f"Lite6 cleanup: {e}")
            self._arm.disconnect()
            self._arm = None
        for cam in self.cameras.values():
            if cam.is_connected:
                cam.disconnect()
        logger.info(f"{self} disconnected.")

    # ------------------------------------------------------------------ motion helpers
    def _check_ok(self, what: str) -> None:
        arm = self._arm
        if arm.error_code != 0:
            raise RuntimeError(f"Lite6 controller error {arm.error_code} during {what} (clear it in UFactory Studio or power-cycle).")

    def _read_pose(self) -> tuple[np.ndarray, Rotation]:
        code, pose = self._arm.get_position(is_radian=True)
        if code != 0:
            raise RuntimeError(f"get_position failed: {code}")
        return xarm_pose_to_pR(pose)

    def _move_home_blocking(self) -> None:
        arm = self._arm
        arm.set_mode(0)
        arm.set_state(0)
        time.sleep(0.2)
        home = list(self.config.home_joints_deg)
        waypoints = [home]
        code, q = arm.get_servo_angle(is_radian=False)
        if code == 0 and max(abs(v) for v in q[:6]) < 5.0:
            # Folded zero pose: moving J2 first can trip C22 self-collision. Lift J3/J5 first.
            waypoints.insert(0, [0.0, 0.0, home[2], 0.0, home[4], 0.0])
        for wp in waypoints:
            code = arm.set_servo_angle(
                angle=wp, speed=self.config.home_speed_deg_s, is_radian=False, wait=True
            )
            if code != 0:
                raise RuntimeError(f"Homing failed: set_servo_angle returned {code}, error_code={arm.error_code}")
            self._check_ok("homing")

    def _enter_servo_mode(self) -> None:
        arm = self._arm
        arm.set_mode(1)
        arm.set_state(0)
        time.sleep(0.3)
        self._check_ok("entering servo mode")
        p, R = self._read_pose()
        with self._lock:
            self._cmd_p, self._cmd_R = p.copy(), R
            self._tgt_p, self._tgt_R = p.copy(), R
            self._vel, self._omega = np.zeros(3), np.zeros(3)
            self._fault = None

    def go_home(self) -> None:
        """Pause streaming, move to the joint home pose, then resume servoing from there."""
        if self._arm is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        self._paused.set()
        time.sleep(2.0 / self.config.servo_rate_hz)
        try:
            self._move_home_blocking()
            self._enter_servo_mode()
        finally:
            self._paused.clear()

    def _open_gripper_now(self) -> None:
        self._gripper_target = 0.0
        self._gripper_applied = None

    def _apply_gripper(self) -> None:
        arm = self._arm
        tgt = self._gripper_target
        if self._gripper_applied != tgt:
            if tgt >= 0.5:
                arm.close_lite6_gripper()
            else:
                arm.open_lite6_gripper()
                self._gripper_open_t = time.monotonic()
                self._gripper_stopped = False
            self._gripper_applied = tgt
        # Opening has no end-stop feedback: stop driving after a moment (as the SDK examples do).
        if tgt < 0.5 and not self._gripper_stopped and time.monotonic() - self._gripper_open_t > self.config.gripper_open_stop_s:
            arm.stop_lite6_gripper()
            self._gripper_stopped = True

    def read_gripper_state(self) -> float:
        """Gripper Lite state from the tool-IO outputs: 1 closed, 0 open (stopped keeps the last state)."""
        code, out = self._arm.get_tgpio_output_digital()
        if code == 0:
            if out[1] == 1:
                self._gripper_target = 1.0
            elif out[0] == 1:
                self._gripper_target = 0.0
        return float(self._gripper_target)

    @staticmethod
    def _track(err: np.ndarray, vel: np.ndarray, tau: float, vmax: float, amax: float, dt: float) -> np.ndarray:
        """One step of a smooth, accel-limited tracker. Returns the new velocity vector."""
        e = float(np.linalg.norm(err))
        if e < 1e-9:
            v_des = np.zeros(3)
        else:
            # proportional approach, capped by max speed and by the braking distance
            speed = min(e / tau, vmax, np.sqrt(2.0 * amax * e))
            v_des = err * (speed / e)
        dv = v_des - vel
        n = float(np.linalg.norm(dv))
        if n > amax * dt:
            dv *= amax * dt / n
        return vel + dv

    def _servo_loop(self) -> None:
        cfg = self.config
        dt = 1.0 / cfg.servo_rate_hz
        tau = cfg.smoothing_time_s
        vmax, amax = cfg.max_lin_speed_m_s, cfg.max_lin_acc_m_s2
        wmax, alpha = np.deg2rad(cfg.max_ang_speed_deg_s), np.deg2rad(cfg.max_ang_acc_deg_s2)
        next_t = time.perf_counter()
        last_tick = next_t
        periods: list[float] = []
        last_report = next_t
        while not self._stop.is_set():
            next_t += dt
            now = time.perf_counter()
            periods.append(now - last_tick)
            last_tick = now
            if now - last_report > 10.0 and periods:
                pa = np.array(periods[1:]) * 1000
                late = int((pa > 2.5 * dt * 1000).sum())
                if late > 0.01 * len(pa):
                    logger.warning(
                        f"[Lite6 servo timing] mean {pa.mean():.1f} ms, max {pa.max():.1f} ms, "
                        f"{late}/{len(pa)} ticks late (target {dt * 1000:.0f} ms) -- CPU/GIL contention"
                    )
                periods, last_report = [], now
            if not self._paused.is_set() and self._fault is None:
                try:
                    with self._lock:
                        self._vel = self._track(self._tgt_p - self._cmd_p, self._vel, tau, vmax, amax, dt)
                        self._cmd_p = self._cmd_p + self._vel * dt
                        r_err = (self._tgt_R * self._cmd_R.inv()).as_rotvec()
                        self._omega = self._track(r_err, self._omega, tau, wmax, alpha, dt)
                        self._cmd_R = Rotation.from_rotvec(self._omega * dt) * self._cmd_R
                        cmd = pR_to_xarm_pose(self._cmd_p, self._cmd_R)
                    code = self._arm.set_servo_cartesian(cmd, is_radian=True)
                    self._apply_gripper()
                    if code != 0 or self._arm.error_code != 0:
                        self._fault = f"set_servo_cartesian code={code} error_code={self._arm.error_code}"
                        logger.error(f"Lite6 fault, servo streaming stopped: {self._fault}")
                except Exception as e:  # noqa: BLE001
                    self._fault = f"servo loop exception: {e}"
                    logger.exception("Lite6 servo loop")
            sleep = next_t - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.perf_counter()

    # ------------------------------------------------------------------ IO
    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        arm = self._arm
        # Explicit requests (~1 ms each): the push report only refreshes a few times per second.
        code_q, q = arm.get_servo_angle(is_radian=True)
        code_p, pose = arm.get_position(is_radian=True)
        if code_q != 0 or code_p != 0:
            raise RuntimeError(f"Lite6 state read failed (angles {code_q}, pose {code_p}, error {arm.error_code})")
        q = list(q[:6])
        p, R = xarm_pose_to_pR(pose)
        obs: dict[str, Any] = {k: float(v) for k, v in zip(JOINT_KEYS, q)}
        obs.update(pR_to_dict(p, R))
        obs["gripper.pos"] = float(self._gripper_target) if self.config.servo_on_connect else self.read_gripper_state()
        for key, cam in self.cameras.items():
            obs[key] = cam.async_read()
        return obs

    def _log_reject(self, msg: str) -> None:
        now = time.monotonic()
        if now - self._last_reject_log > 1.0:
            logger.warning(f"[Lite6 SAFETY] {msg} -- holding previous target")
            self._last_reject_log = now

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.config.servo_on_connect:
            raise RuntimeError("Lite6 was connected with servo_on_connect=False; send_action is disabled.")
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        if self._fault is not None:
            raise RuntimeError(f"Lite6 is faulted ({self._fault}). Clear the error and reconnect.")
        cfg = self.config

        if "gripper.pos" in action:
            self._gripper_target = 1.0 if float(action["gripper.pos"]) >= 0.5 else 0.0

        if all(k in action for k in EE_KEYS):
            p, R = dict_to_pR(action)
            p = np.clip(p, cfg.workspace_min_m, cfg.workspace_max_m)
            r = float(np.hypot(p[0], p[1]))
            if cfg.min_base_radius_m > 0 and r < cfg.min_base_radius_m:
                if time.monotonic() - self._last_reject_log > 1.0:
                    logger.warning(f"[Lite6 SAFETY] target {r * 1000:.0f} mm from the base axis, "
                                   f"pushed out to {cfg.min_base_radius_m * 1000:.0f} mm")
                    self._last_reject_log = time.monotonic()
                d = p[:2] / r if r > 1e-6 else np.array([1.0, 0.0])
                p = p.copy()
                p[:2] = d * cfg.min_base_radius_m
            with self._lock:
                prev_p, prev_R = self._tgt_p, self._tgt_R
            jump = np.linalg.norm(p - prev_p)
            rot_jump = np.rad2deg((R * prev_R.inv()).magnitude())
            tilt = np.rad2deg(tilt_from_down(R))
            if not np.all(np.isfinite(p)) or jump > cfg.max_target_jump_m or rot_jump > cfg.max_target_jump_deg:
                self._log_reject(f"target jump {jump * 1000:.0f} mm / {rot_jump:.0f} deg")
                p, R = prev_p, prev_R
            elif tilt > cfg.max_tilt_deg:
                self._log_reject(f"tool tilt {tilt:.0f} deg > {cfg.max_tilt_deg:.0f}")
                R = prev_R
            with self._lock:
                self._tgt_p, self._tgt_R = p.copy(), R

        with self._lock:
            sent = pR_to_dict(self._tgt_p, self._tgt_R)
        sent["gripper.pos"] = float(self._gripper_target)
        return sent
