from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.cameras.realsense import RealSenseCameraConfig
from lerobot.robots.config import RobotConfig

# TATARO camera serials (see memory/tataro_cv_calibration: never trust device order, always serials)
WRIST_D435I_SERIAL = "141722076951"
OVERHEAD_D455_SERIAL = "318122301267"


def _default_cameras() -> dict[str, CameraConfig]:
    return {
        "wrist": RealSenseCameraConfig(
            serial_number_or_name=WRIST_D435I_SERIAL, fps=30, width=640, height=480
        ),
        "overhead": RealSenseCameraConfig(
            serial_number_or_name=OVERHEAD_D455_SERIAL, fps=30, width=640, height=480
        ),
    }


@RobotConfig.register_subclass("lite6")
@dataclass
class Lite6Config(RobotConfig):
    """UFactory Lite6 + Gripper Lite, commanded in Cartesian (end-effector) space.

    Action  = absolute TCP pose target: ee.x/y/z [m] + ee.wx/wy/wz [rotvec, rad, relative to
              "gripper pointing straight down"] + gripper.pos (0 = open, 1 = closed).
    The Lite6 controller does the IK; this class only streams set_servo_cartesian targets.
    """

    ip: str = "192.168.1.150"

    # Joint-space home ("ready" pose from the 2026-10-27 pick cycle): TCP ~ (200, 0, 250) mm, tool down.
    home_joints_deg: list[float] = field(default_factory=lambda: [-0.3, -11.8, 47.0, 0.0, 58.8, -0.2])
    go_home_on_connect: bool = True
    home_speed_deg_s: float = 20.0
    # False = stay in position mode (0) with no servo thread, e.g. while the controller plays
    # back a recorded .traj; send_action() is then unavailable.
    servo_on_connect: bool = True

    # Internal servo thread: interpolates toward the latest target at this rate with speed limits.
    servo_rate_hz: float = 100.0
    max_lin_speed_m_s: float = 0.15
    max_ang_speed_deg_s: float = 90.0
    # Smooth tracking filter: velocity = error / smoothing_time, limited in speed AND acceleration,
    # so the setpoint never stops-and-starts between 30 Hz targets and phone jitter is filtered.
    smoothing_time_s: float = 0.10
    max_lin_acc_m_s2: float = 1.0
    max_ang_acc_deg_s2: float = 600.0

    # Safety: targets further than this from the previous accepted target are rejected (held).
    max_target_jump_m: float = 0.15
    max_target_jump_deg: float = 60.0
    # Max tilt of the tool axis away from straight-down.
    max_tilt_deg: float = 60.0
    # Workspace box for the TCP (base frame, metres). z_min is fingertips above the base plane.
    # Widened 2026-11-10 to cover the hand-guided dataset (x -0.111..0.442, y -0.358..0.355, grasp z >= 0.007).
    workspace_min_m: list[float] = field(default_factory=lambda: [-0.12, -0.37, 0.005])
    workspace_max_m: list[float] = field(default_factory=lambda: [0.45, 0.37, 0.40])
    # TCP targets closer than this to the base axis (xy) are pushed radially outward: self-collision guard
    # (the arm folded into itself with C22/C31 at 87 mm on 2026-11-06). 0 disables.
    min_base_radius_m: float = 0.15

    collision_sensitivity: int = 3
    # Seconds to drive the Gripper Lite open before stopping it (it has no position feedback).
    gripper_open_stop_s: float = 1.0

    cameras: dict[str, CameraConfig] = field(default_factory=_default_cameras)
