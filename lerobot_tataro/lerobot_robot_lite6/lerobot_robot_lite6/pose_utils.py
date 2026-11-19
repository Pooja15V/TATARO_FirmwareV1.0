"""Pose helpers shared by the robot and the phone pipeline.

Orientation is expressed as a rotation vector *relative to the tool-down orientation*
R_DOWN = Rx(180 deg). Plain rotvec/RPY of a tool-down gripper sits exactly on the +-pi
wrap-around, which makes training targets jump; the relative form is ~0 near tool-down
and continuous for any reachable orientation.
"""

import numpy as np
from scipy.spatial.transform import Rotation

R_DOWN = Rotation.from_euler("x", np.pi)

EE_KEYS = ("ee.x", "ee.y", "ee.z", "ee.wx", "ee.wy", "ee.wz")


def rel_rotvec_from_R(R: Rotation) -> np.ndarray:
    return (R_DOWN.inv() * R).as_rotvec()


def R_from_rel_rotvec(w) -> Rotation:
    return R_DOWN * Rotation.from_rotvec(np.asarray(w, dtype=float))


def xarm_pose_to_pR(pose_mm_rad) -> tuple[np.ndarray, Rotation]:
    """xArm [x,y,z (mm), roll,pitch,yaw (rad, extrinsic xyz)] -> (p [m], R)."""
    p = np.asarray(pose_mm_rad[:3], dtype=float) / 1000.0
    R = Rotation.from_euler("xyz", pose_mm_rad[3:6])
    return p, R


def pR_to_xarm_pose(p: np.ndarray, R: Rotation) -> list[float]:
    rpy = R.as_euler("xyz")
    return [*(np.asarray(p) * 1000.0).tolist(), *rpy.tolist()]


def dict_to_pR(d: dict) -> tuple[np.ndarray, Rotation]:
    p = np.array([d["ee.x"], d["ee.y"], d["ee.z"]], dtype=float)
    return p, R_from_rel_rotvec([d["ee.wx"], d["ee.wy"], d["ee.wz"]])


def pR_to_dict(p: np.ndarray, R: Rotation) -> dict[str, float]:
    w = rel_rotvec_from_R(R)
    return {
        "ee.x": float(p[0]),
        "ee.y": float(p[1]),
        "ee.z": float(p[2]),
        "ee.wx": float(w[0]),
        "ee.wy": float(w[1]),
        "ee.wz": float(w[2]),
    }


def tilt_from_down(R: Rotation) -> float:
    """Angle [rad] between the tool z-axis and straight down."""
    z_tool = R.apply([0.0, 0.0, 1.0])
    return float(np.arccos(np.clip(-z_tool[2], -1.0, 1.0)))
