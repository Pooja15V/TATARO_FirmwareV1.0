"""
TATARO CV — Gram-Schmidt Orthonormalization
===========================================
Ported from ACTIBO_CV_GSO.py — pyrealsense2 dependency removed.
Provides the reference-frame construction used in table cropping.
"""

import cv2
import numpy as np


def gram_schmidt_orthonormalization(tvec_marker1, tvec_marker3, tvec_marker2):
    """
    Build a 4x4 homogeneous transformation matrix from three ArUco marker positions.

    Convention (matches ACTIBO_CV_TableCropping_with_ARUCO_7x7.py):
        tvec_marker1 — bottom-left  marker (ID 51)
        tvec_marker3 — bottom-right marker (ID 52)  [passed as 3rd arg for GSO x-axis]
        tvec_marker2 — top-left    marker (ID 53)  [passed as 2nd arg for GSO y-axis]

    Call site in table cropping:
        gram_schmidt_orthonormalization(tvec_marker1, tvec_marker3, tvec_marker2)
                                         ^BL            ^BR            ^TL

    Returns
    -------
    np.ndarray (4, 4) float64 — homogeneous transformation matrix
        [:3, :3]  rotation    (orthonormal, right-handed)
        [:3,  3]  translation (= bottom-left marker position, camera frame)
    """
    tvec_marker1 = tvec_marker1.reshape(-1)
    tvec_marker3 = tvec_marker3.reshape(-1)
    tvec_marker2 = tvec_marker2.reshape(-1)

    p_bottom_left  = tvec_marker1
    p_bottom_right = tvec_marker3
    p_top_left     = tvec_marker2

    x_axis = p_bottom_right - p_bottom_left
    x_axis /= np.linalg.norm(x_axis)

    y_raw  = p_top_left - p_bottom_left
    y_axis = y_raw - np.dot(y_raw, x_axis) * x_axis
    y_axis /= np.linalg.norm(y_axis)

    z_axis = np.cross(x_axis, y_axis)
    if np.dot(z_axis, np.cross(x_axis, y_raw)) < 0:
        z_axis = -z_axis
    z_axis /= np.linalg.norm(z_axis)

    # Re-orthogonalise y to guarantee strict right-handed frame
    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)

    R = np.column_stack((x_axis, y_axis, z_axis))

    homogeneous_transformation_matrix = np.eye(4)
    homogeneous_transformation_matrix[:3, :3] = R
    homogeneous_transformation_matrix[:3,  3] = p_bottom_left

    return homogeneous_transformation_matrix


def qr_orthonormalization(tvec_marker1, tvec_marker3, tvec_marker2):
    """
    QR-decomposition alternative to gram_schmidt_orthonormalization.
    Same call signature and return format.
    """
    tvec_marker1 = tvec_marker1.reshape(-1, 1)
    tvec_marker3 = tvec_marker3.reshape(-1, 1)
    tvec_marker2 = tvec_marker2.reshape(-1, 1)

    x_axis = (tvec_marker3 - tvec_marker1).flatten()
    y_axis = (tvec_marker2 - tvec_marker1).flatten()
    A = np.column_stack((x_axis, y_axis, np.cross(x_axis, y_axis)))
    Q, _ = np.linalg.qr(A)

    x_ortho, y_ortho = Q[:, 0], Q[:, 1]
    z_ortho = np.cross(x_ortho, y_ortho)

    if np.dot(x_ortho, x_axis) < 0:
        x_ortho = -x_ortho
    if np.dot(z_ortho, np.cross(x_axis, y_axis)) < 0:
        z_ortho = -z_ortho

    homogeneous_transformation_matrix = np.eye(4)
    homogeneous_transformation_matrix[:3, 0] = x_ortho
    homogeneous_transformation_matrix[:3, 1] = y_ortho
    homogeneous_transformation_matrix[:3, 2] = z_ortho
    homogeneous_transformation_matrix[:3, 3] = tvec_marker1.flatten()

    return homogeneous_transformation_matrix
