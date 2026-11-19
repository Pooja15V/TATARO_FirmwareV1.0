import numpy as np

import cv2
from scipy.spatial.transform import Rotation as Rot

# ----------------------------
# Table -> Robot (measured once)
# ----------------------------

R_robot_table = np.array([
    [ -1,  0,  0],
    [  0,  1,  0],
    [  0,  0, -1]
], dtype=np.float64)

# Robot origin expressed in the TABLE frame (cm)
t_robot_table = np.array([
    [76.0],
    [42.5],
    [0.5]
], dtype=np.float64)


def camera_to_robot(point_cam,P1_3D,rvec_marker_wrt_camera):
    point_cam = np.asarray(point_cam).reshape(3,1)
    P1_3D = np.asarray(P1_3D).reshape(3, 1)
    # Rodrigues -> Rotation Matrix
    R_cam_table, _ = cv2.Rodrigues(rvec_marker_wrt_camera)
    point_table = R_cam_table.T @ (point_cam - P1_3D)
    point_robot = R_robot_table @ (point_table - t_robot_table)
    # print("Marker40 robot =", point_robot.ravel())
    # print("Table coordinates:", point_table.ravel())
    
    return point_robot.flatten()
