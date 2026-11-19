"""Compose T_base_cam (m) for the overhead D455 from the existing TATARO CV chain:
camera -> table (markers 51/52/53, GSO) -> robot (TATARO_CV_Transformations).
Read-only: subscribes to colour + aligned depth, averages N frames, writes an annotated check image."""
import os, sys
os.environ.setdefault('FASTRTPS_DEFAULT_PROFILES_FILE', os.path.expanduser('~/.tataro_fastdds_bigbuf.xml'))
CV = os.path.expanduser('~/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0/xarm_ros2/src/computer_vision')
sys.path.insert(0, CV)
import numpy as np, cv2, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from TATARO_CV_TableCropping_with_ARUCO_7x7 import detect_and_transform_table_using_aruco
from TATARO_CV_Transformations import R_robot_table, t_robot_table

N = 15
OUT = sys.argv[1]

class Grab(Node):
    def __init__(s):
        super().__init__('overhead_T_base_cam_grab')
        s.color = s.depth = s.K = None
        s.create_subscription(Image, '/camera/camera/color/image_raw', s.c, qos_profile_sensor_data)
        s.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', s.d, qos_profile_sensor_data)
        s.create_subscription(CameraInfo, '/camera/camera/color/camera_info', s.i, qos_profile_sensor_data)
    def c(s, m):
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
        s.color = cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if m.encoding == 'rgb8' else a.copy()
    def d(s, m): s.depth = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).copy()
    def i(s, m): s.K = np.array(m.k).reshape(3, 3)

rclpy.init(); n = Grab()
Ts, last, tries = [], None, 0
for _ in range(2000):
    rclpy.spin_once(n, timeout_sec=0.1)
    if n.color is None or n.depth is None: continue
    tries += 1
    r = detect_and_transform_table_using_aruco(n.color.copy(), n.depth)
    n.color = None
    if r is None:
        if tries % 20 == 0: print('frames tried:', tries, '- markers 51/52/53 not all found yet', flush=True)
        continue
    rvec_ct, P1 = r[7], r[10]                       # GSO table rotation, table origin (cm, camera frame)
    R_ct, _ = cv2.Rodrigues(rvec_ct)
    R_bc = R_robot_table @ R_ct.T                   # p_base = R_bc p_cam + t_bc  (cm)
    t_bc = -R_bc @ P1.reshape(3, 1) - R_robot_table @ t_robot_table
    T = np.eye(4); T[:3, :3] = R_bc; T[:3, 3] = t_bc.ravel() / 100.0
    Ts.append(T); last = r[0]
    if len(Ts) >= N: break
print('frames used:', len(Ts), 'of', tries, ' live fx=%.2f' % n.K[0, 0] if n.K is not None else '')
if not Ts: sys.exit('markers 51/52/53 not all detected')
A = np.array(Ts)
t = A[:, :3, 3]
U, _, Vt = np.linalg.svd(A[:, :3, :3].mean(0)); R = U @ Vt
T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t.mean(0)
np.set_printoptions(suppress=True, precision=6)
print('translation std (mm):', (t.std(0) * 1000).round(2))
print('T_base_cam (m):\n', T)
print('camera position in base (mm):', (T[:3, 3] * 1000).round(1))
print('camera optical +Z (view dir) in base:', R[:, 2].round(3), ' (expect ~[0,0,-1] for straight down)')
Tcb = np.linalg.inv(T); K = n.K
img = last.copy()
def proj(p):
    pc = Tcb[:3, :3] @ p + Tcb[:3, 3]
    uv = K @ pc; return (int(uv[0] / uv[2]), int(uv[1] / uv[2])), pc[2]
o, zo = proj(np.zeros(3))
for ax, col in [((0.1, 0, 0), (0, 0, 255)), ((0, 0.1, 0), (0, 255, 0)), ((0, 0, 0.1), (255, 0, 0))]:
    cv2.line(img, o, proj(np.array(ax))[0], col, 3)
cv2.circle(img, o, 8, (0, 255, 255), -1)
cv2.putText(img, 'link_base origin (X red, Y green, Z blue)', (o[0] + 10, o[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
cv2.imwrite(OUT, img)
print('base origin pixel:', o, ' depth from camera (mm): %.0f' % (zo * 1000))
np.savetxt(OUT.replace('.png', '.txt'), T, fmt='%.6f')
