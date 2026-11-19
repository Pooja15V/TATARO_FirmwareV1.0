"""
test_calibration.py
-------------------
Verify the saved calibration. Shows the live RealSense image; click any
point and it prints the camera coordinates and the predicted robot
coordinates using calibration_data.pkl.

    Robot = A @ [Xc, Yc] + t

KEYS:
  left-click : convert clicked point
  m          : move to clicked hover pose
  h          : go to home joints
  g          : save current spot as home
  b          : go back to saved home
  q / ESC    : quit
"""

import os
import sys
import pickle

import cv2
import numpy as np

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from camera.realsense_camera import RealSenseCamera
from camera.robot_interface import RobotInterface

HERE = os.path.dirname(os.path.abspath(__file__))
PKL_PATH = os.path.join(HERE, "calibration_data.pkl")

ROBOT_IP = "192.168.1.150"  # <-- set your Lite 6's IP

# -------------------------------------------------------------------------
# Z = TCP_Z - camera_depth + Z_OFFSET
#   TCP_Z        : live robot TCP Z from get_position()
#   camera_depth : aligned depth at the clicked pixel
#   Z_OFFSET     : hover offset (mm)
# -------------------------------------------------------------------------
Z_OFFSET = 70.0

# Home joint configuration (deg) - pressing 'h' sends the robot here.
# Lite 6 default "all zeros" pose; adjust to a safe pose for your setup.
HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def load_calibration(path=PKL_PATH):
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"{path} not found. Run compute_calibration.py first."
        )
    with open(path, "rb") as f:
        data = pickle.load(f)
    return data


def camera_to_robot(A, t, Xc, Yc):
    cam = np.array([Xc, Yc])
    rob = A @ cam + t
    return float(rob[0]), float(rob[1])


def camera_to_robot_xyz(A, t, Xc, Yc, depth_mm, tcp_z, z_offset=Z_OFFSET):
    """Full mapping for downstream projects.

    Returns (Xr, Yr, Z_object):
      Xr, Yr    : robot X/Y from the calibrated affine map (mm)
      Z_object  : Z_object = tcp_z - depth_mm + z_offset
                  where tcp_z is the live TCP Z from get_position().
    """
    Xr, Yr = camera_to_robot(A, t, Xc, Yc)
    Z_object = tcp_z - depth_mm + z_offset
    return Xr, Yr, Z_object


class Tester:
    def __init__(self, cam, A, t, robot=None):
        self.cam = cam
        self.A = A
        self.t = t
        self.robot = robot
        self.last_depth = None
        self.click = None
        self.marker = None       # (u,v) of last click for display
        self.last_target = None  # [X,Y,Z,Rx,Ry,Rz] from the last valid click
        self.saved_home = None   # saved joint config to return to

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.click = (x, y)

    def _handle(self):
        u, v = self.click
        self.click = None
        self.marker = (u, v)
        if self.last_depth is None:
            return
        cam_xyz = self.cam.get_camera_coords_at_pixel(self.last_depth, u, v)
        if cam_xyz is None:
            print(f"No valid depth at ({u},{v}).")
            return
        Xc, Yc, Zc = cam_xyz
        Xr, Yr = camera_to_robot(self.A, self.t, Xc, Yc)

        # Live TCP pose from the robot at this moment.
        pose = None
        if self.robot is not None and self.robot.connected:
            pose = self.robot.get_pose()

        print("\n----------------------------")
        print(f"Pixel:             ({u}, {v})")
        print(f"Camera coordinates: ({Xc:.2f}, {Yc:.2f})  depth={Zc:.1f} mm")
        print(f"Robot X, Y:         ({Xr:.2f}, {Yr:.2f})")
        if pose is None:
            print("Z_object:           (no robot TCP Z - connect robot)")
            self.last_target = None
        else:
            z_tcp = pose[2]
            Z_object = z_tcp - Zc + Z_OFFSET
            # Keep current orientation (Roll, Pitch, Yaw unchanged).
            rx, ry, rz = pose[3], pose[4], pose[5]
            self.last_target = [Xr, Yr, Z_object, rx, ry, rz]
            print(f"TCP Z (robot):      {z_tcp:.1f} mm")
            print(f"Z_object (hover):   {Z_object:.1f} mm  (TCP_Z - depth + {Z_OFFSET:.0f})")
            print(f"Target pose:        X={Xr:.1f} Y={Yr:.1f} Z={Z_object:.1f} "
                  f"Rx={rx:.1f} Ry={ry:.1f} Rz={rz:.1f}")
            print("Press 'm' to MOVE the robot to this hover pose.")

    def _do_move(self):
        if self.robot is None or not self.robot.connected:
            print("[Move] Robot not connected.")
            return
        if self.last_target is None:
            print("[Move] No target yet - click a valid point first.")
            return
        x, y, z, rx, ry, rz = self.last_target
        print(f"\n[Move] Moving to hover pose "
              f"X={x:.1f} Y={y:.1f} Z={z:.1f} (orientation kept).")
        self.robot.move_to(x, y, z, rx, ry, rz)

    def _go_home(self):
        if self.robot is None or not self.robot.connected:
            print("[Home] Robot not connected.")
            return
        print(f"\n[Home] Going to home joints {HOME_JOINTS}...")
        self.robot.move_joints(HOME_JOINTS)

    def _save_home(self):
        if self.robot is None or not self.robot.connected:
            print("[Home] Robot not connected.")
            return
        j = self.robot.get_joints()
        if j is None:
            print("[Home] Could not read joints.")
            return
        self.saved_home = j
        print(f"\n[Home] Saved current spot as home: "
              f"{[round(v,2) for v in j]}")

    def _go_saved_home(self):
        if self.robot is None or not self.robot.connected:
            print("[Home] Robot not connected.")
            return
        if self.saved_home is None:
            print("[Home] No saved home yet - press 'g' to save one first.")
            return
        print("\n[Home] Returning to saved home...")
        self.robot.move_joints(self.saved_home)

    def run(self):
        win = "Test | click=target  m=move  h=home  g=save-home  b=back  q=quit"
        cv2.namedWindow(win)
        cv2.setMouseCallback(win, self.on_mouse)

        print("\n--- KEYS (focus the image window) ---")
        print("  left-click : pick a target point")
        print("  m          : move to clicked hover pose")
        print(f"  h          : go to home joints {HOME_JOINTS}")
        print("  g          : save current spot as home")
        print("  b          : go back to saved home")
        print("  q / ESC    : quit\n")

        while True:
            color, depth = self.cam.get_frames()
            if color is None:
                continue
            self.last_depth = depth
            disp = color.copy()
            if self.marker is not None:
                cv2.drawMarker(disp, self.marker, (0, 0, 255),
                               cv2.MARKER_TILTED_CROSS, 18, 2)
            cv2.imshow(win, disp)

            if self.click is not None:
                self._handle()

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            elif key == ord('m'):
                self._do_move()
            elif key == ord('h'):
                self._go_home()
            elif key == ord('g'):
                self._save_home()
            elif key == ord('b'):
                self._go_saved_home()
        cv2.destroyAllWindows()


def main():
    data = load_calibration()
    A, t = data["A"], data["t"]
    print("Loaded calibration:")
    print("  A =\n", np.array2string(A, precision=5))
    print("  t =", np.array2string(t, precision=4))
    print(f"  RMSE = {data.get('rmse', float('nan')):.3f} mm "
          f"({data.get('n_points', '?')} pts)")

    robot = RobotInterface()
    robot.connect(ROBOT_IP)   # needed for live TCP Z

    with RealSenseCamera() as cam:
        Tester(cam, A, t, robot=robot).run()

    robot.disconnect()


if __name__ == "__main__":
    main()
