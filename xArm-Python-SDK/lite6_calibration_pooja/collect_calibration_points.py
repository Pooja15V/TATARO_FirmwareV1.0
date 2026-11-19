"""
collect_calibration_points.py
-----------------------------
Interactive collection of camera<->robot calibration pairs for an
eye-in-hand setup (camera mounted looking straight down on the TCP),
targeting a UFACTORY Lite 6.

WORKFLOW per point:
  1. Jog the robot to a target point. Lite 6 has no physical teach
     pendant, so the robot goes into manual/freedrive mode (joint
     teaching) and you hand-guide it, or jog it in UFACTORY Studio.
  2. Read robot (Xr, Yr):
       - automatically via get_position() if the robot is connected, OR
       - typed in manually from UFACTORY Studio's pose display.
  3. Click that SAME physical point in the live camera window.
  4. The pixel is converted to camera coords (Xc, Yc) using RealSense
     intrinsics + aligned depth.
  5. The pair (Xc, Yc) <-> (Xr, Yr) is stored.
  Repeat for 10-20 well-spread points.

OUTPUT: calibration/calibration_points.csv
  columns: camera_x, camera_y, robot_x, robot_y

KEYS (focus the OpenCV window):
  left-click : capture the clicked point (asks for robot XY)
  u          : undo the last captured point
  s          : save CSV now
  q / ESC    : save and quit
"""

import os
import csv
import sys

import cv2
import numpy as np

# Allow running directly: add project root to path.
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from camera.realsense_camera import RealSenseCamera
from camera.robot_interface import RobotInterface

CSV_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "calibration_points.csv")

ROBOT_IP = "192.168.1.150"  # <-- set your Lite 6's IP


class Collector:
    def __init__(self, cam, robot):
        self.cam = cam
        self.robot = robot
        self.points = []           # list of dict(camera_x, camera_y, robot_x, robot_y)
        self.last_depth_frame = None
        self.click_pixel = None    # (u, v) pending confirmation

    # -------------------------------------------------------------- #
    def _ask_robot_xy(self):
        """Get robot XY: auto from robot if available, else typed in."""
        if self.robot.connected:
            xy = self.robot.get_xy()
            if xy is not None:
                use = input(f"  Robot reports X={xy[0]:.2f} Y={xy[1]:.2f}. "
                            f"Use it? [Y/n/manual]: ").strip().lower()
                if use in ("", "y", "yes"):
                    return xy
        # Manual entry fallback.
        try:
            xr = float(input("  Enter robot X (mm): ").strip())
            yr = float(input("  Enter robot Y (mm): ").strip())
            return xr, yr
        except ValueError:
            print("  Invalid number; point discarded.")
            return None

    # -------------------------------------------------------------- #
    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.click_pixel = (x, y)

    # -------------------------------------------------------------- #
    def _handle_click(self):
        u, v = self.click_pixel
        self.click_pixel = None

        if self.last_depth_frame is None:
            print("  No depth frame yet; try again.")
            return

        cam_xyz = self.cam.get_camera_coords_at_pixel(self.last_depth_frame, u, v)
        if cam_xyz is None:
            print(f"  No valid depth at pixel ({u},{v}); reclick on a solid surface.")
            return
        Xc, Yc, Zc = cam_xyz
        print(f"\n[Point {len(self.points)+1}] pixel=({u},{v}) "
              f"camera=({Xc:.2f}, {Yc:.2f}, Z={Zc:.1f} mm)")

        xy = self._ask_robot_xy()
        if xy is None:
            return
        Xr, Yr = xy

        self.points.append(dict(camera_x=Xc, camera_y=Yc, robot_x=Xr, robot_y=Yr))
        print(f"  Stored. Total points: {len(self.points)}")

    # -------------------------------------------------------------- #
    def save(self):
        with open(CSV_PATH, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=["camera_x", "camera_y",
                                              "robot_x", "robot_y"])
            w.writeheader()
            for p in self.points:
                w.writerow({k: f"{p[k]:.4f}" for k in p})
        print(f"[Saved] {len(self.points)} points -> {CSV_PATH}")

    # -------------------------------------------------------------- #
    def run(self):
        win = "Calibration - click target point | u=undo s=save q=quit"
        cv2.namedWindow(win)
        cv2.setMouseCallback(win, self.on_mouse)

        print("\nReady. Jog robot, then click the matching point in the image.\n")

        while True:
            color, depth = self.cam.get_frames()
            if color is None:
                continue
            self.last_depth_frame = depth

            disp = color.copy()
            cv2.putText(disp, f"Captured: {len(self.points)}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            h, w = disp.shape[:2]
            cv2.drawMarker(disp, (w // 2, h // 2), (255, 0, 0),
                           cv2.MARKER_CROSS, 20, 1)
            cv2.imshow(win, disp)

            if self.click_pixel is not None:
                self._handle_click()

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):       # q or ESC
                break
            elif key == ord('u'):
                if self.points:
                    self.points.pop()
                    print(f"  Undo. Total points: {len(self.points)}")
            elif key == ord('s'):
                self.save()

        cv2.destroyAllWindows()
        if self.points:
            self.save()


def main():
    robot = RobotInterface()
    if robot.connect(ROBOT_IP):        # ok if it fails; manual entry still works
        robot.enable_manual_mode()     # freedrive: hand-guide the arm to jog it

    with RealSenseCamera() as cam:
        Collector(cam, robot).run()

    robot.disconnect()
    print("\nDone. Next: run compute_calibration.py")


if __name__ == "__main__":
    main()
