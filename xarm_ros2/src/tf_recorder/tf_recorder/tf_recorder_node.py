#!/usr/bin/env python3
"""
TF Transform Recorder for Hand-Eye Calibration.

Eye-in-hand configuration: RealSense D435i mounted on xArm Lite6 TCP.
Captures synchronized TF transforms on [S] keypress.
Collected pairs (T_base_tcp, T_camera_marker) are used to solve AX = XB
for the fixed transform T_tcp_camera.

Usage:
    ros2 run tf_recorder tf_recorder_node [/path/to/output.csv] [--ros-args -p charuco_board_frame:=frame_name]
    ros2 launch tf_recorder record.launch.py [output_csv:=/path/to/output.csv] [charuco_board_frame:=frame_name]

Parameters:
    charuco_board_frame (string, default='aruco_marker_frame'): TF frame name for ChArUco board detection
"""

import csv
import os
import sys
import threading
import termios
import tty
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ExtrapolationException, ConnectivityException
from geometry_msgs.msg import TransformStamped


# ── Configuration ──────────────────────────────────────────────────────────────

# Robot transform pairs (target_frame, source_frame, csv_column_prefix)
# lookup_transform(target, source) → T_target_source (source expressed in target)
ROBOT_TRANSFORM_PAIRS: Tuple[Tuple[str, str, str], ...] = (
    ("link_base",  "link6",     "lb_l6"),
    ("link_base",  "link_eef",  "lb_eef"),
    ("link_base",  "link_tcp",  "lb_tcp"),
)

TF_TIMEOUT_S: float = 0.5    # seconds to wait per TF lookup
MARKER_MAX_AGE_S: float = 1.0  # reject if ChArUco detection is older than this
ROBOT_MAX_AGE_S: float = 0.5   # reject if robot TF is older than this (stale after manual move)
SETTLE_DELAY_S: float = 1.0    # wait after S press before capturing (lets TF settle after web-interface move)
FRAMES_PER_SAMPLE: int = 20    # consecutive frames to record per S keypress
FRAME_INTERVAL_S: float = 0.05 # seconds between frames (~20 Hz)


# ── CSV schema ─────────────────────────────────────────────────────────────────

def build_csv_header(transform_pairs: Tuple[Tuple[str, str, str], ...]) -> List[str]:
    cols = ["sample_id", "timestamp_sec", "timestamp_nanosec"]
    for _, _, pfx in transform_pairs:
        cols += [f"{pfx}_x",  f"{pfx}_y",  f"{pfx}_z"]
        cols += [f"{pfx}_qx", f"{pfx}_qy", f"{pfx}_qz", f"{pfx}_qw"]
    return cols


def build_pose_log_header() -> List[str]:
    """Header for robot_poses_*.csv — just TCP/EEF for automation."""
    return [
        "sample_id", "timestamp_sec", "timestamp_nanosec",
        "link_tcp_x", "link_tcp_y", "link_tcp_z",
        "link_tcp_qx", "link_tcp_qy", "link_tcp_qz", "link_tcp_qw",
    ]


# ── Node ───────────────────────────────────────────────────────────────────────

class TFRecorderNode(Node):
    """
    Keyboard-driven TF capture node.

    On [S]:
      1. Captures FRAMES_PER_SAMPLE consecutive frames at ~20 Hz.
      2. Each frame: looks up all transforms and validates ArUco marker freshness.
      3. Writes each frame as its own CSV row with sample_id format {pose}.{frame}
         e.g. pose 0 → rows 0.1, 0.2, … 0.20
    """

    def __init__(self, output_csv: str) -> None:
        super().__init__("tf_recorder")

        # Declare charuco_board_frame parameter
        self.declare_parameter('charuco_board_frame', 'aruco_marker_frame')
        charuco_frame = self.get_parameter('charuco_board_frame').value

        # Build transform pairs with the ChArUco board frame
        # Each entry: (target_frame, source_frame, csv_column_prefix)
        self._transform_pairs: Tuple[Tuple[str, str, str], ...] = (
            *ROBOT_TRANSFORM_PAIRS,
            ("camera_color_optical_frame", charuco_frame, "cam_marker"),
        )

        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._output_csv = Path(output_csv)
        self._csv_lock   = threading.Lock()
        self._sample_id: int  = 0
        self._running:   bool = True

        self._init_csv()

        threading.Thread(target=self._keyboard_loop, daemon=True).start()

        self.get_logger().info(
            "\n" + "=" * 64 + "\n"
            "  Hand-Eye TF Recorder  |  xArm Lite6 + RealSense D435i\n"
            + f"  Output : {self._output_csv.resolve()}\n"
            + f"  ChArUco frame: {charuco_frame}\n"
            "  Frames :\n"
            + "".join(f"    {p[0]}  ←  {p[1]}\n" for p in self._transform_pairs)
            + "  Keys   :  [S] capture     [Q / Ctrl+C] quit\n"
            + "=" * 64
        )

    # ── CSV initialisation ────────────────────────────────────────────────────

    def _init_csv(self) -> None:
        new_file = (
            not self._output_csv.exists()
            or self._output_csv.stat().st_size == 0
        )
        if new_file:
            self._output_csv.parent.mkdir(parents=True, exist_ok=True)
            with open(self._output_csv, "w", newline="") as f:
                csv.writer(f).writerow(build_csv_header(self._transform_pairs))
            self.get_logger().info(f"Created new CSV: {self._output_csv}")
        else:
            with open(self._output_csv, "r") as f:
                existing_rows = max(0, sum(1 for _ in f) - 1)  # minus header row
            self._sample_id = existing_rows // FRAMES_PER_SAMPLE
            self.get_logger().info(
                f"Appending to: {self._output_csv}  "
                f"({existing_rows} frames / {self._sample_id} poses already saved)"
            )

    # ── TF lookup ─────────────────────────────────────────────────────────────

    def _lookup(
        self, target: str, source: str
    ) -> Tuple[Optional[TransformStamped], Optional[str]]:
        try:
            tf = self._tf_buffer.lookup_transform(
                target,
                source,
                rclpy.time.Time(),  # latest available
                timeout=rclpy.duration.Duration(seconds=TF_TIMEOUT_S),
            )
            return tf, None
        except (LookupException, ExtrapolationException, ConnectivityException) as exc:
            return None, str(exc)

    # ── Capture ───────────────────────────────────────────────────────────────

    def _capture(self) -> None:
        pid = self._sample_id

        self.get_logger().info(
            f"[Pose {pid:04d}] Capturing {FRAMES_PER_SAMPLE} frames..."
        )

        import time

        self.get_logger().info(
            f"[Pose {pid:04d}] Settling for {SETTLE_DELAY_S}s..."
        )
        time.sleep(SETTLE_DELAY_S)

        for frame_idx in range(FRAMES_PER_SAMPLE):
            if frame_idx > 0:
                time.sleep(FRAME_INTERVAL_S)

            now = self.get_clock().now()
            ts  = now.to_msg()

            results: Dict[str, TransformStamped] = {}
            errors:  List[str] = []

            for target, source, prefix in self._transform_pairs:
                tf, err = self._lookup(target, source)
                if tf is None:
                    errors.append(f"    {target}  ←  {source}   [{err}]")
                else:
                    results[prefix] = tf

            if errors:
                self.get_logger().warn(
                    f"[Pose {pid:04d}] Frame {frame_idx+1}/{FRAMES_PER_SAMPLE} "
                    "REJECTED — unavailable transform(s)"
                )
                return

            # Check robot TF freshness — guards against stale joint states
            # after a manual move via the uFactory web interface
            robot_stamp = rclpy.time.Time.from_msg(
                results["lb_tcp"].header.stamp
            )
            robot_age_s = (now - robot_stamp).nanoseconds * 1e-9
            if robot_age_s > ROBOT_MAX_AGE_S:
                self.get_logger().warn(
                    f"[Pose {pid:04d}] Frame {frame_idx+1}/{FRAMES_PER_SAMPLE} "
                    f"REJECTED — robot TF {robot_age_s:.2f}s stale (joint states not updating?)"
                )
                return

            marker_stamp = rclpy.time.Time.from_msg(
                results["cam_marker"].header.stamp
            )
            marker_age_s = (now - marker_stamp).nanoseconds * 1e-9

            if marker_age_s > MARKER_MAX_AGE_S:
                self.get_logger().warn(
                    f"[Pose {pid:04d}] Frame {frame_idx+1}/{FRAMES_PER_SAMPLE} "
                    f"REJECTED — ArUco marker {marker_age_s:.2f}s stale"
                )
                return

            sample_id = f"{pid}.{frame_idx + 1}"
            row: List = [sample_id, ts.sec, ts.nanosec]

            for _, _, prefix in self._transform_pairs:
                t = results[prefix].transform.translation
                q = results[prefix].transform.rotation
                row += [t.x, t.y, t.z, q.x, q.y, q.z, q.w]

            with self._csv_lock:
                with open(self._output_csv, "a", newline="") as f:
                    csv.writer(f).writerow(row)
                    f.flush()
                    os.fsync(f.fileno())

        self._sample_id += 1
        tcp = results["lb_tcp"].transform.translation
        self.get_logger().info(
            f"[Pose {pid:04d}] SAVED {FRAMES_PER_SAMPLE} frames  "
            f"TCP=({tcp.x:.4f}, {tcp.y:.4f}, {tcp.z:.4f})"
        )

    # ── Keyboard thread ───────────────────────────────────────────────────────

    def _keyboard_loop(self) -> None:
        """
        Raw-mode stdin listener.  Runs in its own daemon thread so it does not
        block the ROS spin loop.  Terminal state is always restored on exit.
        """
        fd  = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while self._running and rclpy.ok():
                ch = sys.stdin.read(1)
                if ch.lower() == "s":
                    self._capture()
                elif ch.lower() == "q" or ch == "\x03":  # q or Ctrl+C
                    self.get_logger().info("Quit key received — shutting down.")
                    rclpy.shutdown()
                    break
        except Exception as exc:
            self.get_logger().error(f"Keyboard thread error: {exc}")
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def destroy_node(self) -> None:
        self._running = False
        super().destroy_node()


# ── Entry point ────────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)

    default_csv = os.path.join(
        "/home/pooja/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0"
        "/xarm_ros2/install/tf_recorder/TF_Records",
        f"handeye_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
    )
    output_csv = sys.argv[1] if len(sys.argv) > 1 else default_csv

    node = TFRecorderNode(output_csv)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
