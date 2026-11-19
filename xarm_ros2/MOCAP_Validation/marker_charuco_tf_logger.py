"""
Real-time User_Marker + TF chain CSV logger.

Simpler sibling of sync_marker_charuco_logger.py: no SyncPulse hardware, no
direct /charuco/pose subscription. Everything is read straight from tf2 so
the same script covers the robot pose, the published hand-eye calibration,
and the live ChArUco detection at once.

Logs these transforms (position + quaternion) at a fixed sampling rate:
    link_base       -> link_eef                    robot forward kinematics
    link_base       -> camera_color_optical_frame   published hand-eye calibration
                                                     folded through the live FK
                                                     (requires easy_handeye2's
                                                     publish.launch.py running)
    link_base       -> charuco_board_frame          full chain: FK + calibration
                                                     + live marker detection
    camera_color_optical_frame -> charuco_board_frame   raw detection only,
                                                          independent of FK/calibration

Columns:
    Timestamp    - wall-clock time, millisecond precision
    User_Marker  - 1 for 10 samples after the 'S' key is pressed, else 0
    <Name>_Valid - 1 if that transform resolved within max_transform_age_s, else 0
    <Name>_X/Y/Z, <Name>_QX/QY/QZ/QW - position (m) and orientation quaternion

When a transform is invalid (not yet available, or lookup failed), its
position/quaternion columns are written as 0.0 rather than left blank, so
the CSV stays rectangular for downstream tools.

Requires already running:
    ros2 launch xarm_moveit_config lite6_moveit_realmove.launch.py ...   (robot TF)
    ros2 launch easy_handeye2 publish.launch.py name:=<calib_name>       (link_eef -> camera_color_optical_frame)
    ros2 launch aruco_ros charuco_detector.launch.py                    (camera_color_optical_frame -> charuco_board_frame)

Run:
    python3 marker_charuco_tf_logger.py
Stop:
    Ctrl+C

Note: key detection reads directly from the terminal (no root required),
so this must be run in an interactive terminal that has focus.
"""

import csv
import os
import sys
import select
import termios
import time
import tty
from datetime import datetime

import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


# --------------------------------------------------------------------------
# Non-blocking, no-root single-key terminal reader
# --------------------------------------------------------------------------
class TerminalKeyReader:
    """
    Puts stdin into cbreak mode so individual keypresses are available
    immediately (no Enter required) and can be polled without blocking.
    `restore()` must be called before exit to return the terminal to its
    normal (cooked) mode.
    """

    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

    def poll(self):
        """Return the next pending character, or None if nothing is waiting."""
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

    def restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)


# --------------------------------------------------------------------------
# User marker state machine (triggered by the 'S' key)
# --------------------------------------------------------------------------
class UserMarker:
    """
    Tracks the User_Marker column.

    Pressing 'S' (case-insensitive) arms the marker for the next
    `hold_samples` samples. Pressing it again while still armed restarts
    the countdown rather than stacking it.
    """

    def __init__(self, hold_samples: int = 10):
        self.hold_samples = hold_samples
        self._remaining = 0

    def on_key(self, key: str) -> bool:
        """Returns True if this key press (re)armed the marker."""
        if key is not None and key.lower() == "s":
            self._remaining = self.hold_samples
            return True
        return False

    def next_value(self) -> int:
        """Call exactly once per sample; returns this sample's marker value."""
        if self._remaining > 0:
            self._remaining -= 1
            return 1
        return 0


# --------------------------------------------------------------------------
# TF chain reader
# --------------------------------------------------------------------------
class TFChainReader:
    """
    Wraps a single tf2_ros.Buffer/TransformListener and resolves a fixed set
    of (target_frame, source_frame) pairs on demand. `read(target, source)`
    is non-blocking-ish: it uses a short lookup timeout so a single missing
    transform can't stall the logger's fixed sample rate.
    """

    def __init__(self, max_transform_age_s: float = 0.5, node_name: str = "marker_charuco_tf_logger"):
        self.max_transform_age_s = max_transform_age_s

        rclpy.init(args=None)
        self._node = rclpy.create_node(node_name)

        self._buffer = Buffer()
        # spin_thread=True: TransformListener spins this node continuously on
        # its own background thread, so /tf and /tf_static keep getting fed
        # into the buffer regardless of the main loop's timing. A manual
        # single-threaded spin_once(timeout_sec=0.0) once per loop iteration
        # is not a reliable substitute - it only processes a callback that
        # happens to already be ready at that exact instant, and does not
        # run concurrently with lookup_transform()'s own blocking wait.
        self._listener = TransformListener(self._buffer, self._node, spin_thread=True)

    def read(self, target_frame: str, source_frame: str):
        """
        Returns (valid, x, y, z, qx, qy, qz, qw).
        valid is 0 (with all-zero fields) if the transform isn't available,
        or is older than max_transform_age_s.
        """
        try:
            tf = self._buffer.lookup_transform(
                target_frame, source_frame, Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return (0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        stamp_s = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        now_s = self._node.get_clock().now().nanoseconds * 1e-9
        if (now_s - stamp_s) > self.max_transform_age_s:
            return (0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        t, q = tf.transform.translation, tf.transform.rotation
        return (1, t.x, t.y, t.z, q.x, q.y, q.z, q.w)

    def close(self):
        self._node.destroy_node()
        rclpy.shutdown()


# --------------------------------------------------------------------------
# Main logger
# --------------------------------------------------------------------------
class MarkerCharucoTFLogger:
    # (column prefix, target_frame, source_frame)
    CHAINS = [
        ("EEF",         "link_base",                  "link_eef"),
        ("Camera",      "link_base",                  "camera_color_optical_frame"),
        ("CharucoBase", "link_base",                  "charuco_board_frame"),
        ("CharucoCam",  "camera_color_optical_frame", "charuco_board_frame"),
    ]
    def __init__(
        self,
        csv_path: str,
        sample_rate_hz: float = 30.0,
        marker_hold_samples: int = 10,
        flush_every_n_samples: int = 10,
        max_transform_age_s: float = 0.5,
    ):
        self.period_s = 1.0 / sample_rate_hz
        self.flush_every_n_samples = flush_every_n_samples

        self.tf_reader = TFChainReader(max_transform_age_s=max_transform_age_s)
        self.user_marker = UserMarker(hold_samples=marker_hold_samples)
        self.key_reader = TerminalKeyReader()

        self._sample_count = 0
        self.csv_path = csv_path
        self.csv_file = open(csv_path, "w", newline="")
        self.writer = csv.writer(self.csv_file)

        header = ["Timestamp", "User_Marker"]
        for prefix, _, _ in self.CHAINS:
            header += [
                f"{prefix}_Valid",
                f"{prefix}_X", f"{prefix}_Y", f"{prefix}_Z",
                f"{prefix}_QX", f"{prefix}_QY", f"{prefix}_QZ", f"{prefix}_QW",
            ]
        self.writer.writerow(header)

    @staticmethod
    def _timestamp_ms() -> str:
        # Millisecond precision: microseconds truncated to 3 digits.
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

    def run(self):
        print(f"Logging at {1.0 / self.period_s:.1f} Hz -> {self.csv_path}")
        print("Press 'S' to set a marker. Press Ctrl+C to stop.")
        print("Waiting for TF chains: " + ", ".join(f"{t}->{s}" for _, t, s in self.CHAINS))

        reported = {prefix: False for prefix, _, _ in self.CHAINS}
        next_tick = time.perf_counter()
        try:
            while True:
                key = self.key_reader.poll()
                if self.user_marker.on_key(key):
                    print("\n[MARKER] 'S' pressed - User_Marker set for the next "
                          f"{self.user_marker.hold_samples} samples")

                timestamp = self._timestamp_ms()
                marker_value = self.user_marker.next_value()

                row = [timestamp, marker_value]
                for prefix, target, source in self.CHAINS:
                    valid, x, y, z, qx, qy, qz, qw = self.tf_reader.read(target, source)
                    row += [valid, x, y, z, qx, qy, qz, qw]
                    if valid and not reported[prefix]:
                        print(f"\n[TF] {prefix}: {target} -> {source} resolved")
                        reported[prefix] = True

                self.writer.writerow(row)
                self._sample_count += 1

                if self._sample_count % self.flush_every_n_samples == 0:
                    self.csv_file.flush()

                # Fixed-rate scheduling with drift correction: schedule the
                # next tick relative to the previous target, not "now".
                next_tick += self.period_s
                sleep_time = next_tick - time.perf_counter()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    # We fell behind (sample took longer than the period);
                    # resync instead of trying to catch up.
                    next_tick = time.perf_counter()

        except KeyboardInterrupt:
            print("\nInterrupted by user. Shutting down gracefully...")
        finally:
            self.csv_file.flush()
            self.csv_file.close()
            self.key_reader.restore()
            self.tf_reader.close()
            print(f"CSV saved to {self.csv_path}")


if __name__ == "__main__":
    # Save alongside this script (MOCAP_Validation folder) regardless of
    # the directory the script is launched from. Each run gets its own
    # timestamped file so previous runs are never overwritten.
    _SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    _RUN_STAMP = datetime.now().strftime("%Y%m%d_%H%M%S")
    CSV_PATH = os.path.join(_SCRIPT_DIR, f"marker_charuco_tf_log_{_RUN_STAMP}.csv")
    SAMPLE_RATE_HZ = 30  # easy to change: samples per second
    MAX_TRANSFORM_AGE_S = 0.5  # transforms older than this are marked invalid

    logger = MarkerCharucoTFLogger(
        csv_path=CSV_PATH,
        sample_rate_hz=SAMPLE_RATE_HZ,
        marker_hold_samples=10,
        max_transform_age_s=MAX_TRANSFORM_AGE_S,
    )
    logger.run()
