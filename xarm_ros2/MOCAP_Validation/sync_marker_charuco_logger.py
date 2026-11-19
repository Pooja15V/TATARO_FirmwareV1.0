"""
Real-time SyncPulse + User_Marker + ChArUco-pose CSV logger.

Extends sync_marker_logger.py with the ChArUco board pose expressed in the
camera optical frame (camera_color_optical_frame by default), as published
by aruco_ros's charuco_detector_node on /charuco/pose. This lets a single
CSV carry the hardware sync line, the manual event marker, and the vision
pose sample all on the same 100 Hz clock for later alignment against MOCAP.

Logs these columns at a fixed sampling rate:
    Timestamp      - wall-clock time, millisecond precision
    Sync_Data      - 1/0 from the SyncPulse hardware over serial
    User_Marker    - 1 for 10 samples after the 'S' key is pressed, else 0
    Charuco_Valid  - 1 if a ChArUco pose arrived within max_pose_age_s, else 0
    Charuco_X/Y/Z          - board position in the camera optical frame (m)
    Charuco_QX/QY/QZ/QW    - board orientation in the camera optical frame
    Charuco_RX/RY/RZ       - same orientation as a Rodrigues rotation vector
                             (radians), taken straight from
                             estimatePoseCharucoBoard rather than
                             reconstructed from the quaternion, so it
                             matches OpenCV's rvec bit-for-bit

When Charuco_Valid is 0 (board not detected, or the last detection is
stale), the pose columns are written as 0.0 rather than left blank, so the
CSV stays rectangular for downstream tools.

Requires charuco_detector_node (aruco_ros package) already running and
publishing /charuco/pose and /charuco/rvec from a live camera feed:
    ros2 run aruco_ros charuco_detector_node.py
    ros2 launch aruco_ros charuco_detector.launch.py

Run:
    python3 sync_marker_charuco_logger.py
Stop:
    Ctrl+C

Note: key detection reads directly from the terminal (no root required),
so this must be run in an interactive terminal that has focus.
"""

import csv
import os
import struct
import time
import sys
import select
import termios
import threading
import tty
from datetime import datetime

import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped, Vector3Stamped

from syncpulse_reader import SerialPort


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
# SyncPulse hardware interface
# --------------------------------------------------------------------------
class SyncPulseReader:
    """
    Reads SyncPulse frames using syncpulse_reader.py's SerialPort directly
    (itself an exact copy of the confirmed-working ACTIBO_CV_SyncPulse.py) -
    no reimplementation of the frame protocol here. SerialPort.serial_read()
    is blocking, so it runs on a background thread; `read()` just returns
    the most recently decoded value and is non-blocking, safe to call once
    per sample from the main loop.
    """

    def __init__(self, serialport: str = "/dev/ttyACM0", baudrate: int = 115200):
        # csv_enable=False (the default) - SerialPort won't touch any file;
        # this logger owns its own CSV writing.
        self.port = SerialPort(serialport=serialport, serialrate=baudrate, dof=6)

        self._last_value = 0
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        while not self._stop_event.is_set():
            try:
                valid = self.port.serial_read()
            except (OSError, ValueError):
                break  # port was closed
            if valid:
                sync_char = struct.unpack("c", self.port.payload)[0].decode("utf-8")
                with self._lock:
                    self._last_value = int(sync_char) if sync_char in ("0", "1") else 0

    def read(self) -> int:
        """Non-blocking: returns the most recently received raw value."""
        with self._lock:
            return self._last_value

    def close(self):
        self._stop_event.set()
        self.port.ser_port.close()  # unblocks the thread's pending read
        self._thread.join(timeout=1.0)


# --------------------------------------------------------------------------
# ChArUco pose interface (ROS 2)
# --------------------------------------------------------------------------
class ChArucoPoseReader:
    """
    Subscribes to /charuco/pose (geometry_msgs/PoseStamped) and /charuco/rvec
    (geometry_msgs/Vector3Stamped), both published by aruco_ros's
    charuco_detector_node in the camera optical frame from the same
    detection - same header.stamp, quaternion and rvec just being two
    representations of the same rotation. rclpy's executor.spin() is
    blocking, so it runs on a background thread; `read()` just returns the
    most recently cached values and is non-blocking, safe to call once per
    sample from the main loop.

    A sample is only reported valid if both messages arrived within
    `max_pose_age_s` of now - this stops the logger from silently repeating
    a stale detection (e.g. after the board is occluded or out of frame) as
    if it were live.
    """

    POSE_TOPIC = "/charuco/pose"
    RVEC_TOPIC = "/charuco/rvec"

    def __init__(self, max_pose_age_s: float = 0.5, node_name: str = "sync_marker_charuco_logger"):
        self.max_pose_age_s = max_pose_age_s

        rclpy.init(args=None)
        self._node = rclpy.create_node(node_name)

        self._lock = threading.Lock()
        self._last_pose = None        # (x, y, z, qx, qy, qz, qw)
        self._last_pose_stamp_s = None
        self._last_rvec = None        # (rx, ry, rz), Rodrigues vector, radians
        self._last_rvec_stamp_s = None
        self._frame_id = None         # camera optical frame reported by the publisher

        self._node.create_subscription(
            PoseStamped, self.POSE_TOPIC, self._pose_callback, 10
        )
        self._node.create_subscription(
            Vector3Stamped, self.RVEC_TOPIC, self._rvec_callback, 10
        )

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()

    def _pose_callback(self, msg: PoseStamped):
        p, o = msg.pose.position, msg.pose.orientation
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            self._last_pose = (p.x, p.y, p.z, o.x, o.y, o.z, o.w)
            self._last_pose_stamp_s = stamp_s
            self._frame_id = msg.header.frame_id

    def _rvec_callback(self, msg: Vector3Stamped):
        v = msg.vector
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            self._last_rvec = (v.x, v.y, v.z)
            self._last_rvec_stamp_s = stamp_s

    def frame_id(self):
        """Camera optical frame the last received pose was expressed in, or None."""
        with self._lock:
            return self._frame_id

    def read(self):
        """
        Non-blocking. Returns (valid, x, y, z, qx, qy, qz, qw, rx, ry, rz).
        valid is 0 (with all-zero fields) if pose and/or rvec hasn't been
        received yet, or the last one is older than max_pose_age_s.
        """
        with self._lock:
            pose, pose_stamp_s = self._last_pose, self._last_pose_stamp_s
            rvec, rvec_stamp_s = self._last_rvec, self._last_rvec_stamp_s

        if pose is None or rvec is None:
            return (0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        now_s = self._node.get_clock().now().nanoseconds * 1e-9
        pose_fresh = (now_s - pose_stamp_s) <= self.max_pose_age_s
        rvec_fresh = (now_s - rvec_stamp_s) <= self.max_pose_age_s
        valid = 1 if (pose_fresh and rvec_fresh) else 0
        return (valid, *pose, *rvec)

    def close(self):
        self._executor.shutdown()
        self._node.destroy_node()
        rclpy.shutdown()
        self._thread.join(timeout=1.0)


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
# Main logger
# --------------------------------------------------------------------------
class SyncMarkerCharucoLogger:
    def __init__(
        self,
        csv_path: str,
        sample_rate_hz: float = 100.0,
        marker_hold_samples: int = 10,
        flush_every_n_samples: int = 10,
        serial_port: str = "/dev/ttyACM0",
        serial_baud: int = 115200,
        max_pose_age_s: float = 0.5,
    ):
        self.period_s = 1.0 / sample_rate_hz
        self.flush_every_n_samples = flush_every_n_samples

        self.sync_reader = SyncPulseReader(serial_port, serial_baud)
        self.charuco_reader = ChArucoPoseReader(max_pose_age_s=max_pose_age_s)
        self.user_marker = UserMarker(hold_samples=marker_hold_samples)
        self.key_reader = TerminalKeyReader()

        self._sample_count = 0
        self.csv_path = csv_path
        self.csv_file = open(csv_path, "w", newline="")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            "Timestamp", "Sync_Data", "User_Marker",
            "Charuco_Valid",
            "Charuco_X", "Charuco_Y", "Charuco_Z",
            "Charuco_QX", "Charuco_QY", "Charuco_QZ", "Charuco_QW",
            "Charuco_RX", "Charuco_RY", "Charuco_RZ",
        ])

    @staticmethod
    def _timestamp_ms() -> str:
        # Millisecond precision: microseconds truncated to 3 digits.
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

    def run(self):
        print(f"Logging at {1.0 / self.period_s:.1f} Hz -> {self.csv_path}")
        print("Press 'S' to set a marker. Press Ctrl+C to stop.")
        print("Waiting for /charuco/pose ...")

        next_tick = time.perf_counter()
        frame_id_reported = False
        try:
            while True:
                key = self.key_reader.poll()
                if self.user_marker.on_key(key):
                    print("\n[MARKER] 'S' pressed - User_Marker set for the next "
                          f"{self.user_marker.hold_samples} samples")

                timestamp = self._timestamp_ms()
                sync_value = self.sync_reader.read()
                marker_value = self.user_marker.next_value()
                (charuco_valid, cx, cy, cz, cqx, cqy, cqz, cqw,
                 crx, cry, crz) = self.charuco_reader.read()

                if charuco_valid and not frame_id_reported:
                    print(f"\n[CHARUCO] receiving pose in frame '{self.charuco_reader.frame_id()}'")
                    frame_id_reported = True

                self.writer.writerow([
                    timestamp, sync_value, marker_value,
                    charuco_valid, cx, cy, cz, cqx, cqy, cqz, cqw,
                    crx, cry, crz,
                ])
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
            self.sync_reader.close()
            self.charuco_reader.close()
            print(f"CSV saved to {self.csv_path}")


if __name__ == "__main__":
    # Save alongside this script (MOCAP_Validation folder) regardless of
    # the directory the script is launched from. Each run gets its own
    # timestamped file so previous runs are never overwritten.
    _SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    _RUN_STAMP = datetime.now().strftime("%Y%m%d_%H%M%S")
    CSV_PATH = os.path.join(_SCRIPT_DIR, f"sync_marker_charuco_log_{_RUN_STAMP}.csv")
    SAMPLE_RATE_HZ = 100  # easy to change: samples per second
    SERIAL_PORT = "/dev/ttyACM0"
    SERIAL_BAUD = 115200
    MAX_POSE_AGE_S = 0.5  # ChArUco detections older than this are marked invalid

    logger = SyncMarkerCharucoLogger(
        csv_path=CSV_PATH,
        sample_rate_hz=SAMPLE_RATE_HZ,
        marker_hold_samples=10,
        serial_port=SERIAL_PORT,
        serial_baud=SERIAL_BAUD,
        max_pose_age_s=MAX_POSE_AGE_S,
    )
    logger.run()
