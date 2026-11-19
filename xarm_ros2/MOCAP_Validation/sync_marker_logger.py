"""
Real-time SyncPulse + User_Marker CSV logger.

Logs three columns at a fixed sampling rate:
    Timestamp    - wall-clock time, millisecond precision
    Sync_Data    - 1/0 from the SyncPulse hardware over serial
    User_Marker  - 1 for 10 samples after the 'S' key is pressed, else 0

Run:
    python3 sync_marker_logger.py
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
class SyncMarkerLogger:
    def __init__(
        self,
        csv_path: str,
        sample_rate_hz: float = 100.0,
        marker_hold_samples: int = 10,
        flush_every_n_samples: int = 10,
        serial_port: str = "/dev/ttyACM0",
        serial_baud: int = 115200,
    ):
        self.period_s = 1.0 / sample_rate_hz
        self.flush_every_n_samples = flush_every_n_samples

        self.sync_reader = SyncPulseReader(serial_port, serial_baud)
        self.user_marker = UserMarker(hold_samples=marker_hold_samples)
        self.key_reader = TerminalKeyReader()

        self._sample_count = 0
        self.csv_path = csv_path
        self.csv_file = open(csv_path, "w", newline="")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(["Timestamp", "Sync_Data", "User_Marker"])

    @staticmethod
    def _timestamp_ms() -> str:
        # Millisecond precision: microseconds truncated to 3 digits.
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

    def run(self):
        print(f"Logging at {1.0 / self.period_s:.1f} Hz -> {self.csv_path}")
        print("Press 'S' to set a marker. Press Ctrl+C to stop.")

        next_tick = time.perf_counter()
        try:
            while True:
                key = self.key_reader.poll()
                if self.user_marker.on_key(key):
                    print("\n[MARKER] 'S' pressed - User_Marker set for the next "
                          f"{self.user_marker.hold_samples} samples")

                timestamp = self._timestamp_ms()
                sync_value = self.sync_reader.read()
                marker_value = self.user_marker.next_value()

                self.writer.writerow([timestamp, sync_value, marker_value])
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
            print(f"CSV saved to {self.csv_path}")


if __name__ == "__main__":
    # Save alongside this script (MOCAP_Validation folder) regardless of
    # the directory the script is launched from. Each run gets its own
    # timestamped file so previous runs are never overwritten.
    _SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    _RUN_STAMP = datetime.now().strftime("%Y%m%d_%H%M%S")
    CSV_PATH = os.path.join(_SCRIPT_DIR, f"sync_marker_log_{_RUN_STAMP}.csv")
    SAMPLE_RATE_HZ = 100  # easy to change: samples per second
    SERIAL_PORT = "/dev/ttyACM0"
    SERIAL_BAUD = 115200

    logger = SyncMarkerLogger(
        csv_path=CSV_PATH,
        sample_rate_hz=SAMPLE_RATE_HZ,
        marker_hold_samples=10,
        serial_port=SERIAL_PORT,
        serial_baud=SERIAL_BAUD,
    )
    logger.run()
