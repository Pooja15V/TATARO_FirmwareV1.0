"""
Raw SyncPulse diagnostic tool.

Prints every valid frame received over serial exactly as-is (timestamp,
raw payload bytes) plus a running frame rate, with zero interpretation of
"recording state." Use this to see the ground truth of what the device
is actually sending:

    1. Run this with mocap recording OFF - note whether anything prints.
    2. Turn recording ON - note what changes (new frames? different rate?).
    3. Turn recording OFF again - note whether frames actually stop.

Run:
    python3 syncpulse_raw_debug.py
Stop:
    Ctrl+C
"""

import time
from datetime import datetime

import serial

SERIAL_PORT = "/dev/ttyACM0"
SERIAL_BAUD = 115200


def read_one_frame(ser):
    """
    Blocking read of one frame using the same protocol as
    syncpulse_reader.py's SerialPort.serial_read():

        | 0xFF | 0xFF | payload_size | payload | checksum |

    Returns the payload bytes on a valid frame, otherwise None (some
    bytes were consumed while searching for the next header).
    """
    if ser.read(1) == b"\xff" and ser.read(1) == b"\xff":
        chksum = 255 + 255
        payload_size = ser.read(1)[0]
        chksum += payload_size
        payload = ser.read(payload_size - 1)
        chksum += sum(payload)
        chksum = bytes([chksum % 256])
        _chksum = ser.read(1)
        if _chksum == chksum:
            return payload
    return None


def main():
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD)
    print(f"Listening on {SERIAL_PORT} @ {SERIAL_BAUD} baud. Press Ctrl+C to stop.")
    print("Toggle mocap recording on/off while this runs and watch for changes.\n")

    frame_count = 0
    window_start = time.monotonic()

    try:
        while True:
            payload = read_one_frame(ser)
            if payload is None:
                continue  # misaligned byte(s); protocol resyncs on the next header

            frame_count += 1
            now = time.monotonic()
            elapsed = now - window_start
            if elapsed >= 1.0:
                print(f"--- {frame_count} frames in the last {elapsed:.2f}s "
                      f"(~{frame_count / elapsed:.1f} Hz) ---")
                frame_count = 0
                window_start = now

            ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            print(f"{ts}  raw_payload={payload!r}  hex={payload.hex()}")

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
