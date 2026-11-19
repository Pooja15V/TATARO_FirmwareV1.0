import serial
import struct
import keyboard
import csv
from sys import stdout
from datetime import datetime
import os
import cv2
# _filepath = os.path.dirname(os.path.abspath(__file__))


class SerialPort(object):

    def __init__(self, serialport = '/dev/ttyACM0', serialrate=115200,csv_path ="", csv_enable=False, single_file_protocol=False, dof=9, csv_name=None):
        # Initialise serial payload
        self.count = 0
        self.plSz = 0
        self.payload = bytearray()
        self.serialport = serialport
        self.ser_port = serial.Serial(serialport, serialrate)
        self.dof = dof



        self.csv_enabled = csv_enable
        if csv_enable:
            if csv_name is None:
                # self.csv_file = open(csv_path+ "//mocap_sync_data.csv", "w")
                self.csv_file = open(os.path.join(csv_path+ "//mocap_sync_data.csv"), "w")
                print(csv_path)
            else:
                self.csv_file = open(csv_path+ "//" + str(csv_name) + ".csv", "w")

            self.csv = csv.writer(self.csv_file)
            if self.dof == 6:
                self.csv.writerow(["sys_time","sync"])
        self.triggered = True
        self.connected = False


        stdout.write("Initializing\n")

    def serial_write(self, payload):
        # Format:
        # | 255 | 255 | no. of bytes | payload | checksum |

        header = [255, 255]
        chksum = 254
        payload_size = len(payload) + 1
        chksum += payload_size

        self.ser_port.write(bytes([header[0]]))
        self.ser_port.write(bytes([header[1]]))
        self.ser_port.write(bytes([payload_size]))
        self.ser_port.write(bytes([payload]))
        self.ser_port.write(bytes([chksum % 256]))

    def serial_read(self):
        """returns bool for valid read, also returns the data read"""

        if (self.ser_port.read() == b'\xff') and (self.ser_port.read() == b'\xff'):
            self.connected = True
            chksum = 255 + 255
            self.plSz = self.ser_port.read()[0]
            chksum += self.plSz
            self.payload = self.ser_port.read(self.plSz - 1)
            chksum += sum(self.payload)
            chksum = bytes([chksum % 256])
            _chksum = self.ser_port.read()
            return _chksum == chksum

        return False

    def disconnect(self):
        stdout.write("disconnected\n")

    def run_program(self):
        try:
            while True:
                if self.serial_read():
                    _sync = struct.unpack("c", self.payload)[0].decode("utf-8")
                    # nw = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
                    nw = datetime.now().isoformat()
                    print(_sync)
                    # sys.stdout.write('\r'+ _sync)
                    # sys.stdout.flush()
                    if self.csv_enabled:
                        if self.dof == 6:
                            self.csv.writerow([nw, _sync])
                            self.csv_file.flush()
        except KeyboardInterrupt:
            pass
        finally:
            self.csv_file.close()


if __name__ == '__main__':
    _filepath = r"/home/pooja/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0/xarm_ros2/MOCAP_Validation"
    # _filepath = os.path.dirname(os.path.abspath(__file__))

    # myport = SerialPort("/dev/ttyACM0", 115200, csv_path=_filepath, csv_enable=True, dof=6)
    myport = SerialPort("/dev/ttyACM0", 115200, csv_path=_filepath, csv_enable=True, dof=6)

    myport.run_program()
