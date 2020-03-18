import socket
import time
import struct
import numpy as np

LASER_ANGLES = [-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15]
NUM_LASERS = 16

EXPECTED_PACKET_TIME = 0.001327  # valid only in "the strongest return mode"
EXPECTED_SCAN_DURATION = 0.1
DISTANCE_RESOLUTION = 0.002
ROTATION_RESOLUTION = 0.01
ROTATION_MAX_UNITS = 36000


class GetData():
    def __init__(self):
        self.HOST = "192.168.1.201"
        self.PORT = 2369
        self.data = []
        self.POINT = []

    def capture(self):
        soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        soc.bind(('', self.PORT))
        try:
            while True:
                try:
                    data = soc.recv(1248)
                    if len(data) > 0:
                        assert len(data) == 1206, len(data)
                        self.parsing(data)
                except Exception as e:
                    print(dir(e), e.__class__.__name__)              
        except KeyboardInterrupt as e:
            print(e)
    
    def parsing(self, data):
        ts = time.time()
        timestamp, factory = struct.unpack_from("<IH", data, offset=1200)
        assert factory == 0x2237, hex(factory)  # 0x22=VLP-16, 0x37=Strongest Return
        timestamp = float(ts)
        seq_index = 0
        self.POINT = []
        for offset in range(0, 1200, 100):
            flag, azimuth = struct.unpack_from("<HH", data, offset)
            assert flag == 0xEEFF, hex(flag)
            for step in range(2):
                seq_index += 1
                azimuth += step
                azimuth %= ROTATION_MAX_UNITS
                # H-distance (2mm step), B-reflectivity (0
                arr = struct.unpack_from('<' + "HB" * 16, data, offset + 4 + step * 48)
                for i in range(NUM_LASERS):
                    time_offset = (55.296 * seq_index + 2.304 * i) / 1000000.0
                    if arr[i * 2] != 0:
                        self.POINT.append(self.calc(arr[i * 2], azimuth, i, timestamp + time_offset))

    def calc(self, dis, azimuth, laser_id, timestamp):
        R = dis * DISTANCE_RESOLUTION
        omega = LASER_ANGLES[laser_id] * np.pi / 180.0
        alpha = azimuth / 100.0 * np.pi / 180.0
        X = R * np.cos(omega) * np.sin(alpha)
        Y = R * np.cos(omega) * np.cos(alpha)
        Z = R * np.sin(omega)
        return [X, Y, Z, timestamp]