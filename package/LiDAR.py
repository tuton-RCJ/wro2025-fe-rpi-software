import ydlidar
import serial
import numpy as np
from listen_to_lidar import listen_to_lidar

class lidar_: # いい命名が思いつかないのでとりあえず衝突を防ぐためこれで
    def __init__(self,port="/dev/ttyAMA4", baudrate=230400, lidar_type=ydlidar.TYPE_TRIANGLE, device_type=ydlidar.YDLIDAR_TYPE_SERIAL, scan_frequency=10.0, sample_rate=4, single_channel=False, max_angle=180.0, min_angle=-180.0, max_range=16.0, min_range=0.02, intensity=True,is_ld06=False):
        self._is_ld06 = is_ld06
        self.port = port
        if not self._is_ld06:
            self.lidar = ydlidar.CYdLidar()
            self.lidar.setlidaropt(ydlidar.LidarPropSerialPort, port)
            self.lidar.setlidaropt(ydlidar.LidarPropSerialBaudrate, baudrate)
            self.lidar.setlidaropt(ydlidar.LidarPropLidarType, lidar_type)
            self.lidar.setlidaropt(ydlidar.LidarPropDeviceType, device_type)
            self.lidar.setlidaropt(ydlidar.LidarPropScanFrequency, scan_frequency)
            self.lidar.setlidaropt(ydlidar.LidarPropSampleRate, sample_rate)
            self.lidar.setlidaropt(ydlidar.LidarPropSingleChannel, single_channel)
            self.lidar.setlidaropt(ydlidar.LidarPropMaxAngle, max_angle)
            self.lidar.setlidaropt(ydlidar.LidarPropMinAngle, min_angle)
            self.lidar.setlidaropt(ydlidar.LidarPropMaxRange, max_range)
            self.lidar.setlidaropt(ydlidar.LidarPropMinRange, min_range)
            self.lidar.setlidaropt(ydlidar.LidarPropIntenstiy, intensity)
            self.lidar.initialize()
        else:
            self.lidar_data = None
            self.stop = None
        self.points = None

    def turn_on(self):
        if self._is_ld06:
            self.lidar_data, self.stop = listen_to_lidar(self.port)
            import time
            time.sleep(1)
        else:
            self.lidar.turnOn()
    
    def turn_off(self):
        if self._is_ld06:
            self.stop()
        else:
            self.lidar.turnOff()

    def close_port(self):
        if self._is_ld06:
            return
        else:
            self.lidar.disconnecting()

    def scan(self):
        scan = ydlidar.LaserScan()
        self.lidar.doProcessSimple(scan)
        return scan.points
    
    def update(self):
        if self._is_ld06:
            pts = self.lidar_data['distances']
            angles = np.array([np.deg2rad(p) for p in pts.keys()], dtype=np.float32)
            ranges = np.array([p for p in pts.values()], dtype=np.float32)
            intensities = np.array([1], dtype=np.float32)
            self.points = np.stack((angles, ranges, intensities), axis=1)
        else:
            pts = self.scan()
            angles = np.array([-p.angle + np.pi for p in pts], dtype=np.float32)
            ranges = np.array([p.range for p in pts], dtype=np.float32)
            intensities = np.array([p.intensity for p in pts], dtype=np.float32)
            self.points = np.stack((angles, ranges, intensities), axis=1)
