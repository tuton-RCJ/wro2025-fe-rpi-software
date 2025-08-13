import ydlidar

class ydlider:
    def __init__(self,port="/dev/ttyUSB0", baudrate=230400, lidar_type=ydlidar.TYPE_TRIANGLE, device_type=ydlidar.YDLIDAR_TYPE_SERIAL, scan_frequency=10.0, sample_rate=4, single_channel=False, max_angle=180.0, min_angle=-180.0, max_range=16.0, min_range=0.02, intensity=True):
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
        self.lidar.setlidaropt(ydlidar.LidarPropIntensity, intensity)
        self.lidar.initialize()

    def turn_on(self):
        return self.lidar.turnOn()
    
    def turn_off(self):
        self.lidar.turnOff()

    def close_port(self):
        self.lidar.disconnecting()

    def scan(self):
        scan = ydlidar.LaserScan()
        return scan