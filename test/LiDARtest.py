import os
import ydlidar
import time
import numpy as np

if __name__ == "__main__":
    ydlidar.os_init();
    ports = ydlidar.lidarPortList();
    port = "/dev/ttyUSB0";
    for key, value in ports.items():
        port = value;
        print(port);
    laser = ydlidar.CYdLidar();
    laser.setlidaropt(ydlidar.LidarPropSerialPort, port);
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 230400);
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE);
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL);
    laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0);
    laser.setlidaropt(ydlidar.LidarPropSampleRate, 4);
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, False);
    laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0);
    laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0);
    laser.setlidaropt(ydlidar.LidarPropMaxRange, 16.0);
    laser.setlidaropt(ydlidar.LidarPropMinRange, 0.02);
    laser.setlidaropt(ydlidar.LidarPropIntenstiy, True);

    ret = laser.initialize();
    if ret:
        ret = laser.turnOn();
        scan = ydlidar.LaserScan();
        
        while ret and ydlidar.os_isOk() :
            r = laser.doProcessSimple(scan);
            pts = scan.points
            n   = pts.size()                 # 点の数

            # 各フィールドを抽出して numpy の 1D 配列を作成
            angles      = np.array([p.angle     for p in pts], dtype=np.float32)  # 角度
            ranges      = np.array([p.range     for p in pts], dtype=np.float32)  # 距離[m]
            intensities = np.array([p.intensity for p in pts], dtype=np.float32)  # 反射強度（ない機種もあり）

            # 3 列 (angle, range, intensity) の 2D 配列にまとめる
            data = np.stack((angles, ranges, intensities), axis=1)  # shape == (n, 3)

            print(data.shape)     # => (n, 3)
            print(data[:5])       # 先頭 5 点を表示
            
            if r:
                print("Scan received [", scan.points.size(), "] points");
            else :
                print("Failed to get lidar data")
                time.sleep(0.05);
        laser.turnOff();
    laser.disconnecting();
    
