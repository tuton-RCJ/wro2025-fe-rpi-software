#!/usr/bin/env python3
# viz_icp_fixed.py
import time, copy
import numpy as np
import matplotlib; matplotlib.use('agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import  ydlidar
import math


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
    try:
        while True:
            r = laser.doProcessSimple(scan)
            if r:
                print("Scan received [", scan.points.size(), "] points");
            else :
                print("Failed to get lidar data")
            pts = scan.points
            angles      = np.array([p.angle     for p in pts], dtype=np.float32)  # 角度
            ranges      = np.array([p.range     for p in pts], dtype=np.float32)  # 距離[m]
            intensities = np.array([p.intensity for p in pts], dtype=np.float32)  # 反射強度（ない機種もあり）
            pt = []
            for a, r, i in zip(angles, ranges, intensities):
                pt.append([r * math.cos(-a+np.pi), r * math.sin(-a+np.pi)])
            pt = np.array(pt)
            plt.clf()
            ax = plt.subplot(111)
            ax.scatter(pt[:, 0], pt[:, 1], c=intensities, cmap='hsv', alpha=0.75)
            plt.savefig("lidar_scan.png")
            time.sleep(0.05);
    except KeyboardInterrupt:
        pass
    finally:
        laser.turnOff();
        laser.disconnecting();
