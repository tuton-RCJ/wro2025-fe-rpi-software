#!/usr/bin/env python3
# viz_icp_fixed.py
import time, copy
import numpy as np
import matplotlib; matplotlib.use('agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import sys
sys.path.append('/home/tuton/wro2025-fe-rpi-software/package')
import math
from LiDAR import lidar_

port = "/dev/ttyUSB0";
lidar = lidar_(is_ld06=True, port=port)
ret = lidar.turn_on()
try:
    while True:
        lidar.update()
        pts = lidar.points
        pt = []
        for a, r, i in pts:
            pt.append([r * math.cos(a + np.pi), r * math.sin(a + np.pi)])
        intensities = np.array([i for _, _, i in pts], dtype=np.float32)
        pt = np.array(pt)
        plt.clf()
        ax = plt.subplot(111)
        ax.set_xlim(-100, 100)
        ax.set_ylim(-100, 100)

        ax.scatter(pt[:, 0], pt[:, 1], c=intensities, cmap='hsv', alpha=0.75)
        plt.savefig("lidar_scan.png")
        time.sleep(0.15)
except KeyboardInterrupt:
    pass
finally:
    lidar.turn_off()
    lidar.close_port()
