import sys
import time
import numpy as np
sys.path.append('/home/tuton/wro2025-fe-rpi-software/package')
from UnitV import UnitV
from STS3032 import sts3032
from LiDAR import lidar_
import random

uv = UnitV()

while True:
    print(uv.update_data())