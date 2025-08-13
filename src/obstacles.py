import sys
import numpy as np
sys.path.append('/home/tuton/wro2025-fe-rpi-software/package')
from UnitV import UnitV
from STS3032 import sts3032
from LiDAR import lidar_

sts = sts3032()
uv = UnitV()
lidar = lidar_()
lidar.turn_on()
image_width = 160
fov = 60 # degrees
direct = 0 # 1 if clockwise else -1
target_dist = [20, 50, 80]

objects = [[-1 for j in range(3)] for i in range(4)] # -1: not yet explored, 0: There are NO OBSTACLES 1: red, 2: green 

def get_dist(angle): # this function is expected to use after executing lidar.update()
    rad_a = np.deg2rad(angle)
    min_abs = float('inf')
    min_dist = -1
    for d in lidar.points:
        if abs(d[0] - rad_a) < min_abs:
            min_dist = d[1]
            min_abs = abs(d[0] - rad_a)
    return min_dist

def polartoXY(angle,dist):
    x = dist * np.cos(angle)
    y = dist * np.sin(angle)
    return x, y

def get_obj_angle(x): #this function returns the angle as degree(float)
    l = image_width/2 - x
    return fov * l / image_width

def get_obj_dist(x):
    angle = get_obj_angle(x)
    return get_dist(angle)

def get_side_dist(): # first is left, second is right
    return [get_dist(90), get_dist(-90)]

def decide_clockwise():
    return 1 if get_side_dist()[0] > get_side_dist()[1] else -1

def forward_to_specified_dist(dist):
    sts3032.drive()
    while get_dist(0) > dist:
        lidar.update()

def turn_corner():
    forward_to_specified_dist(0.6)
    if direct == 1:
        sts.turn_right()
    else:
        sts.turn_left()

class PID_towall:
    def __init__(self, target_lane=1):
        self._kp = 2
        self._ki = 5
        self._kd = 0.1
        self._k = 1
        self._old_error = 0
        self._integral = 0
        self._target_distance = target_dist[target_lane]
    
    def update(self):
        current_distance = get_dist(direct*90)
        
        error = self._target_distance - current_distance
        
        p = self._kp * error
        self._integral += error
        i = self._ki * self._integral
        d = self._kd * (error - self._old_error) 
        pid = p + i + d
        self._old_error = error
        sts.drive(degree=pid*self._k)
    
    def reset(self):
        self._old_error = 0
        self._integral = 0
    
    def switch_lane(self, target_lane):
        self._target_distance = target_dist[target_lane]
        self.reset()

