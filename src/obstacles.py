import sys
import time
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
turn_cnt = 0
target_dist = [0.2, 0.5, 0.2]
thres_forward_dist = 0.6
cut_line = [0.75,1.25,1.75,2.25]

objects = [[-1 for j in range(2)] for i in range(4)] #  1: There are NO OBSTACLES 2: red, 0: green 


def get_index(x):
    if cut_line[0] < x < cut_line[0] + 0.2:
        return 0
    elif cut_line[1] < x < cut_line[1] + 0.2:

        return 1
    elif cut_line[2] < x < cut_line[2] + 0.2:
        return 2
    else:
        return -1
    
def get_index_strict(x):
    x -= 0.25
    if cut_line[0] < x <= cut_line[1]:
        return 0
    if cut_line[1] < x <= cut_line[2]:
        return 1
    if cut_line[2] < x <= cut_line[3]:
        return 2
    if cut_line[3] < x:
        return 3
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

def get_abs_dist():
    return 3-get_dist(0)

def get_obj_angle(x): #this function returns the angle as degree(float)
    l = image_width/2 - x
    return fov * l / image_width

def get_obj_dist(x):
    angle = get_obj_angle(x)
    return get_dist(angle)

def get_side_dist(): # first is left, second is right
    return [get_dist(90), get_dist(-90)]

def decide_clockwise():
    direct = 1 if get_side_dist()[0] > get_side_dist()[1] else -1
    return direct

def forward_to_specified_dist(dist):
    sts.drive()
    while get_dist(0) > dist:
        lidar.update()
    sts.stop()

def turn_corner():
    forward_to_specified_dist(0.6)
    if direct == 1:
        sts.turn_right()
    else:
        sts.turn_left()

def escape_from_parking():
    decide_clockwise()
    if direct == 1:
        sts.turn_right()
        time.sleep(0.1)
        sts.turn_left()
    else:
        sts.turn_left()
        time.sleep(0.1)
        sts.turn_right()

def enter_to_parking():
    if direct == 1:
        sts.turn_left()
        time.sleep(0.1)
        sts.turn_right()
    else:
        sts.turn_right()
        time.sleep(0.1)
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
        self._target_lane = target_lane
        self._Imax = 100

    def update(self):
        current_distance = get_dist(direct*(90 if direct == 0 else -90))

        error = self._target_distance - current_distance
        
        p = self._kp * error
        self._integral += error
        i = self._ki * self._integral
        i = min(max(i, -self._Imax), self._Imax)
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

if __name__ == "__main__":
    escape_from_parking()
    pid = PID_towall()
    now_index = 0
    now_color = -1
    turn_cnt = 0
    while turn_cnt < 4:
        lidar.update()
        red, green = uv.update_data()
        pid.update()
        x = get_abs_dist()
        if not uv.is_empty():
            red_dist = get_obj_dist(red) if red != 255 else float('inf')
            green_dist = get_obj_dist(green) if green != 255 else float('inf')
            red_dist += x
            green_dist += x
            if red_dist != float('inf'):
                if get_index(red_dist) != -1:
                    objects[turn_cnt][get_index(red_dist)] = 2
            if green_dist != float('inf'):
                if get_index(green_dist) != -1:
                    objects[turn_cnt][get_index(green_dist)] = 0
        now_index = get_index_strict(x)
        if now_index == 3:
            turn_corner()
            turn_cnt += 1
        else:
            if pid._target_lane != objects[turn_cnt][now_index]:
                pid.switch_lane(objects[turn_cnt][now_index])
    while turn_cnt < 12:  
        lidar.update()
        pid.update()      
        now_index = get_index_strict(x)
        if now_index == 3:
            turn_corner()
            turn_cnt += 1
        else:
            if pid._target_lane != objects[turn_cnt][now_index]:
                pid.switch_lane(objects[turn_cnt][now_index])

    while get_dist(180) < 1.2:
        lidar.update()
        red, green = uv.update_data()
        pid.update()
        pid.switch_lane(2)
    sts.stop()
    enter_to_parking()
