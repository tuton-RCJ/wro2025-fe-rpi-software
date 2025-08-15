import sys
import time
import numpy as np
sys.path.append('/home/tuton/wro2025-fe-rpi-software/package')
from UnitV import UnitV
from STS3032 import sts3032
from LiDAR import lidar_
import random

sts = sts3032()
uv = UnitV()
lidar = lidar_()
lidar.turn_on()
image_width = 160
fov = 60 # degrees
direct = 1 # 1 if clockwise else -1
turn_cnt = 0
target_dist = [0.2, 0.5, 0.2]
thres_forward_dist = 0.6
cut_line = [0.75,1.25,1.75,2.55]

objects = [[-1 for j in range(3)] for i in range(4)] #  -1: there are no obstacles, 2: red , 0: green 

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
    x += 0.25
    if x <= cut_line[1]:
        return 0
    if cut_line[1] < x <= cut_line[2]:
        return 1
    if cut_line[2] < x <= cut_line[3]:
        return 2
    if cut_line[3] < x:
        return 3
def get_dist(angle): # this function is expected to use after executing lidar.update()
    rad_a = np.deg2rad(angle)
    rad_a %= np.pi * 2
    min_abs = float('inf')
    min_dist = -1
    min_index = -1
    for i,d in enumerate(lidar.points):
        if abs(d[0]%(np.pi*2) - rad_a) < min_abs:
            min_dist = d[1]
            min_abs = abs(d[0] - rad_a)
            min_index = i
    
    return min_dist

def polartoXY(angle,dist):
    x = dist * np.cos(angle)
    y = dist * np.sin(angle)
    return x, y

def get_abs_dist():
    return 3-get_dist(0) if get_dist(0) > 0.6 else -1

def get_obj_angle(x): #this function returns the angle as degree(float)
    l = x-image_width/2
    return fov * l / image_width

def get_minpoint(low,high):
    low = np.deg2rad(low)
    high = np.deg2rad(high)
    min_dist = float('inf')
    for i in range(len(lidar.points)):
        if low < lidar.points[i][0] < high:
            min_dist = min(min_dist, lidar.points[i][1])
    return min_dist

def get_obj_dist(x):
    angle = get_obj_angle(x)
    return get_minpoint(x-10,x+10) * np.cos(angle)

def get_wall_line_distance(target_angle, angle_range=15):
    
    target_rad = np.deg2rad(target_angle)
    range_rad = np.deg2rad(angle_range)

    nearby_points = []
    for point in lidar.points:
        angle_diff = abs((point[0] % (2 * np.pi)) - (target_rad % (2 * np.pi)))
        angle_diff = min(angle_diff, 2 * np.pi - angle_diff) 
        
        if angle_diff <= range_rad:
            x = point[1] * np.cos(point[0])
            y = point[1] * np.sin(point[0])
            nearby_points.append((x, y))
    
    if len(nearby_points) < 2:
        return get_dist(target_angle)
    
    nearby_points = np.array(nearby_points)
    x_coords = nearby_points[:, 0]
    y_coords = nearby_points[:, 1]
    
    if abs(target_angle) > 45 and abs(target_angle) < 135:
        if len(set(y_coords)) > 1: 
            A = np.vstack([y_coords, np.ones(len(y_coords))]).T
            a, b = np.linalg.lstsq(A, x_coords, rcond=None)[0]
            distance = abs(b) / np.sqrt(a**2 + 1)
        else:
            distance = abs(np.mean(x_coords))
    else:
        if len(set(x_coords)) > 1:  
            A = np.vstack([x_coords, np.ones(len(x_coords))]).T
            a, b = np.linalg.lstsq(A, y_coords, rcond=None)[0]
            
            distance = abs(b) / np.sqrt(a**2 + 1)
        else:
            distance = abs(np.mean(y_coords))
    
    return distance

def get_side_dist(): # first is left, second is right
    left_dist = get_wall_line_distance(90)   
    right_dist = get_wall_line_distance(-90) 
    return [left_dist, right_dist]

def decide_clockwise():
    global direct
    direct = 1 if get_side_dist()[0] > get_side_dist()[1] else -1
    return direct

def forward_to_specified_dist(dist):
    sts.drive()
    while get_dist(0) > dist:
        lidar.update()
    sts.stop()

def escape_from_parking():
    decide_clockwise()
    if direct == 1:
        sts.turn_right()
        sts.drive()
        time.sleep(0.7)
        sts.turn_left()
    else:
        sts.turn_left()
        sts.drive()
        time.sleep(0.7)
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

def estimate_wall_angle(is_left=True):  # return wall angle (°) [0~360)
    lidar_data = []
    calc_range = 20  # range of angles to consider(half)
    
    if is_left:
        # Left wall (90° ± calc_range)
        for i in range(90, 90 + calc_range + 1, 10):
            lidar_data.append((i - 90, get_dist(i)))
        for i in range(90 - calc_range, 90, 10):
            lidar_data.append((i + 270, get_dist(i)))
    else:
        # Right wall (270° ± calc_range)
        for i in range(270, 270 + calc_range + 1, 10):
            angle_normalized = (i - 270) if i < 360 else (i - 270 - 360)
            lidar_data.append((angle_normalized, get_dist(i % 360)))
        for i in range(270 - calc_range, 270, 10):
            angle_normalized = (i - 270) if i >= 0 else (i + 90)
            lidar_data.append((angle_normalized, get_dist(i % 360)))

    data = np.array(lidar_data, dtype=float)
    angles = np.deg2rad(data[:, 0])
    dist = data[:, 1]

    # convert to cartesian coordinates
    x = dist * np.cos(angles)
    y = dist * np.sin(angles)

    x, y = y, x  # swap x and y to avoid m goes inf

    # Least squares y = m*x + c
    A = np.vstack([x, np.ones(len(x))]).T
    m, _ = np.linalg.lstsq(A, y, rcond=None)[0]

    # calc wall angle
    line_angle = np.rad2deg(np.arctan(m))
    wall_angle = -line_angle if is_left else line_angle  # adjust sign based on wall side
    return wall_angle

def turn_corner():
    curve_angle = estimate_wall_angle()*direct
    # forward_to_specified_dist(0.6)
    sts.drive(speed=90, degree=65*direct)
    time.sleep(1.5*((90+curve_angle)/90))  # need to adjust
    print(90+curve_angle)
    sts.stop()

class PID:
    def __init__(self, target=0, target_lane=1):
        self._kp = 2
        self._ki = 0
        self._kd = 0.1
        self._k = 1
        self._old_error = 0
        self._integral = 0
        self._Imax = 100
        self._target = target
        self._target_lane = target_lane

    def update(self,current):
        error = self._target - current
        p = self._kp * error
        self._integral += error
        i = self._ki * self._integral
        i = min(max(i, -self._Imax), self._Imax)
        d = self._kd * (error - self._old_error)
        pid = p + i + d
        self._old_error = error
        sts.drive(speed=50, degree=pid*self._k)

    def reset(self):
        self._old_error = 0
        self._integral = 0
    
    def switch_lane(self, target_lane):
        if self._target_lane != target_lane:
            self._target_lane = target_lane
            self.reset()

pid = PID()

def drive_straight(il):
    pid.update(estimate_wall_angle(is_left=il))

def switch_lane(target_lane):
    old_lane = pid._target_lane
    if old_lane == target_lane:
        return
    if old_lane == 1:
        if target_lane == 0:
            sts.turn_right()
            sts.turn_left()
            t
        elif target_lane == 2:
            sts.turn_left()
            sts.turn_right()
    elif old_lane == 2:
        sts.turn_right()
        sts.drive()
        time.sleep(1.5)
        sts.turn_left()
    elif old_lane == 0:
        sts.turn_left()
        sts.drive()
        time.sleep(1.5)
        sts.turn_right()        
    sts.drive()
    time.sleep(1.5)
    pid._target_lane = target_lane


if __name__ == "__main__":
    try:
        lidar.update()
        escape_from_parking()
        now_index = 0
        turn_cnt = 0
        x_old = 1.5
        while turn_cnt < 5:
            lidar.update()
            drive_straight(True)
            red, green = uv.update_data()
            x = get_abs_dist()
            if x == -1:
                x = x_old
            red_dist = get_obj_dist(red) if red != 255 else float('inf')
            green_dist = get_obj_dist(green) if green != 255 else float('inf')
            red_dist += x
            green_dist += x
            if red_dist != float('inf'):
                if get_index(red_dist) != -1:
                    objects[turn_cnt%4][get_index(red_dist)] = 2
            if green_dist != float('inf'):
                if get_index(green_dist) != -1:
                    objects[turn_cnt%4][get_index(green_dist)] = 0
            now_index = get_index_strict(x)
            if (now_index == 3) and (get_dist(90*(-direct)) > 0):
                turn_corner()
                now_index = 0
                turn_cnt += 1
                pid._target_lane = 1
            if pid._target_lane != objects[turn_cnt%4][now_index] and objects[turn_cnt%4][now_index] != -1:
                switch_lane(objects[turn_cnt%4][now_index])
            print(f"debug: red:{red_dist}, green:{green_dist} now_index:{now_index}, target_lane:{pid._target_lane}, object:{objects[turn_cnt%4]}")
            print(f"debug: dist:{[get_dist(0), get_dist(90), get_dist(180), get_dist(270)]}, turn_cnt: {turn_cnt}")
            x_old = x
        """
            while turn_cnt < 12:  
                lidar.update()
                pid.update()      
                x = get_abs_dist()
                now_index = get_index_strict(x)
                if now_index == 3:
                    turn_corner()
                    turn_cnt += 1
                    now_index = 1
                    pid._target_lane = 1
                else:
                    if pid._target_lane != objects[turn_cnt%4][now_index]:
                        switch_lane(objects[turn_cnt%4][now_index])

            while get_dist(180) < 1.2:
                lidar.update()
                red, green = uv.update_data()
                pid.update()
                pid.switch_lane(2)
            sts.stop()
            enter_to_parking()
        """
    except KeyboardInterrupt:
        sts.stop()
    finally:
        pass