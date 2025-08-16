import sys
import time
import numpy as np

sys.path.append('/home/tuton/wro2025-fe-rpi-software/package')
from LiDAR import lidar_
from STS3032 import sts3032
from UnitV import UnitV


sts = sts3032()
lidar = lidar_()
lidar.turn_on()


direct = 0  # 1 if clockwise else -1
turn_cnt = 0
target_dist = [0.2, 0.5, 0.2]
thres_forward_dist = 0.6


def get_dist(angle): # this function is expected to use after executing lidar.update()
    rad_a = np.deg2rad(angle)
    # print(rad_a)
    rad_a %= np.pi * 2
    # print(rad_a)
    min_abs = float('inf')
    min_dist = -1
    min_index = -1
    for i,d in enumerate(lidar.points):
        if abs(d[0]%(np.pi*2) - rad_a) < min_abs:
            min_dist = d[1]
            min_abs = abs(d[0] - rad_a)
            min_index = i
    # print(i, lidar.points[i])
    return min_dist


def get_side_dist():  # first is left, second is right
    return [get_dist(90), get_dist(-90)]


def decide_clockwise():  # decide turn direction based on side distances
    global direct
    side_dists = get_side_dist()
    direct = 1 if side_dists[0] < side_dists[1] else -1
    return direct


# the robot will move forward untill the front distance achieve given dist
def forward_to_specified_dist(dist: float):
    lidar.update()
    while get_dist(0) > dist:
        lidar.update()
        drive_straight()
    sts.stop()


def estimate_wall_angle():  # return wall angle (°) [0~360)
    lidar_data = []
    calc_range = 10  # range of angles to consider(half)
    for i in range(0, calc_range+1,10):
        lidar_data.append((i, get_dist(i)))
    for i in range(3600-calc_range, 360,10):
        lidar_data.append((i, get_dist(i)))

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
    wall_angle = line_angle  # need to adjust?
    return wall_angle


def estimate_left_wall_angle():  # return wall angle (°) [0~360)
    lidar_data = []
    calc_range = 20  # range of angles to consider(half)
    for i in range(90, 90 + calc_range+1,10):
        lidar_data.append((i-90, get_dist(i)))
    for i in range(90-calc_range, 90,10):
        lidar_data.append((i+270, get_dist(i)))

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
    wall_angle = -line_angle  # need to adjust?
    return wall_angle



def turn_corner():
    curve_angle = estimate_wall_angle()*direct
    # forward_to_specified_dist(0.6)
    sts.drive(speed=90, degree=65*direct)
    time.sleep(0.5*((90+curve_angle)/90))  # need to adjust
    print(90+curve_angle)
    sts.stop()


class PID:
    def __init__(self, target=0):
        self._kp = 2
        self._ki = 0
        self._kd = 0.1
        self._k = 1
        self._old_error = 0
        self._integral = 0
        self._Imax = 100
        self._target = target

    def update(self,current):
        
        error = self._target - current

        p = self._kp * error

        self._integral += error
        i = self._ki * self._integral
        i = min(max(i, -self._Imax), self._Imax)

        d = self._kd * (error - self._old_error)

        pid = p + i + d
        self._old_error = error
        sts.drive(speed=90, degree=pid*self._k)

    def reset(self):
        self._old_error = 0
        self._integral = 0


def drive_straight():
    pid.update(estimate_left_wall_angle())
    # sts.drive(speed=80)

pid=PID()



if __name__ == "__main__":
    
    try:
        sts.stop()

        # while True:
        #     lidar.update()
        #     print(estimate_wall_angle())
        #     time.sleep(0.1)
        
        lidar.update()
        # decide_clockwise()
        # turn_corner()
        # time.sleep(1)
        turn_cnt = 0
        while turn_cnt < 12:
            lidar.update()
            drive_straight()
            if get_dist(0) < 0.6:
                decide_clockwise()
                turn_corner()
                turn_cnt += 1
        sts.stop()
        time.sleep(0.2)
        forward_to_specified_dist(1.5)
        lidar.turn_off()
        sts.stop()
    except KeyboardInterrupt:
        print("Ctrl+C pressed. Programm Stopped.")
    finally:
        lidar.turn_off()
        sts.stop()
