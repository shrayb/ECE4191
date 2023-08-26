import math
import numpy as np

import RPi.GPIO as GPIO
from time import time, sleep

from threading import Thread

from BaseClasses import Motor, Pose, Pose, Ultrasonic
from Robot import Robot

MAP_SIZE = 1500
ROBOT_SIZE = 40
E_MAP_SIZE = MAP_SIZE - ROBOT_SIZE

motor_left_positive = 15
motor_left_negative = 18
motor_left_enable = 14
motor_left_encoder_a = 3
motor_left_encoder_b = 4

motor_right_positive = 17
motor_right_negative = 27
motor_right_enable = 22
motor_right_encoder_a = 23
motor_right_encoder_b = 24

front_left_us_echo = 1
front_left_us_trig = 2

front_right_us_echo = 3
front_right_us_trig = 4

rear_left_us_echo = 5
rear_left_us_trig = 6

rear_right_us_echo = 7
rear_right_us_trig = 8

front_left_us = Ultrasonic(echo_pin= front_left_us_echo, trig_pin= front_left_us_trig)
front_right_us = Ultrasonic(echo_pin=front_right_us_echo, trig_pin=front_right_us_trig)
rear_left_us = Ultrasonic(echo_pin=rear_left_us_echo, trig_pin=rear_left_us_trig)
rear_right_us = Ultrasonic(echo_pin=rear_right_us_echo, trig_pin=rear_right_us_trig)

GPIO.setmode(GPIO.BCM)

left_motor = Motor(motor_left_enable, motor_left_positive, motor_left_negative, motor_left_encoder_a, motor_left_encoder_b)
right_motor = Motor(motor_right_enable, motor_right_positive, motor_right_negative, motor_right_encoder_a, motor_right_encoder_b)

pose = Pose(0.3, 0.2, 0)
robot = Robot(pose)
robot.left_motor = left_motor
robot.right_motor = right_motor

encoder_thread = Thread(target=robot.encoder_update_loop)
encoder_thread.start()

def localise():
    front_left_dist = front_left_us.measure_dist()
    front_right_dist = front_right_us.measure_dist()
    rear_left_dist = rear_left_us.measure_dist()
    rear_right_dist = rear_right_us.measure_dist()

    return front_left_dist, front_right_dist, rear_left_dist, rear_right_dist

# Will take each sensor reading and also the current coordinate of the robot and check whether the measruements are valid.
def validate_measurements(front_left_dist = None, front_right_dist = None, rear_left_dist = None, rear_right_dist = None):
    ## TODO
    return None

def detect_obstacle(front_left_dist = None, front_right_dist = None, rear_left_dist = None, rear_right_dist = None, robot_pose = None):
    front_left_dist, front_right_dist, rear_left_dist, rear_right_dist = 0
    counter = 0
    while counter<3:
        # front_left_dist_check, front_right_dist_check, rear_left_dist_check, rear_right_dist_check = localise()
        # valid_check = validate_measurements()
        # while not valid_check:
        #     front_left_dist_check, front_right_dist_check, rear_left_dist_check, rear_right_dist_check = localise()
        # front_left_dist, front_right_dist, rear_left_dist, rear_right_dist += front_left_dist_check, front_right_dist_check, rear_left_dist_check, rear_right_dist_check
        front_left_dist, front_right_dist, rear_left_dist, rear_right_dist += localise()
        counter += 1

    front_left_dist = front_left_dist/counter
    front_right_dist = front_right_dist/counter
    rear_left_dist = rear_left_dist/counter
    rear_right_dist = rear_right_dist/counter

    ## Need to find effective map size based on the robot coordinates
    x, y, th = robot_pose

    if 0 <= th < np.pi/2:
        '''TO DO'''
        th_hor = th
    else:
        '''TO DO'''

    coords_x = E_MAP_SIZE - (front_left_dist*np.sin(robot_pose[2])+ROBOT_SIZE/2)
    coords_y = E_MAP_SIZE - (front_right_dist*np.cos(robot_pose[2]+ROBOT_SIZE/2))

    uncertainty_meas = 15
    if np.abs(coords_x-robot_pose[0])<uncertainty_meas and np.abs(coords_y-robot_pose[1])<uncertainty_meas:
        flag = False
    else:
        flag = True
    return flag


scanner_thread = Thread(target=detect_obstacle)
scanner_thread.start()

def loop():
    coords = [Pose(0.3, 0.2, math.pi / 6),
              Pose(0.3, 0.2, 2 * math.pi / 6),
              Pose(0.3, 0.2, 3 * math.pi / 6),
              Pose(0.3, 0.2, 4 * math.pi / 6),
              Pose(0.3, 0.2, 5 * math.pi / 6),
              Pose(0.3, 0.2, 6 * math.pi / 6)]

    for coord in coords:
        # Drive to each coord
        robot.pose = Pose(0.3, 0.2, 0)
        x = input("Press 1 to go again")
        if x == "1":
            robot.drive_to_coordinate(coord)

    sleep(10)
    x = input("hello")
    coords = [Pose(0.3, 0.2, math.pi)]

    for coord in coords:
        # Drive to each coord
        robot.drive_to_coordinate(coord)
        sleep(2)

if __name__ == "__main__":
    loop()



