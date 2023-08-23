import math

import RPi.GPIO as GPIO
from time import time, sleep

from threading import Thread

from BaseClasses import Motor, Pose, Point
from Robot import Robot

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

GPIO.setmode(GPIO.BCM)

left_motor = Motor(motor_left_enable, motor_left_positive, motor_left_negative, motor_left_encoder_a, motor_left_encoder_b)
right_motor = Motor(motor_right_enable, motor_right_positive, motor_right_negative, motor_right_encoder_a, motor_right_encoder_b)

pose = Pose(0, 0,math.pi / 2)
robot = Robot(pose)

def loop():
    # Drive to (0.5, 0)
    coordinate1 = Point(0.5, 0)
    robot.drive_to_coordinate(coordinate1)

    # Drive to (0, 0.5)
    coordinate2 = Point(0, 0.5)
    robot.drive_to_coordinate(coordinate2)

    # Drive to (0.5, 0.5)
    coordinate3 = Point(0.5, 0.5)
    robot.drive_to_coordinate(coordinate3)

    # Drive to (0, 0)
    coordinate4 = Point(0, 0)
    robot.drive_to_coordinate(coordinate4, end_orientation=math.pi/2)

if __name__ == "__main__":
    loop()



