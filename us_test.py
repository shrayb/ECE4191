import math
import numpy as np
import FakeRPi.GPIO as GPIO
from time import time, sleep
from BaseClasses import Pose, Ultrasonic
from Robot import Robot

front_left_us_echo = 1
front_left_us_trig = 2

front_right_us_echo = 3
front_right_us_trig = 4

front_left_us = Ultrasonic(echo_pin= front_left_us_echo, trig_pin= front_left_us_trig)
front_right_us = Ultrasonic(echo_pin=front_right_us_echo, trig_pin=front_right_us_trig)

GPIO.setmode(GPIO.BCM)

pose = Pose(0.3,0.2,0)
robot = Robot(pose=pose)


def loop():
    robot.detect_obstacle(front_left_us, front_right_us)

if __name__ == "__main__":
    while (True):
        loop()
