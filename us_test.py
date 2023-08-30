import RPi.GPIO as GPIO
from BaseClasses import *
from Robot import Robot
import time

front_left_us_echo = 26
front_left_us_trig = 5

front_right_us_echo = 6
front_right_us_trig = 13

front_left_us = Ultrasonic(echo_pin= front_left_us_echo, trig_pin= front_left_us_trig)
front_right_us = Ultrasonic(echo_pin=front_right_us_echo, trig_pin=front_right_us_trig)



pose = Pose(0.3,0.2,0)
robot = Robot(pose=pose)


def loop():
    flag, left, right, th = robot.detect_obstacle(front_left_us, front_right_us)

if __name__ == "__main__":
    while (True):
        loop()
        time.sleep(1)
