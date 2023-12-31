import math

import RPi.GPIO as GPIO
from time import time, sleep

from threading import Thread

from BaseClasses import Motor, Pose, Pose, Ultrasonic
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

pose = Pose(0.3, 0.2, 0)
robot = Robot(pose)
robot.left_motor = left_motor
robot.right_motor = right_motor

encoder_thread = Thread(target=robot.encoder_process)
encoder_thread.start()


def loop():
    coords = [Pose(0.9, 0.8),
              Pose(0.4, 0.3, math.pi),
              Pose(0.3, 0.8),
              Pose(0.3, 0.1, math.pi / 6),
              Pose(0.7, 0.9, (3* math.pi) / 4),
              Pose(0.3, 0.2, 0)]

    for coord in coords:
        # Drive to each coord
        robot.drive_to_coordinate(coord)

if __name__ == "__main__":
    loop()



