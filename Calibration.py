import math

import RPi.GPIO as GPIO
from time import time, sleep

from threading import Thread

from BaseClasses import *
from Robot import Robot

motor_right_positive = 18
motor_right_negative = 15
motor_right_enable = 14
motor_right_encoder_a = 3
motor_right_encoder_b = 4

motor_left_positive = 27
motor_left_negative = 17
motor_left_enable = 22
motor_left_encoder_a = 23
motor_left_encoder_b = 24

front_left_sonic_echo = 26
front_left_sonic_trig = 5

front_right_sonic_echo = 6
front_right_sonic_trig = 13

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

left_motor = Motor(motor_left_enable, motor_left_positive, motor_left_negative, motor_left_encoder_a, motor_left_encoder_b)
right_motor = Motor(motor_right_enable, motor_right_positive, motor_right_negative, motor_right_encoder_a, motor_right_encoder_b)

front_left_sonic = Ultrasonic(echo_pin=front_left_sonic_echo, trig_pin=front_left_sonic_trig, x_offset=0.155, y_offset=0.0585, theta=0, reading_index=0, maximum_read_distance=0.25)
front_right_sonic = Ultrasonic(echo_pin=front_right_sonic_echo, trig_pin=front_right_sonic_trig, x_offset=0.155, y_offset=-0.0585, theta=0, reading_index=1, maximum_read_distance=0.25)

limit_switch = LimitSwitch(distance=0.15486+0.03617)

pose = Pose(0.5, 0.5, 0)
robot = Robot(pose)
robot.left_motor = left_motor
robot.right_motor = right_motor
robot.front_left_ultrasonic = front_left_sonic
robot.front_right_ultrasonic = front_right_sonic
robot.limit_switch = limit_switch


encoder_thread = Thread(target=robot.encoder_process)
encoder_thread.start()

ultrasonic_thread = Thread(target=robot.ultrasonic_thread)
ultrasonic_thread.start()

drive_thread = Thread(target=robot.drive_thread)
drive_thread.start()


def loop():
    try:
        # Loop and travel to each waypoint
        robot.do_drive(0.75)
        while True:
            sleep(1)
            print("Total ticks:", (robot.left_motor.ticks + robot.right_motor.ticks) / 2)

    except KeyboardInterrupt:
        robot.left_motor.speed = 0
        robot.right_motor.speed = 0
        robot.left_motor.stop()
        robot.right_motor.stop()



if __name__ == "__main__":
    loop()

