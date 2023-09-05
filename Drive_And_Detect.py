import math

import RPi.GPIO as GPIO
from time import time, sleep

from threading import Thread

from BaseClasses import Motor, Pose, Pose, Ultrasonic
from Robot import Robot

motor_left_positive = 18
motor_left_negative = 15
motor_left_enable = 14
motor_left_encoder_a = 3
motor_left_encoder_b = 4

motor_right_positive = 27
motor_right_negative = 17
motor_right_enable = 22
motor_right_encoder_a = 23
motor_right_encoder_b = 24

front_left_sonic_echo = 26
front_left_sonic_trig = 5

front_right_sonic_echo = 6
front_right_sonic_trig = 13

GPIO.setmode(GPIO.BCM)

left_motor = Motor(motor_left_enable, motor_left_positive, motor_left_negative, motor_left_encoder_a, motor_left_encoder_b)
right_motor = Motor(motor_right_enable, motor_right_positive, motor_right_negative, motor_right_encoder_a, motor_right_encoder_b)

front_left_sonic = Ultrasonic(echo_pin=front_left_sonic_echo, trig_pin=front_left_sonic_trig)
front_right_sonic = Ultrasonic(echo_pin=front_right_sonic_echo, trig_pin=front_right_sonic_trig)

pose = Pose(0.0, 0.0, 0)
robot = Robot(pose)
robot.left_motor = left_motor
robot.right_motor = right_motor
robot.front_left_ultrasonic = front_left_sonic
robot.front_right_ultrasonic = front_right_sonic

encoder_thread = Thread(target=robot.encoder_update_loop)
encoder_thread.start()

ultrasonic_thread = Thread(target=robot.ultrasonic_update_loop)
ultrasonic_thread.start()

drive_thread = Thread(target=robot.follow_path)
drive_thread.start()

def loop():
    # Define waypoints to go to in order
    robot.current_goal = Pose(1.0, 0.0)

    # Loop and travel to each waypoint
    if not robot.is_moving:
        # Travel to next waypoint
        robot.create_path()
        # robot.path_queue.pop(0)
        robot.is_moving = True

        # robot.map_class.plot_grid()

    while True:
        sleep(1)


if __name__ == "__main__":
    loop()

