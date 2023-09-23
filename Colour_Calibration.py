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

front_left_sonic = Ultrasonic(echo_pin=front_left_sonic_echo, trig_pin=front_left_sonic_trig, x_offset=0.155, y_offset=0.0585, theta=0, reading_index=0, maximum_read_distance=0.15)
front_right_sonic = Ultrasonic(echo_pin=front_right_sonic_echo, trig_pin=front_right_sonic_trig, x_offset=0.155, y_offset=-0.0585, theta=0, reading_index=1, maximum_read_distance=0.15)

limit_switch = LimitSwitch(distance=0.15, switch_pin=21)
colour_sensor = ColourSensor(s3=20, s2=16, signal=12)

pose = Pose(0.3, 0.2, math.pi/2)
robot = Robot(pose)
robot.left_motor = left_motor
robot.right_motor = right_motor
robot.front_left_ultrasonic = front_left_sonic
robot.front_right_ultrasonic = front_right_sonic
robot.limit_switch = limit_switch
robot.colour_sensor = colour_sensor

def red_calibration():
    red_averages = [0, 0, 0]
    for index in range(100):
        reading = colour_sensor.calibration_call()
        red_averages[0] += reading[0]
        red_averages[1] += reading[1]
        red_averages[2] += reading[2]

    red_averages[0] /= 100
    red_averages[1] /= 100
    red_averages[2] /= 100
    print("red average:", red_averages)

def green_calibration():
    green_average = [0, 0, 0]
    for index in range(100):
        reading = colour_sensor.calibration_call()
        green_average[0] += reading[0]
        green_average[1] += reading[1]
        green_average[2] += reading[2]

    green_average[0] /= 100
    green_average[1] /= 100
    green_average[2] /= 100
    print("green average:", green_average)

def blue_calibration():
    blue_average = [0, 0, 0]
    for index in range(100):
        reading = colour_sensor.calibration_call()
        blue_average[0] += reading[0]
        blue_average[1] += reading[1]
        blue_average[2] += reading[2]

    blue_average[0] /= 100
    blue_average[1] /= 100
    blue_average[2] /= 100
    print("blue average:", blue_average)

def mainloop():
    try:
        while True:
            # Wait for input
            x = input("Press enter to scan red")
            red_calibration()

            # Wait for input
            x = input("Press enter to scan green")
            green_calibration()

            # Wait for input
            x = input("Press enter to scan blue")
            blue_calibration()

            break

    # Handle Control-C to stop motors
    except KeyboardInterrupt:
        robot.left_motor.speed = 0
        robot.right_motor.speed = 0
        robot.left_motor.stop()
        robot.right_motor.stop()


if __name__ == "__main__":
    mainloop()

