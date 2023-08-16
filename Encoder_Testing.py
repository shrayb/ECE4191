import math

import RPi.GPIO as GPIO
from time import time, sleep

from threading import Thread

from BaseClasses import Motor

motor_left_positive = 14
motor_left_negative = 15
motor_left_enable = 18
motor_left_encoder_a = 17
motor_left_encoder_b = 27

motor_right_positive = 22
motor_right_negative = 23
motor_right_enable = 24
motor_right_encoder_a = 10
motor_right_encoder_b = 9

GPIO.setmode(GPIO.BCM)

left_motor = Motor(motor_left_enable, motor_left_positive, motor_left_negative, motor_left_encoder_a, motor_left_encoder_b)
right_motor = Motor(motor_right_enable, motor_right_positive, motor_right_negative, motor_right_encoder_a, motor_right_encoder_b)

def encoder_loop():
    while True:
        left_motor.update_encoder()
        right_motor.update_encoder()

def loop():
    # Turn wheels forward at 50 percent speed
    left_motor.set_speed(50)
    right_motor.set_speed(50)
    left_motor.forward()
    right_motor.forward()

    encoder_thread = Thread(target=encoder_loop)
    encoder_thread.start()

    while True:
        circum = math.pi * 55
        cpr = 48 * 75
        tick_dist = circum / cpr
        left_distance = tick_dist * left_motor.ticks
        right_distance = tick_dist * right_motor.ticks
        # print("Ticks:", left_motor.ticks)
        print("Left distance:", left_distance, "mm")
        print("Right distance:", right_distance, "mm")
        sleep(0.25)


if __name__ == "__main__":
    loop()
