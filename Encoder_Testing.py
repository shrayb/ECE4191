import math

import RPi.GPIO as GPIO
from time import time, sleep

from threading import Thread

from BaseClasses import Motor

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

def encoder_loop():
    while True:
        left_motor.update_encoder()
        right_motor.update_encoder()

def loop():
    # Turn wheels forward at 50 percent speed
    left_motor.set_speed(100)
    right_motor.set_speed(100)
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
        print("Ticks:", left_motor.ticks)
        print("Left distance:", left_distance, "mm")
        print("Right distance:", right_distance, "mm")
        sleep(0.5)

def turn_right(angle):
    
    left_motor.set_speed(100)
    right_motor.set_speed(100)
    left_motor.forward()
    right_motor.backward()

    encoder_thread = Thread(target=encoder_loop)
    left_motor.reset_encoder()
    right_motor.reset_encoder()

    encoder_thread.start()

    currentAngle = 0
    circum = math.pi * 55
    cpr = 48 * 75
    tick_dist = circum / cpr
    while currentAngle < angle:
        left_distance = tick_dist * left_motor.ticks
        currentAngle = left_distance/140
        sleep(0.001)
    left_motor.stop()
    right_motor.stop()

def turn_left(angle):
    
    left_motor.set_speed(100)
    right_motor.set_speed(100)
    left_motor.backward()
    right_motor.forward()

    encoder_thread = Thread(target=encoder_loop)
    left_motor.reset_encoder()
    right_motor.reset_encoder()

    encoder_thread.start()

    currentAngle = 0
    circum = math.pi * 55
    cpr = 48 * 75
    tick_dist = circum / cpr
    while currentAngle < angle:
        left_distance = tick_dist * left_motor.ticks
        currentAngle = left_distance/140
        sleep(0.001)
    left_motor.stop()
    right_motor.stop()

def turn(angle):
    if angle>0:
        turn_left(angle)
    else:
        turn_right(-angle)
if __name__ == "__main__":
    # loop()
    turn(-math.pi/4)
    turn(math.pi/2)
    turn(math.pi/3)
    turn(-math.pi)


