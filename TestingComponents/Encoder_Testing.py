import RPi.GPIO as GPIO
from time import time, sleep

from threading import Thread

from BaseClasses.BaseClasses import Motor

motor_left_positive = 14
motor_left_negative = 15
motor_left_enable = 18
motor_left_encoder_a = 17
motor_left_encoder_b = 27

left_motor = Motor(motor_left_enable, motor_left_positive, motor_left_negative, motor_left_encoder_a, motor_left_encoder_b)

# Encoder stuff

def setup():
    GPIO.setmode(GPIO.BCM)

def encoder_loop():
    while True:
        left_motor.update_encoder()

def loop():
    # Turn wheels forward at 50 percent speed
    left_motor.set_speed(50)
    left_motor.forward()

    encoder_thread = Thread(target=encoder_loop)
    encoder_thread.start()

    while True:
        print("Ticks:", left_motor.ticks)
        distance = 3.5 * left_motor.ticks
        sleep(0.25)


if __name__ == "__main__":
    setup()
    loop()
