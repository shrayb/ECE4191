# Simple single motor test, go forwards 2 seconds, stop for 2 seconds
# go backwards for 2 seconds, stop for 2 seconds, repeat

import FakeRPi.GPIO as GPIO
from time import sleep

motor_left_enable = 27  # GPIO-0
PWM0 = 32  # GPIO 12 - (PWM-0)
PWM1 = 33  # GPIO 13 - (PWM-1)
motor_left_positive = PWM0
motor_left_negative = PWM1
# motor_right_enable =
# motor_right_positive =
# motor_right_negative =

def setup():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(motor_left_enable, GPIO.OUT)
    GPIO.setup(motor_left_positive, GPIO.OUT)
    GPIO.setup(motor_left_negative, GPIO.OUT)

def loop():
    while True:
        # Go forwards
        GPIO.output(motor_left_enable, GPIO.HIGH)
        GPIO.output(motor_left_positive, GPIO.HIGH)
        GPIO.output(motor_left_negative, GPIO.LOW)

        print()

        # Wait two seconds
        sleep(2)

        # Stop
        GPIO.output(motor_left_positive, GPIO.LOW)
        GPIO.output(motor_left_negative, GPIO.LOW)

        # Wait two seconds
        sleep(2)

        # Go backwards
        GPIO.output(motor_left_enable, GPIO.HIGH)
        GPIO.output(motor_left_positive, GPIO.LOW)
        GPIO.output(motor_left_negative, GPIO.HIGH)

        # Wait two seconds
        sleep(2)

        # Stop
        GPIO.output(motor_left_positive, GPIO.LOW)
        GPIO.output(motor_left_negative, GPIO.LOW)

        # Wait two seconds
        sleep(2)

if "__main__" == __name__:
    setup()
    loop()