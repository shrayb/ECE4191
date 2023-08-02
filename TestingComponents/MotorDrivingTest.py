# Simple single motor test, go forwards 2 seconds, stop for 2 seconds
# go backwards for 2 seconds, stop for 2 seconds, repeat

import RPi.GPIO as GPIO
from time import sleep

motor_left_enable = 12
motor_left_PWM0 = 16
motor_left_PWM1 = 20
# motor_right_enable =
# motor_right_positive =
# motor_right_negative =

def setup():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(motor_left_enable, GPIO.OUT)
    GPIO.setup(motor_left_PWM0, GPIO.OUT)
    GPIO.setup(motor_left_PWM1, GPIO.OUT)

def loop():
    PWM0 = GPIO.PWM(motor_left_enable, 50)
    while True:
        # Go forwards
        PWM0.start(50)
        GPIO.output(motor_left_PWM0, GPIO.HIGH)
        GPIO.output(motor_left_PWM1, GPIO.LOW)

        print()

        # Wait two seconds
        sleep(2)

        # Stop
        PWM0.stop()
        # Wait two seconds
        sleep(2)

        # Go backwards
        PWM0.start(50)
        GPIO.output(motor_left_PWM0, GPIO.LOW)
        GPIO.output(motor_left_PWM1, GPIO.HIGH)

        # Wait two seconds
        sleep(2)

        # Stop
        PWM0.stop()

        # Wait two seconds
        sleep(2)

if "__main__" == __name__:
    setup()
    loop()