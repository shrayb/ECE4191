import FakeRPi.GPIO as GPIO
from time import sleep

test_pin = 3

def setup():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(test_pin, GPIO.OUT)

def loop():
    while True:
        # Turn high
        GPIO.output(test_pin, GPIO.HIGH)

        sleep(1)

        # Turn low
        GPIO.output(test_pin, GPIO.LOW)

        sleep(1)