import FakeRPi.GPIO as GPIO
from time import sleep

trigger_pin = 14
echo_pin = 15

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(trigger_pin, GPIO.OUT)
    GPIO.setup(echo_pin, GPIO.IN)

def main_loop():


if __name__ == "__main__":
    setup()
    main_loop()