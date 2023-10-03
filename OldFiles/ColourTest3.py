import RPi.GPIO as GPIO
from time import time, sleep
from BaseClasses import *

s2 = 16
s3 = 20
signal = 12

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(signal, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(s2, GPIO.OUT)
    GPIO.setup(s3, GPIO.OUT)
    print("\n")

def loop():
    colour_sens = ColourSensor(s0=None, s1=None, s2=s2, s3=s3, signal=signal)
    while True:
        reading = colour_sens.read_colour()
        print("Colour:", reading)
        sleep(0.1)

def endprogram():
    GPIO.cleanup()


if __name__ == '__main__':
    setup()
    try:
        loop()

    except KeyboardInterrupt:
        endprogram()