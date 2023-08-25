import RPi.GPIO as GPIO
from time import time, sleep
from BaseClasses import ColourSensor

s0 = 17
s1 = 27
s2 = 23
s3 = 24
signal = 25

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(signal, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(s0, GPIO.OUT)
    GPIO.setup(s1, GPIO.OUT)
    GPIO.setup(s2, GPIO.OUT)
    GPIO.setup(s3, GPIO.OUT)
    print("\n")

def loop():
    colour_sens = ColourSensor(s0, s1, s2, s3, signal)
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