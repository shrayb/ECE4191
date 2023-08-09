import RPi.GPIO as GPIO
from time import sleep

in1 = 24  # Red
in2 = 23  # Orange
en = 25  # Brown

GPIO.setmode(GPIO.BCM)
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(en, GPIO.OUT)
GPIO.output(in1, GPIO.LOW)
GPIO.output(in2, GPIO.LOW)
p = GPIO.PWM(en, 1000)
p.start(100)


while 1:
    p.ChangeDutyCycle(75)
    # Forward
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)