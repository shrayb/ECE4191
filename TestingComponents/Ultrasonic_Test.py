import RPi.GPIO as GPIO
from time import time, sleep

trigger_pin = 14
echo_pin = 15

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(trigger_pin, GPIO.OUT)
    GPIO.setup(echo_pin, GPIO.IN)

def main_loop():
    # Trigger the ultrasonic sensor
    GPIO.output(trigger_pin, GPIO.HIGH)

    # Wait 0.00001
    sleep(0.00001)

    # Stop the trigger
    GPIO.output(trigger_pin, GPIO.LOW)
    start_time = time()
    stop_time = time()

    # Save start time
    while GPIO.input(echo_pin) == 0:
        start_time = time()

    # Save time echo arrives
    while GPIO.input(echo_pin) == 1:
        stop_time = time()

    time_elapsed = stop_time - start_time
    distance = (time_elapsed * 343000) / 2

    print("Distance:", distance, "mm")


if __name__ == "__main__":
    setup()
    main_loop()
