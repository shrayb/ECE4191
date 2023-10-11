from PinAssignment import *
import RPi.GPIO as GPIO

from threading import Thread
from multiprocessing import Process, Value, Queue

from BaseClasses import *
from Robot import Robot

# Rasp pi stuff
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Create component classes
left_motor = Motor(motor_left_enable, motor_left_positive, motor_left_negative, motor_left_encoder_a)
right_motor = Motor(motor_right_enable, motor_right_positive, motor_right_negative, motor_right_encoder_a)

front_left_sonic = Ultrasonic(echo_pin=front_left_sonic_echo, trig_pin=front_left_sonic_trig, x_offset=0.155, y_offset=0.0585, theta=0, reading_index=0, maximum_read_distance=0.15)
front_right_sonic = Ultrasonic(echo_pin=front_right_sonic_echo, trig_pin=front_right_sonic_trig, x_offset=0.155, y_offset=-0.0585, theta=0, reading_index=1, maximum_read_distance=0.15)
middle_sonic = Ultrasonic(echo_pin=middle_sonic_echo, trig_pin=middle_sonic_trig, x_offset=0.155, y_offset=-0.0585, theta=0, reading_index=2, maximum_read_distance=0.15)
left_sonic = Ultrasonic(echo_pin=left_sonic_echo, trig_pin=left_sonic_trig, x_offset=0.155, y_offset=-0.0585, theta=0, reading_index=3, maximum_read_distance=0.15)
right_sonic = Ultrasonic(echo_pin=right_sonic_echo, trig_pin=right_sonic_trig, x_offset=0.155, y_offset=-0.0585, theta=0, reading_index=4, maximum_read_distance=0.15)

limit_switch = LimitSwitch(distance=0.15, switch_pin=limit_switch_pin)

# Create robot class and instantiate component classes
pose = Pose(0.3, 0.4, 0)
robot = Robot(pose)
robot.left_motor = left_motor
robot.right_motor = right_motor
robot.front_left_ultrasonic = front_left_sonic
robot.front_right_ultrasonic = front_right_sonic
robot.middle_ultrasonic = middle_sonic
robot.right_ultrasonic = right_sonic
robot.left_ultrasonic = left_sonic
robot.limit_switch = limit_switch

# Start encoder process
with Manager() as manager:
    left_motor = robot.left_motor
    right_motor = robot.right_motor

    encoder_process = Process(target=robot.encoder_process, args=(left_motor, right_motor))
    encoder_process.start()

# Initial thread start for localisation
ultrasonic_thread = Thread(target=robot.ultrasonic_thread)
ultrasonic_thread.start()

drive_thread = Thread(target=robot.drive_thread)
drive_thread.start()

def mainloop():
    try:
        while True:
            robot.left_motor.reset_encoder()
            robot.right_motor.reset_encoder()
            robot.max_tick_factor = 1.0  # Set max tick to 1 to drive the whole metre
            robot.do_turn(-10000 * math.pi / 180)  # Drive a metre forwards

            while True:
                sleep(1)
                print("Ticks:", (robot.left_motor.ticks.value + robot.right_motor.ticks.value) / 2)
                x = input("Press x to go again")
                break

    # Handle Control-C to stop motors
    except KeyboardInterrupt:
        robot.left_motor.speed = 0
        robot.right_motor.speed = 0
        robot.left_motor.stop()
        robot.right_motor.stop()


if __name__ == "__main__":
    mainloop()

