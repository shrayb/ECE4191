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
colour_sensor = ColourSensor(s3=s3, s2=s2, signal=colour_sensor_signal)

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
robot.colour_sensor = colour_sensor

# # Start encoder process
# with Manager() as manager:
#     left_motor = robot.left_motor
#     right_motor = robot.right_motor
#
#     encoder_process = Process(target=robot.encoder_process, args=(left_motor, right_motor))
#     encoder_process.start()

def Ultrasonic_Test():
    while True:
        # Measure distance
        distance = robot.middle_ultrasonic.measure_dist()

        # Print distance
        print("Distance:", distance)

        # Sleep in while loop
        sleep(0.2)

def Ultrasonic_Test2():
    while True:
        # Measure distances
        distance_middle = robot.middle_ultrasonic.measure_dist()
        distance_front_left = robot.front_left_ultrasonic.measure_dist()
        distance_front_right = robot.front_right_ultrasonic.measure_dist()
        distance_left = robot.left_ultrasonic.measure_dist()
        distance_right = robot.right_ultrasonic.measure_dist()

        # Print all distances
        print("Middle:", distance_middle)
        print("Front left:", distance_front_left)
        print("Front right:", distance_front_right)
        print("Left:", distance_left)
        print("Right:", distance_right)

        # Sleep in while loop
        sleep(0.2)

Ultrasonic_Test2()

# # Initial thread start for localisation
# ultrasonic_thread = Thread(target=Ultrasonic_Test)
# ultrasonic_thread.start()

# drive_thread = Thread(target=robot.drive_thread)
# drive_thread.start()