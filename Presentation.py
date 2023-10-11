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

front_left_sonic = Ultrasonic(echo_pin=front_left_sonic_echo, trig_pin=front_left_sonic_trig, x_offset=0.0875, y_offset=0.060, theta=22.5*math.pi/180, reading_index=0, maximum_read_distance=0.15)
front_right_sonic = Ultrasonic(echo_pin=front_right_sonic_echo, trig_pin=front_right_sonic_trig, x_offset=0.0875, y_offset=-0.060, theta=-22.5*math.pi/180, reading_index=1, maximum_read_distance=0.15)
middle_sonic = Ultrasonic(echo_pin=middle_sonic_echo, trig_pin=middle_sonic_trig, x_offset=0.100, y_offset=0, theta=0, reading_index=2, maximum_read_distance=0.15)
left_sonic = Ultrasonic(echo_pin=left_sonic_echo, trig_pin=left_sonic_trig, x_offset=-0.047, y_offset=0.123, theta=math.pi/2, reading_index=3, maximum_read_distance=0.15)
right_sonic = Ultrasonic(echo_pin=right_sonic_echo, trig_pin=right_sonic_trig, x_offset=-0.047, y_offset=-0.123, theta=-math.pi/2, reading_index=4, maximum_read_distance=0.15)

limit_switch = LimitSwitch(distance=0.1, switch_pin=limit_switch_pin)
package_sonic = Ultrasonic(echo_pin=package_sonic_echo, trig_pin=package_sonic_trig, x_offset=0, y_offset=0, theta=0, reading_index=-1, maximum_read_distance=0.3)

conveyor_motor = Motor(motor_conveyor_enable, motor_conveyor_positive, motor_conveyor_negative)

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
robot.package_ultrasonic = package_sonic
robot.conveyor_motor = conveyor_motor

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
            print("Pose:", round(robot.pose.x, 3), round(robot.pose.y, 3), round(robot.pose.theta, 3))
            x_val = input("Give x value input: ")
            y_val = input("Give y value input: ")
            theta_val = input("Give theta value in degrees: ")
            if x_val != "":
                x_val = float(x_val)
            else:
                x_val = robot.pose.x

            if y_val != "":
                y_val = float(y_val)
            else:
                y_val = robot.pose.y

            if theta_val != "":
                theta_val = float(theta_val) * math.pi / 180
            else:
                theta_val = None

            new_coord = Pose(x_val, y_val, theta_val)
            robot.current_goal = new_coord

            sleep(0.001)

    # Handle Control-C to stop motors
    except KeyboardInterrupt:
        robot.left_motor.speed = 0
        robot.right_motor.speed = 0
        robot.left_motor.stop()
        robot.right_motor.stop()


if __name__ == "__main__":
    mainloop()

