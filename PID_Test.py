import math

import RPi.GPIO as GPIO
from threading import Thread

from BaseClasses import Motor, Pose
from Robot import Robot
import math

motor_left_enable = 14
motor_left_positive = 18
motor_left_negative = 15
motor_left_encoder_a = 3
motor_left_encoder_b = 4

motor_right_enable = 22
motor_right_positive = 17
motor_right_negative = 27
motor_right_encoder_a = 23
motor_right_encoder_b = 24

GPIO.setmode(GPIO.BCM)

left_motor = Motor(motor_left_enable, motor_left_positive, motor_left_negative, motor_left_encoder_a, motor_left_encoder_b)
right_motor = Motor(motor_right_enable, motor_right_positive, motor_right_negative, motor_right_encoder_a, motor_right_encoder_b)

pose = Pose(0, 0, 0)
robot = Robot(pose)
robot.left_motor = left_motor
robot.right_motor = right_motor

encoder_thread = Thread(target=robot.encoder_update_loop)
encoder_thread.start()

def loop():
    while True:
        print("==================================")
        print("Currently at (", robot.pose.x, robot.pose.y, ")")
        print("==================================")
        x_coord = input("Pick new x coordinate")
        if x_coord == "":
            x_coord = robot.pose.x
        y_coord = input("Pick new y coordinate")
        if y_coord == "":
            y_coord = robot.pose.y
        theta_coord = input("Pick new end angle in degrees")
        if theta_coord == "":
            theta_coord = None
        else:
            theta_coord = float(theta_coord) * math.pi / 180
        coord = Pose(int(x_coord), int(y_coord), theta_coord)

        robot.drive_to_coordinate(coord)


if __name__ == "__main__":
    loop()
