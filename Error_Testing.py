import math
import numpy as np

import RPi.GPIO as GPIO
from time import time, sleep

from threading import Thread

from BaseClasses import Motor, Pose, Pose, Ultrasonic
from Robot import Robot

MAP_SIZE = 1500
ROBOT_SIZE = 40
E_MAP_SIZE = MAP_SIZE - ROBOT_SIZE

motor_left_enable = 14
motor_left_positive = 15
motor_left_negative = 18
motor_left_encoder_a = 3
motor_left_encoder_b = 4

motor_right_enable = 22
motor_right_positive = 17
motor_right_negative = 27
motor_right_encoder_a = 23
motor_right_encoder_b = 24

front_left_us_echo = 1
front_left_us_trig = 2

front_right_us_echo = 3
front_right_us_trig = 4

# rear_left_us_echo = 5
# rear_left_us_trig = 6

# rear_right_us_echo = 7
# rear_right_us_trig = 8

front_left_us = Ultrasonic(echo_pin= front_left_us_echo, trig_pin= front_left_us_trig)
front_right_us = Ultrasonic(echo_pin=front_right_us_echo, trig_pin=front_right_us_trig)
# rear_left_us = Ultrasonic(echo_pin=rear_left_us_echo, trig_pin=rear_left_us_trig)
# rear_right_us = Ultrasonic(echo_pin=rear_right_us_echo, trig_pin=rear_right_us_trig)

GPIO.setmode(GPIO.BCM)

left_motor = Motor(motor_left_enable, motor_left_positive, motor_left_negative, motor_left_encoder_a, motor_left_encoder_b)
right_motor = Motor(motor_right_enable, motor_right_positive, motor_right_negative, motor_right_encoder_a, motor_right_encoder_b)

def detect_obstacle(front_left_us, front_right_us, robot_pose):
    left_dist = front_left_us.measure_dist()*10
    right_dist = front_right_us.measure_dist()*10
    x, y, th = robot_pose
    
    if left_dist < 150 and right_dist < 150:
        flag = True
        coords_x = x + 0.5 * (left_dist + right_dist) * np.cos(th)
        coords_y = y + 0.5 * (left_dist + right_dist) * np.sin(th)
        print("Obstacle detected at: " + str(coords_x) + ", " + str(coords_y) + " from both US")

    
    elif left_dist < 150:
        flag = True
        coords_x = x + left_dist * np.cos(th)
        coords_y = y + left_dist * np.sin(th)
        print("Obstacle detected at: " + str(coords_x) + ", " + str(coords_y) + " from left US")

    elif right_dist <150:
        flag = True
        coords_x = x + right_dist * np.cos(th)
        coords_y = y + right_dist * np.sin(th)
        print("Obstacle detected at: " + str(coords_x) + ", " + str(coords_y) + " from right US")

    else:
        flag = False
        coords_x, coords_y = None

    return flag, coords_x, coords_y


pose = Pose(0.3, 0.2, 0)
robot = Robot(pose)
robot.left_motor = left_motor
robot.right_motor = right_motor

encoder_thread = Thread(target=robot.encoder_thread)
encoder_thread.start()

scanner_thread = Thread(target=detect_obstacle)
scanner_thread.start()
# def localise():
#     front_left_dist = front_left_us.measure_dist()
#     front_right_dist = front_right_us.measure_dist()
#     rear_left_dist = rear_left_us.measure_dist()
#     rear_right_dist = rear_right_us.measure_dist()

#     return front_left_dist, front_right_dist, rear_left_dist, rear_right_dist

# # Will take each sensor reading and also the current coordinate of the robot and check whether the measruements are valid.
# def validate_measurements(front_left_dist = None, front_right_dist = None, rear_left_dist = None, rear_right_dist = None):
#     ## TODO
#     return None

# # True means worry about it
# def check_intercept_pos(robot_pose):
#     x, y, th = robot_pose
#     x_int = (y - np.tan(th)*x)/(1-np.tan(th))
    
#     if x_int>MAP_SIZE - ROBOT_SIZE or x_int<0:
#         return False
#     else:
#         return True
    
# def check_intercept_neg(robot_pose):
#     x, y, th = robot_pose
#     x = x - E_MAP_SIZE
    
#     x_int = (-y + np.tan(th)*x)/(1+np.tan(th))
    
#     if x_int<-E_MAP_SIZE or x_int>0:
#         return False
#     else:
#         return True

# def detect_obstacle(front_left_dist = None, front_right_dist = None, rear_left_dist = None, rear_right_dist = None, robot_pose = None):
#     front_left_dist, front_right_dist, rear_left_dist, rear_right_dist = 0
#     counter = 0
#     while counter<3:
#         # front_left_dist_check, front_right_dist_check, rear_left_dist_check, rear_right_dist_check = localise()
#         # valid_check = validate_measurements()
#         # while not valid_check:
#         #     front_left_dist_check, front_right_dist_check, rear_left_dist_check, rear_right_dist_check = localise()
#         # front_left_dist, front_right_dist, rear_left_dist, rear_right_dist += front_left_dist_check, front_right_dist_check, rear_left_dist_check, rear_right_dist_check
#         front_left_dist, front_right_dist, rear_left_dist, rear_right_dist += localise()
#         counter += 1

#     front_left_dist = front_left_dist/counter
#     front_right_dist = front_right_dist/counter
#     rear_left_dist = rear_left_dist/counter
#     rear_right_dist = rear_right_dist/counter

#     ## Need to find effective map size based on the robot coordinates
#     x, y, th = robot_pose
#     h1, h2 = 0

#     if np.tan(th)>=0:
#         if check_intercept_pos(robot_pose=robot_pose):
#             if th % np.pi/2 >= np.pi/4:
#                 if np.sin(th)>=0:
#                     h1 = (E_MAP_SIZE-y)/np.sin(th)
#                     h2 = y/np.sin(th)
#                 else:
#                     h1 = (E_MAP_SIZE-y)/np.sin(th-np.pi)
#                     h2 = y/(np.sin(th-np.pi))
#             else: 
#                 if np.sin(th)>=0:
#                     h1 = (E_MAP_SIZE-x)/np.cos(th)
#                     h2 = x/np.cos(th)
#                 else:
#                     h1 = (E_MAP_SIZE-x)/np.cos(th-np.pi)
#                     h2 = x/np.cos(th-np.pi)
#         else:
#             if x<y:
#                 if np.sin(th)>=0:
#                     h1 = (E_MAP_SIZE-y)/np.sin(th)
#                     h2 = x/np.cos(th)
#                 else:
#                     h1 = (E_MAP_SIZE-y)/np.sin(th-np.pi)
#                     h2 = x/np.cos(th-np.pi)
#             else:
#                 if np.sin(th)>=0:
#                     h1 = (E_MAP_SIZE-x)/np.cos(th)
#                     h2 = y/np.sin(th)
#                 else:
#                     h1 = (E_MAP_SIZE-x)/np.cos(th-np.pi)
#                     h2 = y/np.sin(th-np.pi)
#     else:
#         if check_intercept_neg(robot_pose=robot_pose):
#             if th % np.pi > 3*np.pi/2:
#                 if np.sin(th)>=0:
#                     h1 = x/np.cos(np.pi-th)
#                     h2 = (E_MAP_SIZE-x)/np.cos(np.pi-th)
#                 else:
#                     h1 = x/np.cos(2*np.pi-th)
#                     h2 = (E_MAP_SIZE-x)/np.cos(2*np.pi-th)
#             else:
#                 if np.sin(th)>=0:
#                     h1 = (E_MAP_SIZE-y)/np.sin(np.pi-th)
#                     h2 = y/np.sin(np.pi-th)
#                 else:
#                     h1 = (E_MAP_SIZE-y)/np.sin(2*np.pi-th)
#                     h2 = y/np.sin(2*np.pi-th)
#         else:
#             if (x-E_MAP_SIZE)<-y:
#                 if np.sin(th)>=0:
#                     h1 = x/np.cos(np.pi-th)
#                     h2 = y/np.sin(np.pi-th)
#                 else:
#                     h1 = x/np.cos(2*np.pi-th)
#                     h2 = y/np.sin(2*np.pi-th)
#             else:
#                 if np.sin(th)>=0:
#                     h1 = (E_MAP_SIZE-y)/np.sin(np.pi-th)
#                     h2 = (E_MAP_SIZE-x)/np.sin(np.pi-th)
#                 else:
#                     h1 = (E_MAP_SIZE-y)/np.sin(2*np.pi-th)
#                     h2 = (E_MAP_SIZE-x)/np.sin(2*np.pi-th)
                    

#     diag = h1 + h2

#     line_left = front_left_dist+rear_left_dist+ROBOT_SIZE
#     line_right = front_right_dist+rear_right_dist+ROBOT_SIZE
#     line_front = (front_left_dist+front_right_dist)/2

#     uncertainty_meas = 15
#     if np.abs((line_right+line_left)/2-diag)<uncertainty_meas:
#         flag = False
        
#     else:
#         flag = True
#         print("Obstacle detected")
#         obstacle_x = x + np.sin(line_front)
#         obstacle_y = y + np.cos(line_front)
#         obstacle_th = th
#         obstacle_coords = np.array([obstacle_x, obstacle_y, obstacle_th])
#     return flag, obstacle_coords


def loop():
    # print("==================================")
    # print("Currently at (", robot.pose.x, ",", robot.pose.y, ",", robot.pose.theta * 180 / math.pi, ")")
    # print("==================================")
    # x_coord = input("Pick new x coordinate")
    # if x_coord == "":
    #     x_coord = robot.pose.x
    # y_coord = input("Pick new y coordinate")
    # if y_coord == "":
    #     y_coord = robot.pose.y
    # theta_coord = input("Pick new end angle in degrees")
    # if theta_coord == "":
    #     theta_coord = None
    # else:
    #     theta_coord = float(theta_coord) * math.pi / 180

    coords = [Pose(0.3, 0.2, math.pi / 6),
              Pose(0.3, 0.2, 2 * math.pi / 6),
              Pose(0.3, 0.2, 3 * math.pi / 6),
              Pose(0.3, 0.2, 4 * math.pi / 6),
              Pose(0.3, 0.2, 5 * math.pi / 6),
              Pose(0.3, 0.2, 6 * math.pi / 6)]

    for coord in coords:
        # Drive to each coord
        robot.pose = Pose(0.3, 0.2, 0)
        x = input("Press 1 to go again")
        if x == "1":
            robot.drive_to_coordinate(coord)

    sleep(10)
    x = input("hello")
    coords = [Pose(0.3, 0.2, math.pi)]

    for coord in coords:
        # Drive to each coord
        robot.drive_to_coordinate(coord)
        sleep(2)

if __name__ == "__main__":
    loop()



