import math
import socket
from time import sleep, time

import numpy as np

from BaseClasses import *
from copy import deepcopy
import random
class Robot:
    def __init__(self, pose=None, state="waiting"):
        self.pose = pose
        self.state = state  # Current state of robot:
        # "waiting": Robot is waiting for the start button to be pressed
        # "idle": Robot is stationary, waiting for package to be placed on top of it
        # "scanning": Package has been placed on robot and now attempts to scan it
        # "delivering": Package is being delivered to a destination
        # "returning": Robot is returning to package pickup location
        # "finished": Robot has delivered all packages
        self.sub_state = None  # Sub-state of robots current state
        # Sub-states of "waiting":
        #   "waiting": Robot is waiting for user to press start button on the robot.
        #   "ready": User has pressed the start button on the robot
        # Sub-states of "idle":
        #   "checking": Robot is checking the package detecting sensors
        #   "found": Robot has found a package on top of the scanner
        # Sub-states of "scanning":
        #   "scanning": A scanning attempt is being made
        #   "success": Scanning succeeded and tag has been decoded
        self.scanning_flag = False
        # Sub-states of "delivering" and "returning":
        #   "planning": Robot is planning its route through an arena. This will happen at the beginning and anytime an obstacle interferes with the current route
        #   "moving": Robot is moving along its planned route
        self.is_impending_collision = False  # Goes True if the robot detects something in front of it whilst moving
        self.is_moving = False  # Boolean for if the robot is moving
        #   "stuck": Robot is stuck and has nowhere to travel
        #   "positioned": Robot has arrived to its destination
        #   "completed": Robot has completed the state task
        # Sub-states of "delivering"
        #   "depositing": The robot is depositing the package into the destination
        self.current_goal = None  # Current coordinate the robot wants to end at
        self.package = None  # Package class that was currently scanned
        self.depositing = False  # Flag for when the conveyor motor is depositing a package
        self.left_motor = None  # Motor class for the left motor
        self.right_motor = None  # Motor class for the right motor
        self.conveyor_motor = None  # Motor class for the conveyor belt motor
        self.front_left_ultrasonic = None  # Front left ultrasonic sensor class
        self.front_right_ultrasonic = None  # Front right ultrasonic sensor class
        self.rear_left_ultrasonic = None  # Rear left ultrasonic sensor class
        self.rear_right_ultrasonic = None  # Rear right ultrasonic sensor class
        self.colour_sensor = None  # ColourSensor class for the colour sensor
        self.turn_radius = 0.1263  # Metres
        self.wheel_radius = 0.053761959  # Metres
        self.distance_per_tick = (self.wheel_radius * 2 * math.pi) / (74.83 * 48)  # Distance per tick in metres
        self.max_speed = 38  # Upper percentage for maximum speed
        self.slow_speed = 38  # Upper percentage for slower speed
        self.PID_gain = 0.8  # Raise to make the PID more sensitive, lower to make the PID less sensitive
        self.map_size = (1.2, 1.2)
        self.sensor_readings = set_default_sensor_readings()  # 5 Sensors by 6 columns
        self.drive_success = False
        self.time_flag = False
        self.stopping_time = None
        self.safe_reversing = False
        self.ramp_up_percent = 0.4
        self.ramp_down_percent = 0.6
        self.turn_accuracy_count = 0
        self.max_tick_factor = 0.8
        self.do_localise = False
        self.limit_switch = None
        self.distance_error = 0.005  # Metres accurate
        self.angle_error = 0.5  # Degrees accurate

    def get_current_goal(self):
        if self.package is not None:
            return self.package.destination_pose

    def continuous_scan(self):
        """
        Run this in a thread I reckon. Rotates the conveyor belt and scans constantly until a colour is returned then stops the thread.
        """
        # Turn motor on
        self.conveyor_motor.set_speed(75)
        self.conveyor_motor.forward()

        while True:
            # Do a scan attempt
            scan_result = self.scan_attempt()

            if scan_result is not None:
                # Add this package to the packages variable
                self.package = Package(scan_result)
                break

        # Turn motor off
        self.conveyor_motor.stop()

    def scan_attempt(self):
        """
        Makes a scan attempt using the colour sensor. If a colour is detected return the colour, otherwise return None
        """
        colour_reading = self.colour_sensor.read_colour()
        return colour_reading

    def re_localise(self):
        # Face
        new_pose = Pose(self.pose.x, self.pose.y, -math.pi / 2)
        self.drive_to_coordinate(new_pose)

        # Drive forward slowly until limit switch is triggered
        self.do_drive(0.20, max_speed=35)

        # Set y pose
        self.pose.y = self.limit_switch.distance
        self.pose.theta = -math.pi / 2

        # Drive backwards 10 cm
        self.do_drive(-0.1)

        # Turn towards the close wall
        if self.pose.x < 0.6:
            new_pose = Pose(self.pose.x, self.pose.y, math.pi)
        else:
            new_pose = Pose(self.pose.x, self.pose.y, 0)
        self.drive_to_coordinate(new_pose)

        # Drive forward slowly until limit switch is triggered
        self.do_drive(0.2, max_speed=35)

        # Set x pose
        self.pose.x = self.limit_switch.distance
        self.pose.theta = new_pose.theta

        # Drive back 10 cm to safety
        self.do_drive(-0.1)

    def drive_thread(self):
        # THREAD FUNCTION
        # Will drive to whatever waypoints are in the path queue variable in order and remove them
        while True:
            sleep(0.05)

            # Check if we want to relocalise the robot
            if self.do_localise:
                self.re_localise()
                self.do_localise = False

            # Check if there is an impending collision
            if self.current_goal is None:
                continue

            if self.is_impending_collision:
                if not self.time_flag:
                    self.time_flag = True
                    self.stopping_time = time()

                # Update all sensors if there is an object in its vision now that its stopped
                for index in range(2):
                    is_vision_blocked = self.is_vision_blocked(index)
                    self.sensor_readings[index][0] = is_vision_blocked

                # Check all sensor reading flags and if any are true, then the robot will stay still
                should_it_stay = False
                for index in range(2):
                    if self.sensor_readings[index][0]:
                        should_it_stay = True

                # Once 5 seconds of being stopped waiting for the obstacle to move
                if time() > self.stopping_time + 5:
                    self.time_flag = False
                    self.safe_reversing = True
                    self.is_impending_collision = False
                    self.do_drive(-0.1)  # Drive backwards 10 cm
                    self.safe_reversing = False

                if should_it_stay:
                    continue

                self.is_impending_collision = False

            # Drive to current goal
            self.drive_to_coordinate(self.current_goal)

    def encoder_thread(self):
        while True:
            self.left_motor.update_encoder()
            self.right_motor.update_encoder()

    def is_vision_blocked(self, sensor_index):
        # Check if any are 100
        for index in range(1, len(self.sensor_readings[sensor_index])):
            if self.sensor_readings[sensor_index][index] != 100:
                return True

        return False

    def ultrasonic_thread(self):
        while True:
            sleep(0.1)
            
            # Update sensor readings which includes a detection flag for collisions
            self.detect_impending_collision(self.front_left_ultrasonic)
            self.detect_impending_collision(self.front_right_ultrasonic)

            # Check if any sensors detect an impending collision
            if not self.safe_reversing:
                for index in range(2):
                    if self.sensor_readings[index][0]:
                        self.is_impending_collision = True

    def deposit_package(self):
        # Deposit the next package
        # TODO
        pass

    def set_state(self, state=None, sub_state=None):
        if state is not None:
            self.state = state
            self.sub_state = sub_state

        if state is None and sub_state is not None:
            self.sub_state = sub_state

    def get_state(self):
        return self.state

    def get_sub_state(self):
        return self.sub_state

    def tick_check_and_speed_control(self, max_ticks, max_speed, is_turning):
        """
        Runs the motors until max ticks are reached, also applies PID control to match speed
        """
        distance_total = 0.5 * max_ticks * self.distance_per_tick
        initial_pose = deepcopy(self.pose)

        tick_sum = self.left_motor.ticks + self.right_motor.ticks
        max_ticks *= self.max_tick_factor
        while tick_sum < max_ticks:
            # Check if there will be a collision
            if self.is_impending_collision or self.limit_switch.triggered:
                break

            # Calculate the left tick advantage and tick sum
            left_tick_advantage = self.left_motor.ticks - self.right_motor.ticks
            tick_sum = self.left_motor.ticks + self.right_motor.ticks
            tick_percentage = tick_sum / max_ticks

            # Every two ticks slow down the leading motor by 1 speed
            if left_tick_advantage > 0:
                left_motor_speed = max(max_speed - math.floor(left_tick_advantage / (2 / self.PID_gain)), 0)
                right_motor_speed = max_speed
            elif left_tick_advantage < 0:
                left_motor_speed = max_speed
                right_motor_speed = max(max_speed + math.ceil(left_tick_advantage / (2 / self.PID_gain)), 0)
            else:
                left_motor_speed = max_speed
                right_motor_speed = max_speed

            # # At the start ramp up speed slowly, then near the end slow it down slowly. Increases final pose accuracy
            # if tick_percentage < self.ramp_up_percent:
            #     left_motor_speed *= max(min((tick_percentage / self.ramp_up_percent), max_speed / 100), self.slow_speed / 100)
            #     right_motor_speed *= max(min((tick_percentage / self.ramp_up_percent), max_speed / 100), self.slow_speed / 100)
            # elif tick_percentage > self.ramp_down_percent:
            #     left_motor_speed *= max(min((1 - tick_percentage) / (1 - self.ramp_down_percent), max_speed / 100), self.slow_speed / 100)
            #     right_motor_speed *= max(min((1 - tick_percentage) / (1 - self.ramp_down_percent), max_speed / 100), self.slow_speed / 100)
            # else:
            #     left_motor_speed *= max_speed / 100
            #     right_motor_speed *= max_speed / 100

            left_motor_speed *= self.slow_speed / 100
            right_motor_speed *= self.slow_speed / 100

            self.left_motor.set_speed(left_motor_speed)
            self.right_motor.set_speed(right_motor_speed)

            # During this while loop, continuously update the pose of the robot
            if is_turning == 0:  # 0 = Driving
                current_distance = distance_total * tick_percentage
                self.pose.x = initial_pose.x + current_distance * math.cos(initial_pose.theta)
                self.pose.y = initial_pose.y + current_distance * math.sin(initial_pose.theta)
            else:  # 1 = Turning counterclockwise, -1 = Turning clockwise
                distance_turned = 0.5 * tick_sum * self.distance_per_tick
                measured_angle = distance_turned / self.turn_radius
                self.pose.theta = initial_pose.theta + is_turning * measured_angle

    def do_turn(self, angle):
        # Reset encoders
        self.left_motor.reset_encoder()
        self.right_motor.reset_encoder()

        # Set motor speeds to 0
        self.left_motor.set_speed(0)
        self.right_motor.set_speed(0)

        if angle > 0:  # Turn counterclockwise
            is_turning = 1
            self.left_motor.backward()
            self.right_motor.forward()
        elif angle < 0:  # Turn clockwise
            is_turning = -1
            self.left_motor.forward()
            self.right_motor.backward()
        else:  # Angle of zero given
            return None

        # Calculate how many ticks to do for the given angle
        turn_distance = abs(angle) * self.turn_radius
        turn_ticks = (turn_distance / self.distance_per_tick) * 2

        # Initial pose
        initial_pose = deepcopy(self.pose)

        # Continuously check if the turn has less than 15 degrees of the turn remaining
        if abs(angle) < (15 * math.pi / 180):
            self.tick_check_and_speed_control(turn_ticks, self.slow_speed, is_turning)
        else:
            self.tick_check_and_speed_control(turn_ticks, self.max_speed, is_turning)

        # Stop the motors
        self.left_motor.stop()
        self.right_motor.stop()

        sleep(0.25)

        tick_sum = self.left_motor.ticks + self.right_motor.ticks
        distance_turned = 0.5 * tick_sum * self.distance_per_tick
        measured_angle = distance_turned / self.turn_radius

        if angle > 0:
            self.pose.theta = initial_pose.theta + measured_angle
        else:
            self.pose.theta = initial_pose.theta - measured_angle

        # Clamp angle from [pi to -pi)
        self.pose.theta = math.atan2(math.sin(self.pose.theta), math.cos(self.pose.theta))

    def do_drive(self, distance, max_speed=None):
        if max_speed is None:
            max_speed = self.max_speed

        # Reset encoders
        self.left_motor.reset_encoder()
        self.right_motor.reset_encoder()

        # Set motor speeds to 0
        self.left_motor.set_speed(0)
        self.right_motor.set_speed(0)

        if distance > 0:  # Drive forward
            self.left_motor.forward()
            self.right_motor.forward()
        elif distance < 0:  # Drive backward
            self.left_motor.backward()
            self.right_motor.backward()
        else:  # Distance of zero given
            return None

        # Calculate how many ticks to do for the given distance
        drive_ticks = (abs(distance) / self.distance_per_tick) * 2

        # Initial pose
        initial_pose = deepcopy(self.pose)

        # Continuously check if the robot has driven most of the way
        if distance < 0.05:  # 5 cm
            self.tick_check_and_speed_control(drive_ticks, self.slow_speed, 0)
        else:
            self.tick_check_and_speed_control(drive_ticks, max_speed, 0)

        # Stop the motors
        self.left_motor.stop()
        self.right_motor.stop()
        sleep(0.25)

        # Use the tick count to estimate where the robot is
        tick_sum = self.left_motor.ticks + self.right_motor.ticks
        measure_distance = 0.5 * tick_sum * self.distance_per_tick

        if distance < 0:
            measure_distance *= -1

        self.pose.x = initial_pose.x + measure_distance * math.cos(initial_pose.theta)
        self.pose.y = initial_pose.y + measure_distance * math.sin(initial_pose.theta)

        if measure_distance < distance:
            return None

        return True

    def drive_to_coordinate(self, coordinate):
        print("Driving from: (", self.pose.x, self.pose.y, ") to (", coordinate.x, coordinate.y, ")")

        # Check if the robot is already there
        distance_error = calculate_distance_between_points(self.pose, coordinate)
        if distance_error > self.distance_error:  # 3 cm away

            for index in range(4):
                # Find angle to turn
                goal_angle = math.atan2(coordinate.y - self.pose.y, coordinate.x - self.pose.x)
                angle_difference = goal_angle - self.pose.theta
                if angle_difference > math.pi:
                    angle_difference = angle_difference - 2 * math.pi
                elif angle_difference < -math.pi:
                    angle_difference = angle_difference + 2 * math.pi
                self.do_turn(angle_difference)
                self.max_tick_factor *= 0.8

            self.max_tick_factor = 0.8
            for index in range(4):
                # Find distance to drive
                distance = math.hypot(coordinate.x - self.pose.x, coordinate.y - self.pose.y)
                goal_angle = math.atan2(coordinate.y - self.pose.y, coordinate.x - self.pose.x)
                angle_difference = goal_angle - self.pose.theta
                if angle_difference > math.pi:
                    angle_difference = angle_difference - 2 * math.pi
                elif angle_difference < -math.pi:
                    angle_difference = angle_difference + 2 * math.pi
                if abs(angle_difference) >= math.pi / 2:
                    distance *= -1
                self.do_drive(distance)
                print("\t\tDrive complete")
                self.max_tick_factor *= 0.7

        drive_pose_accuracy = calculate_distance_between_points(self.pose, coordinate)
        # If there is an end orientation face it
        if coordinate.theta is not None and self.pose.theta != coordinate.theta and drive_pose_accuracy < self.distance_error:
            self.max_tick_factor = 0.9
            for index in range(4):
                angle_difference = coordinate.theta - self.pose.theta
                if angle_difference > math.pi:
                    angle_difference = angle_difference - 2 * math.pi
                elif angle_difference < -math.pi:
                    angle_difference = angle_difference + 2 * math.pi
                self.do_turn(angle_difference)
                self.max_tick_factor *= 0.8

        # If drive was successful check error from waypoint
        waypoint_error_distance = calculate_distance_between_points(self.pose, coordinate)
        waypoint_error_angle = calculate_angle_difference(angle1=self.pose.theta, angle2=coordinate.theta)
        print("Drive error:", waypoint_error_distance * 100, "cm")
        print("Angle error:", waypoint_error_angle * 180 / math.pi, "degrees")
        if waypoint_error_distance < self.distance_error and waypoint_error_angle < (self.angle_error * math.pi / 180) and not self.is_impending_collision:  # 3 cm accuracy and 5 degree accuracy
            self.current_goal = None
            self.max_tick_factor = 0.8
        elif waypoint_error_distance < self.distance_error and waypoint_error_angle >= (self.angle_error * math.pi / 180) and not self.is_impending_collision:
            self.max_tick_factor *= 0.8
        if self.max_tick_factor < 0.3:
            self.current_goal = None
            self.max_tick_factor = 0.8

    def detect_obstacle(self, front_left_ultrasonic=None, front_right_ultrasonic=None):
        left_dist = front_left_ultrasonic.measure_dist()
        right_dist = front_right_ultrasonic.measure_dist()
        if left_dist is None or right_dist is None:
            return None
        x, y, th = self.pose.x, self.pose.y, self.pose.theta

        if left_dist is None or right_dist is None:
            return False, 0, 0, 0

        # Convert to metres
        left_dist /= 100
        right_dist /= 100

        # Make sure reading isnt too close
        acceptable_dist = 0.15
        if left_dist < acceptable_dist and right_dist < acceptable_dist:
            flag = True
            coords_x = x + (front_left_ultrasonic.x_offset + 0.5 * (left_dist + right_dist)) * math.cos(self.pose.theta)
            coords_y = y + (front_left_ultrasonic.x_offset + 0.5 * (left_dist + right_dist)) * math.sin(self.pose.theta)
        elif left_dist < acceptable_dist:
            flag = True
            coords_x = front_left_ultrasonic.y_offset * math.sin(self.pose.theta) + (front_left_ultrasonic.x_offset + left_dist) * math.cos(self.pose.theta)
            coords_y = front_left_ultrasonic.y_offset * math.cos(self.pose.theta) + (front_left_ultrasonic.x_offset + left_dist) * math.sin(self.pose.theta)
        elif right_dist < acceptable_dist:
            flag = True
            coords_x = front_right_ultrasonic.y_offset * math.sin(self.pose.theta) + (front_left_ultrasonic.x_offset + left_dist) * math.cos(self.pose.theta)
            coords_y = front_right_ultrasonic.y_offset * math.cos(self.pose.theta) + (front_left_ultrasonic.x_offset + left_dist) * math.sin(self.pose.theta)
        else:
            flag = False
            coords_x, coords_y = None, None

        return flag, coords_x, coords_y, th

    def detect_impending_collision(self, ultrasonic_unit):
        # Get a reading
        sonic_distance = ultrasonic_unit.measure_dist()

        if sonic_distance is None:
            self.sensor_readings[ultrasonic_unit.reading_index][0] = False
            self.sensor_readings[ultrasonic_unit.reading_index].pop(1)
            self.sensor_readings[ultrasonic_unit.reading_index].append(100)
            return None

        # Check that the distance is within acceptable sensor distance
        if sonic_distance > ultrasonic_unit.maximum_read_distance:
            self.sensor_readings[ultrasonic_unit.reading_index][0] = False
            self.sensor_readings[ultrasonic_unit.reading_index].pop(1)
            self.sensor_readings[ultrasonic_unit.reading_index].append(100)
            return None

        # Create coordinate for ultrasonic
        angle_robot_ultra = math.atan2(ultrasonic_unit.y_offset, ultrasonic_unit.x_offset) + self.pose.theta
        ultra_coords = create_point(self.pose, ultrasonic_unit.hypot, angle_robot_ultra)

        coords = create_point(ultra_coords, sonic_distance, ultrasonic_unit.theta + self.pose.theta)

        # Check if coordinate is a wall, if so return none
        if coords.x < 0.1 or coords.x > self.map_size[0] - 0.1 or coords.y < 0.1 or coords.y > self.map_size[1] - 0.1:
            self.sensor_readings[ultrasonic_unit.reading_index][0] = False
            self.sensor_readings[ultrasonic_unit.reading_index].pop(1)
            self.sensor_readings[ultrasonic_unit.reading_index].append(100)
            return None

        # Add distance to proper array in the correct index
        self.sensor_readings[ultrasonic_unit.reading_index].pop(1)
        self.sensor_readings[ultrasonic_unit.reading_index].append(sonic_distance)

        # If object is getting closer
        if object_getting_closer(self.sensor_readings[ultrasonic_unit.reading_index]):
            self.sensor_readings[ultrasonic_unit.reading_index][0] = True
            return None

        # Object not getting closer
        self.sensor_readings[ultrasonic_unit.reading_index][0] = False

    def establish_connection_to_send():
        server_ip = '118.138.20.161'  # Replace with the actual IP address of the receiving computer
        server_port = 137  # Use the same port number as the server
    
        # Create a socket object and connect to the server
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((server_ip, server_port))

        return client_socket

    def establish_connection_to_receive():
        # Define the server IP address and port
        server_ip = '118.138.20.161'  # Replace with the actual IP address of the receiving computer
        server_port = 137  # Use the same port number as the server

        # Create a socket object and bind it to the server address
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind((server_ip, server_port))

        # Accept a connection from a client
        client_socket,client_address = server_socket.accept()
        print(f"Accepted connection from {client_address}")
        return server_socket

    def close_connection(client_socket, server_socket):
        # Close the client and server sockets
        client_socket.close()
        server_socket.close()

    def send_robot_position(self,client_socket):
        # Send text data to the server
        data = str(self.pose.x) + ' ' + str(self.pose.y) + ' ' + str(self.pose.theta)
        client_socket.send(data.encode())
        print("sent position" , data)

    def receive_robot_position(self,client_socket):
        # Receive and print data from the client
        data = client_socket.recv(1024).decode()
        print(f"Received: {data}")
        return data


def object_getting_closer(array):
    # Check if all elements of array are descending from 1st index to end
    for index in range(1, len(array) - 1):
        first_val = array[index]
        second_val = array[index + 1]

        # If the distance goes up instead of down, it's not getting closer
        if first_val == 100 or second_val == 100:
            return False

        if first_val < second_val:
            return False

    return True

def set_default_sensor_readings():
    sensor_readings = []
    for index in range(5):  # 5 ultrasonics
        sensor_readings.append([0] * 6)

    return sensor_readings

def calculate_angle_difference(angle1=None, angle2=None):
    if angle2 is None:
        return 0

    pointA = Pose(math.cos(angle1), math.sin(angle1))
    pointB = Pose(math.cos(angle2), math.sin(angle2))

    theta = math.acos(pointA.dot(pointB) / (pointA.magnitude() * pointB.magnitude()))
    return theta
