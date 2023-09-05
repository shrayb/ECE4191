import math
from time import sleep, time
from BaseClasses import *
from copy import deepcopy
from Map import *

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
        self.node_radius = 0.05  # metres
        self.path_queue = []  # List of coordinates that the robot will go to in order
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
        self.turn_radius = 0.1257  # Metres
        self.wheel_radius = 0.0524  # Metres
        self.distance_per_tick = (self.wheel_radius * 2 * math.pi) / (74.83 * 48)  # Distance per tick in metres
        self.max_speed = 100  # Upper percentage for maximum speed
        self.slow_speed = 100  # Upper percentage for slower speed
        self.PID_gain = 2  # Raise to make the PID more sensitive, lower to make the PID less sensitive
        self.map_class = None
        self.create_map_class()
        self.is_plot = False
        self.done_plot = False
        self.path_is_tested = False

    def create_map_class(self):
        map_class = Map()
        self.map_class = map_class

    def get_current_goal(self, arena_map=None):
        if self.package is not None:
            return self.package.destination_pose
        else:
            return arena_map.pickup_location

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

    def follow_path(self):
        # THREAD FUNCTION
        # Will drive to whatever waypoints are in the path queue variable in order and remove them
        while True:

            # Check if there are any waypoints in the queue
            if len(self.path_queue) == 0 or self.is_impending_collision or self.path_is_tested:
                continue
            print("No collision. Start driving to waypoint")

            # Drive to first waypoint
            print("PATH QUEUE:", self.path_queue)
            self.drive_to_coordinate(self.path_queue[0])

            # Remove first waypoint from queue
            if self.path_is_tested:
                self.path_queue.pop(0)
                self.path_is_tested = False

            sleep(0.1)

    def encoder_update_loop(self):
        while True:
            self.left_motor.update_encoder()
            self.right_motor.update_encoder()

    def ultrasonic_update_loop(self):
        while True:
            flag, coords_x, coords_y, th = self.detect_obstacle(self.front_left_ultrasonic, self.front_right_ultrasonic)
            sleep(0.1)
            if flag:
                print("Obstacle Found at:", coords_x, coords_y)
            if flag and len(self.path_queue) > 0 and self.path_is_tested:
                # Add the new-found obstacle
                self.map_class.add_obstacle_to_grid(th, Pose(coords_x, coords_y))

                # Check for collisions
                is_collision = self.map_class.check_for_collision(self.map_class.path, self.pose)
                print("New collision detected:", is_collision)

                # If collision, re-plan path
                if is_collision:
                    print("Creating new path due to collision")
                    self.is_impending_collision = True
                    self.map_class.plan_path(self.pose, self.current_goal)
                    self.path_queue = self.map_class.path
                    self.path_is_tested = False
                    sleep(0.25)
                    self.is_impending_collision = False
                    print("Done making path")
                    print("Path queue data:", self.path_queue)
                    print("Path coords:")
                    print("\t", self.pose.x, self.pose.y)
                    for point in self.path_queue:
                        print("\t", point.x, point.y)

    def deposit_package(self):
        # Deposit the next package
        # TODO
        pass

    def create_path(self):
        print("Initial Path Creation")
        # Plan a path
        self.map_class.plan_path(self.pose, self.current_goal)
        # Pass coordinates to the queue
        self.path_queue = self.map_class.path
        self.is_impending_collision = False

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
        distance_total = max_ticks * self.distance_per_tick
        initial_pose = deepcopy(self.pose)

        tick_sum = self.left_motor.ticks + self.right_motor.ticks
        while tick_sum < max_ticks:
            # Check if there will be a collision
            if self.is_impending_collision:
                break

            # Calculate the left tick advantage and tick sum
            left_tick_advantage = self.left_motor.ticks - self.right_motor.ticks
            tick_sum = self.left_motor.ticks + self.right_motor.ticks

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

            self.left_motor.set_speed(left_motor_speed)
            self.right_motor.set_speed(right_motor_speed)

            # During this while loop, continuously update the pose of the robot
            if is_turning == 0:  # 0 = Driving
                distance_fraction = tick_sum / max_ticks
                current_distance = distance_total * distance_fraction
                self.pose.x = initial_pose.x + current_distance * math.cos(initial_pose.theta)
                self.pose.y = initial_pose.y + current_distance * math.sin(initial_pose.theta)
            else:  # 1 = Turning counterclockwise, -1 = Turning clockwise
                distance_turned = tick_sum * self.distance_per_tick
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

        # Continuously check if the turn has less than 10 degrees of the turn remaining
        self.tick_check_and_speed_control(turn_ticks, self.max_speed, is_turning)

        # Stop the motors
        self.left_motor.stop()
        self.right_motor.stop()

        sleep(0.1)

        tick_sum = self.left_motor.ticks + self.right_motor.ticks
        distance_turned = (tick_sum / 2) * self.distance_per_tick
        measured_angle = distance_turned / self.turn_radius

        if angle > 0:
            self.pose.theta = initial_pose.theta + measured_angle
        else:
            self.pose.theta = initial_pose.theta - measured_angle

    def do_drive(self, distance):
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
        self.tick_check_and_speed_control(drive_ticks, self.max_speed, 0)

        # Stop the motors
        self.left_motor.stop()
        self.right_motor.stop()
        sleep(0.1)

        # Use the tick count to estimate where the robot is
        tick_sum = self.left_motor.ticks + self.right_motor.ticks
        measure_distance = (tick_sum / 2) * self.distance_per_tick

        self.pose.x = initial_pose.x + measure_distance * math.cos(initial_pose.theta)
        self.pose.y = initial_pose.y + measure_distance * math.sin(initial_pose.theta)

    def drive_to_coordinate(self, coordinate):
        print("Driving from: (", self.pose.x, self.pose.y, ") to (", coordinate.x, coordinate.y, ")")

        # Check if the robot is already there
        if self.pose.equals(coordinate):
            return None

        # Find angle to turn
        goal_angle = math.atan2(coordinate.y - self.pose.y, coordinate.x - self.pose.x)
        angle_difference = goal_angle - self.pose.theta
        if angle_difference > math.pi:
            angle_difference = angle_difference - 2 * math.pi
        elif angle_difference < -math.pi:
            angle_difference = angle_difference + 2 * math.pi
        print("\tTurning from:", self.pose.theta * 180 / math.pi, "degrees by", angle_difference * 180 / math.pi, "degrees")
        print("\t\tStarting turn")
        self.do_turn(angle_difference)
        print("\t\tTurn complete")
        sleep(0.1)

        self.path_is_tested = True

        # Find distance to drive
        distance = math.hypot(coordinate.x - self.pose.x, coordinate.y - self.pose.y)
        print("\tDriving:", distance, "metres")
        print("\t\tStarting drive")
        self.do_drive(distance)
        print("\t\tDrive complete")
        sleep(0.1)

        # If there is an end orientation face it
        if coordinate.theta is not None and self.pose.theta != coordinate.theta:
            angle_difference = coordinate.theta - self.pose.theta
            if angle_difference > math.pi:
                angle_difference = angle_difference - 2 * math.pi
            elif angle_difference < -math.pi:
                angle_difference = angle_difference + 2 * math.pi
            print("\tAdjusting orientation by", angle_difference * 180 / math.pi, "degrees to face", coordinate.theta * 180 / math.pi, "degrees")
            print("\t\tStarting turn")
            self.do_turn(angle_difference)
            print("\t\tTurn complete")

        self.is_moving = False
        sleep(0.1)
    
    def detect_obstacle(self, front_left_ultrasonic=None, front_right_ultrasonic=None):
        left_dist = front_left_ultrasonic.measure_dist()
        right_dist = front_right_ultrasonic.measure_dist()
        x, y, th = self.pose.x, self.pose.y, self.pose.theta

        if left_dist is None or right_dist is None:
            return False, 0, 0, 0

        # Convert to metres
        left_dist /= 100
        right_dist /= 100

        # Make sure reading isn't a wall
        if left_dist < 0.3 and right_dist < 0.3:
            flag = True
            coords_x = x + 0.5 * (left_dist + right_dist) * np.cos(th)
            coords_y = y + 0.5 * (left_dist + right_dist) * np.sin(th)
        elif left_dist < 0.3:
            flag = True
            coords_x = x + left_dist * np.cos(th)
            coords_y = y + left_dist * np.sin(th)
        elif right_dist < 0.3:
            flag = True
            coords_x = x + right_dist * np.cos(th)
            coords_y = y + right_dist * np.sin(th)
        else:
            flag = False
            coords_x, coords_y = None, None

        return flag, coords_x, coords_y, th
