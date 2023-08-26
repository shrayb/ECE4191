import math
from time import sleep, time
from BaseClasses import *

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
        self.is_close_to_crashing = False  # Goes True if the robot detects something in front of it whilst moving
        self.is_moving = False  # Boolean for if the robot is moving
        #   "stuck": Robot is stuck and has nowhere to travel
        #   "positioned": Robot has arrived to its destination
        #   "completed": Robot has completed the state task
        # Sub-states of "delivering"
        #   "depositing": The robot is depositing the package into the destination
        self.path = None  # Path class that holds all the information about the robots current path
        self.package = None  # Package class that was currently scanned
        self.depositing = False  # Flag for when the conveyor motor is depositing a package
        self.left_motor = None  # Motor class for the left motor
        self.right_motor = None  # Motor class for the right motor
        self.conveyor_motor = None  # Motor class for the conveyor belt motor
        self.colour_sensor = None  # ColourSensor class for the colour sensor
        self.turn_radius = 0.137795  # Metres
        self.wheel_radius = 0.05451  # Metres
        self.distance_per_tick = (self.wheel_radius * 2 * math.pi) / (74.83 * 48)  # Distance per tick in metres
        self.max_speed = 100  # Upper percentage for maximum speed
        self.slow_speed = 75  # Upper percentage for slower speed
        self.PID_gain = 1  # Raise to make the PID more sensitive, lower to make the PID less sensitive

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
        # As long as the is_moving flag is True
        while self.is_moving:
            # Make required moves to follow path
            # TODO
            pass
        # When the is_moving flag is False for any reason, stop the robot's movement
        # TODO

    def deposit_package(self):
        # Deposit the next package
        # TODO
        pass

    def plan_path(self, arena_map=None):
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

    def encoder_update_loop(self):
        while True:
            self.left_motor.update_encoder()
            self.right_motor.update_encoder()

    def tick_check_and_speed_control(self, max_ticks, max_speed):
        """
        Runs the motors until max ticks are reached, also applies PID control to match speed
        """
        left_tick_advantage = self.left_motor.ticks - self.right_motor.ticks

        while self.left_motor.ticks + self.right_motor.ticks < max_ticks:
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

    def do_turn(self, angle):
        # Reset encoders
        self.left_motor.reset_encoder()
        self.right_motor.reset_encoder()

        # Set motor speeds to 0
        self.left_motor.set_speed(0)
        self.right_motor.set_speed(0)

        if angle > 0:  # Turn counterclockwise
            self.left_motor.backward()
            self.right_motor.forward()
        elif angle < 0:  # Turn clockwise
            self.left_motor.forward()
            self.right_motor.backward()
        else:  # Angle of zero given
            return None

        # Calculate how many ticks to do for the given angle
        turn_distance = abs(angle) * self.turn_radius
        turn_ticks = (turn_distance / self.distance_per_tick) * 2

        # Calculate how many ticks to do for the given angle minus 20 degrees
        turn_distance = (abs(angle) - 20 * (math.pi / 180)) * self.turn_radius
        turn_distance = max(turn_distance, 0)
        turn_minus_10_ticks = (turn_distance / self.distance_per_tick) * 2

        # Continuously check if the turn has less than 10 degrees of the turn remaining
        self.tick_check_and_speed_control(turn_minus_10_ticks, self.max_speed)

        # Slow down the motors to 50 percent for the remaining 10 degrees of the turn. This is to reduce overshoot
        self.left_motor.set_speed(self.slow_speed)
        self.right_motor.set_speed(self.slow_speed)

        # Continuously check if the turn is completed
        self.tick_check_and_speed_control(turn_ticks, self.slow_speed)

        # Stop the motors
        self.left_motor.stop()
        self.right_motor.stop()

        tick_sum = self.left_motor.ticks + self.right_motor.ticks
        distance_turned = (tick_sum / 2) * self.distance_per_tick
        measured_angle = distance_turned / self.turn_radius

        if angle > 0:
            self.pose.theta += measured_angle
        else:
            self.pose.theta -= measured_angle

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

        # Calculate how many ticks to do for the given distance minus 10 centimetres
        drive_minus_5_ticks = ((abs(distance) - 0.1) / self.distance_per_tick) * 2
        drive_minus_5_ticks = max(drive_minus_5_ticks, 0)

        # Continuously check if the robot has driven most of the way
        self.tick_check_and_speed_control(drive_minus_5_ticks, self.max_speed)

        # Slow down the motors to 50 percent for the remaining 5 cm of the drive
        self.left_motor.set_speed(self.slow_speed)
        self.right_motor.set_speed(self.slow_speed)

        # Continuously check if the drive is completed
        self.tick_check_and_speed_control(drive_ticks, self.slow_speed)

        # Stop the motors
        self.left_motor.stop()
        self.right_motor.stop()

        # Use the tick count to estimate where the robot is
        tick_sum = self.left_motor.ticks + self.right_motor.ticks
        measure_distance = (tick_sum / 2) * self.distance_per_tick

        self.pose.x += measure_distance * math.cos(self.pose.theta)
        self.pose.y += measure_distance * math.sin(self.pose.theta)

    def drive_to_coordinate(self, coordinate):
        print("Driving from: (", self.pose.x, self.pose.y, ") to (", coordinate.x, coordinate.y, ")")

        # Check if the robot is already there
        if coordinate.x != self.pose.x or coordinate.y != self.pose.y:
            # Find angle to turn
            goal_angle = math.atan2(coordinate.y - self.pose.y, coordinate.x - self.pose.x)
            angle_difference = goal_angle - self.pose.theta
            print("\tTurning from:", self.pose.theta * 180 / math.pi, "degrees by", angle_difference * 180 / math.pi, "degrees")
            print("\t\tStarting turn")
            self.do_turn(angle_difference)
            print("\t\tTurn complete")
            sleep(0.25)

        # Find distance to drive
        distance = math.hypot(coordinate.x - self.pose.x, coordinate.y - self.pose.y)
        print("\tDriving:", distance, "metres")
        print("\t\tStarting drive")
        self.do_drive(distance)
        print("\t\tDrive complete")
        sleep(0.25)

        # If there is an end orientation face it
        if coordinate.theta is not None and self.pose.theta != coordinate.theta:
            angle_difference = coordinate.theta - self.pose.theta
            print("\tAdjusting orientation by", angle_difference * 180 / math.pi, "degrees to face", coordinate.theta * 180 / math.pi, "degrees")
            print("\t\tStarting turn")
            self.do_turn(angle_difference)
            print("\t\tTurn complete")

        sleep(0.25)
