import socket
from threading import Thread
from BaseClasses import *
from copy import deepcopy
from client import *
from time import time, sleep
import numpy as np

class Robot:
    def __init__(self, pose=None):
        # Robot crucial variables
        self.pose = pose  # Pose class x, y, Theta, of the robot
        self.current_goal = None  # Current coordinate the robot wants to end at

        # Robot tunable parameters
        self.turn_radius = 0.126  # Metres make bigger to turn more make smaller to turn less
        self.wheel_radius = 0.05401 / 2  # Metres
        self.distance_per_tick = (2 * math.pi * self.wheel_radius) / (74.83 * 24)  # 0.00012265  # Distance per tick in metres make bigger to drive less make smaller to drive more
        self.max_speed = 99  # Upper percentage for maximum speed
        self.slow_speed = 99  # Upper percentage for slower speed
        self.PID_gain = 1  # Raise to make the PID more sensitive, lower to make the PID less sensitive
        self.PID_turning = 1  # Gain for turning PID
        self.distance_error = 0.02  # Metres accurate
        self.angle_error = 3  # Degrees accurate
        self.map_size = (1.2, 1.2)  # Map size in xy metres, used to determine if an ultrasonic reading is a wall
        self.return_destination = Pose(0.8, 0.30)  # Place to return to before calibrating
        self.package_scanning_count = 50  # Number of similar package reading distances required to decide the package is correct
        self.distance_brackets = [[0.2, 0.10], [0.10, 0.06], [0.06, 0.005]]  # ABC # TODO CHANGED THE BRACKETS

        # Robot component classes
        self.left_motor = None  # Motor class for the left motor
        self.right_motor = None  # Motor class for the right motor
        self.conveyor_motor = None  # Motor class for the conveyor belt motor
        self.front_left_ultrasonic = None  # Front left ultrasonic sensor class
        self.front_right_ultrasonic = None  # Front right ultrasonic sensor class
        self.middle_ultrasonic = None  # Rear left ultrasonic sensor class
        self.left_ultrasonic = None  # Rear left ultrasonic sensor class
        self.right_ultrasonic = None  # Rear left ultrasonic sensor class
        self.colour_sensor = None  # Class for the colour sensor
        self.package_ultrasonic = None  # Class for ultrasonic sensor to detect package
        self.limit_switch = None  # Class for the limit switch

        # Movement variables and flags
        self.max_tick_factor = 0.9
        self.is_moving = False  # Boolean for if the robot is moving
        self.is_impending_collision = False  # Goes True if the robot detects something in front of it whilst moving
        self.safe_reversing = False  # Goes true when the robot is reversing after being stuck with an object in front of it
        self.ignore_except_switch = False  # Only stops driving when a limit switch is pressed
        self.do_localise = False  # Goes true when we want the robot to re-localise

        # Package delivering flags
        self.scanning_timeout = 5  # Seconds to scan for before the program assumes there is no package
        self.package = None  # Package class that was currently scanned
        self.delivering = False  # Flag for if the robot is currently driving to deliver a package
        self.depositing = False  # Flag for when the conveyor motor is depositing a package
        self.reversing_counter = 0  # Counter for how many times the robot has reversed before reaching the goal

        # Ultrasonic variables and flags
        self.sensor_readings = set_default_sensor_readings()  # 5 Sensors by 6 columns

        # Thread related flags
        self.end_all_threads = False  # When this is true, all threads (excluding the main thread) will end
        self.end_ultrasonic_thread = False  # When this is true, the ultrasonic thread will end

        # Time variables and flags
        self.time_flag = False  # Goes true when time is being considered (when waiting 5 seconds to see if the obstacle moves)
        self.stopping_time = None  # Saves the current time when the robot starts waiting 5 seconds

        # Network Client Class
        self.client = Client()
        self.client_timestart = None

    """THREADS"""

    def drive_thread(self):
        # DRIVE THREAD
        # Will continuously attempt to drive to the current goal until desired accuracy is reached
        while True:
            sleep(0.05)
            if self.end_all_threads:
                break
            # Check if we want to re-localise the robot
            if self.do_localise:
                # Localise the robot
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
                for index in range(5):
                    is_vision_blocked = self.is_vision_blocked(index)
                    self.sensor_readings[index][0] = is_vision_blocked

                # Check all sensor reading flags and if any are true, then the robot will stay still
                should_it_stay = False
                for index in range(5):
                    if self.sensor_readings[index][0]:
                        should_it_stay = True

                # Once 5 seconds of being stopped waiting for the obstacle to move
                if time() > self.stopping_time + 5:
                    self.time_flag = False
                    # Check if reversing will make the robot crash
                    distance_to_reverse = 0.25  # Metres
                    end_pose = create_point(self.pose, -distance_to_reverse, self.pose.theta)
                    if 0.25 <= end_pose.x <= self.map_size[0] - 0.25 and 0.25 <= end_pose.y <= self.map_size[1] - 0.25:
                        # If the robot reverses 4 times before reaching the current goal
                        if self.reversing_counter >= 4:
                            # Drive to top right corner, then drive to localisation spot, then localise
                            self.ignore_except_switch = True
                            self.is_impending_collision = False
                            self.drive_to_coordinate(Pose(self.map_size[0] - 0.30, self.map_size[1] - 0.30))
                            self.drive_to_coordinate(self.return_destination)

                            # Reset all flags
                            self.delivering = False
                            self.current_goal = None
                            self.is_impending_collision = False
                            self.do_localise = True
                            continue

                        self.safe_reversing = True
                        self.max_tick_factor = 1.0
                        self.reversing_counter += 1
                        self.do_drive(-distance_to_reverse)  # Drive backwards 25 cm
                        self.safe_reversing = False
                        continue
                    else:
                        # Activate emergency function
                        self.mum_im_scared_pick_me_up()

                if should_it_stay:
                    continue

                self.time_flag = False
                self.is_impending_collision = False

            # Drive to current goal
            self.drive_to_coordinate(self.current_goal)

    def ultrasonic_thread(self):
        # ULTRASONIC THREAD and limit switch
        # Saves readings from ultrasonic sensors and limit switch
        self.client_timestart = time()
        while True:
            sleep(0.01)
            if self.end_all_threads or self.end_ultrasonic_thread:
                break

            # Send communication data
            if time() > self.client_timestart + 0.2:
                self.send_pose_and_goal()
                self.client_timestart = time()

            # Update limit switch reading
            self.limit_switch.detect()

            # If in mum im scared pick me up
            if self.ignore_except_switch:
                continue

            continue

            # Update sensor readings which includes a detection flag for collisions
            self.detect_impending_collision(self.front_left_ultrasonic)
            self.detect_impending_collision(self.right_ultrasonic)
            self.detect_impending_collision(self.front_right_ultrasonic)
            self.detect_impending_collision(self.left_ultrasonic)
            self.detect_impending_collision(self.middle_ultrasonic)

            # Check if any sensors detect an impending collision
            if not self.safe_reversing:
                for index in range(5):
                    if self.sensor_readings[index][0]:
                        self.is_impending_collision = True

    def encoder_process(self, left_mot, right_mot):
        # Efficiently updates the encoder ticks based on the encoder states
        while True:
            left_mot.update_encoder()
            right_mot.update_encoder()

    """FUNCTIONS"""

    def send_pose_and_goal(self):
        if self.client.connection_success:
            try:
                if self.current_goal is None:
                    json_pose = {"pose": [self.pose.x * 1000, self.pose.y * 1000, self.pose.theta * 180 / math.pi], "goal": None}
                else:
                    json_pose = {"pose": [self.pose.x * 1000, self.pose.y * 1000, self.pose.theta * 180 / math.pi], "goal": [self.current_goal.x * 1000, self.current_goal.y * 1000]}
                self.client.send_message(json_pose)
            except Exception as e:
                print(f"Communication Timeout: {e}")
                try:
                    self.client = Client()
                except Exception:
                    print("Can't reconnect to server...")
        else:
            try:
                self.client = Client()
            except Exception:
                print("Can't reconnect to server...")

    def get_current_goal(self):
        if self.package is not None:
            return self.package.destination_pose

    def re_localise(self):
        # Kill ultrasonic and limit switch thread
        self.ignore_except_switch = True
        self.end_ultrasonic_thread = True
        sleep(0.1)

        # Face the robot towards the wall in the negative y direction
        new_pose = Pose(self.pose.x, self.pose.y, -math.pi / 2)
        self.drive_to_coordinate(new_pose)

        # Start ultrasonic and limit switch thread
        self.end_ultrasonic_thread = False
        ultrasonic_thread = Thread(target=self.ultrasonic_thread)
        ultrasonic_thread.start()

        # Drive forward slowly until limit switch is triggered
        self.max_tick_factor = 1.0
        self.do_drive(2, max_speed=self.slow_speed)

        # Set y pose
        self.pose.y = self.limit_switch.distance
        self.pose.theta = -math.pi / 2

        # Kill ultrasonic and limit switch thread
        self.end_ultrasonic_thread = True
        sleep(0.1)

        self.limit_switch.triggered = False

        # Drive backwards 10 cm
        self.max_tick_factor = 1.0
        self.do_drive(-0.12)

        # Turn towards the close wall
        if self.pose.x < 0.6:
            new_pose = Pose(self.pose.x, self.pose.y, math.pi)
        else:
            new_pose = Pose(self.pose.x, self.pose.y, 0)
        self.drive_to_coordinate(new_pose)

        # Start ultrasonic and limit switch thread
        self.end_ultrasonic_thread = False
        ultrasonic_thread = Thread(target=self.ultrasonic_thread)
        ultrasonic_thread.start()

        # Drive forward slowly until limit switch is triggered
        self.max_tick_factor = 1.0
        self.do_drive(2, max_speed=self.slow_speed)

        # Set x pose
        if self.pose.x < 0.6:
            self.pose.x = self.limit_switch.distance
        else:
            self.pose.x = self.map_size[0] - self.limit_switch.distance
        self.pose.theta = new_pose.theta

        # Kill ultrasonic and limit switch thread
        self.end_ultrasonic_thread = True
        sleep(0.1)

        self.limit_switch.triggered = False

        # Drive back 10 cm to safety
        self.max_tick_factor = 1.0
        self.do_drive(-0.12)

        self.ignore_except_switch = False

    def is_vision_blocked(self, sensor_index):
        # Check if any are 100
        for index in range(1, len(self.sensor_readings[sensor_index])):
            if self.sensor_readings[sensor_index][index] != 100:
                return True

        return False

    def deposit_package(self):
        # Start ultrasonic and limit switch thread
        self.end_ultrasonic_thread = False
        ultrasonic_thread = Thread(target=self.ultrasonic_thread)
        ultrasonic_thread.start()

        # Drive forward slowly until limit switch is triggered
        self.depositing = True
        self.max_tick_factor = 1.0
        self.do_drive(0.5, max_speed=self.slow_speed)
        self.depositing = False

        # Set y pose
        self.pose.y = 1.2 - self.limit_switch.distance
        self.pose.theta = math.pi / 2

        # Kill ultrasonic and limit switch thread
        self.end_ultrasonic_thread = True
        sleep(0.1)

        self.limit_switch.triggered = False

        # Turn conveyor belt on
        print("\tDepositing package...")
        self.conveyor_motor.forward()

        # Stop all threads
        self.end_all_threads = True

        # Continuously scan until ID change
        while True:
            sleep(0.001)
            new_package_id = self.scan_package_ultrasonic()
            if self.package.ID != new_package_id:
                if new_package_id == 3:
                    self.package = None
                    print("No more packages. Going home...")
                else:
                    self.package = Package(new_package_id)
                    print("Package scanned:", new_package_id)
                break
        # Turn conveyor off
        self.conveyor_motor.stop()
        print("\tPackage delivered.")

        # Check if package exists
        if self.package is not None:
            # Package detected
            self.delivering = True

            # Set current goal
            self.current_goal = self.package.destination_pose
        else:
            # No package detected
            self.delivering = False

            # Return to pre calibration coordinate
            self.current_goal = self.return_destination
            print("Returning to pre localisation pose...")

        self.end_all_threads = False

        # Start ultrasonic
        ultrasonic_thread = Thread(target=self.ultrasonic_thread)
        ultrasonic_thread.start()

        # Drive backwards to clear wall
        self.max_tick_factor = 1.0
        self.do_drive(-0.2)

        # Start drive thread
        drive_thread = Thread(target=self.drive_thread)
        drive_thread.start()

    def tick_check_and_speed_control(self, max_ticks, max_speed, is_turning, direction=1):
        """
        Runs the motors until max ticks are reached, also applies PID control to match speed
        """
        initial_pose = deepcopy(self.pose)
        if is_turning:
            PID_GAIN = self.PID_turning
        else:
            PID_GAIN = self.PID_gain

        # Factor max ticks then calculate the estimated total travel distance
        max_ticks *= self.max_tick_factor
        distance_total = 0.5 * max_ticks * self.distance_per_tick

        tick_sum = self.left_motor.ticks.value + self.right_motor.ticks.value
        start_time = time()
        # print("Safe reverse:", self.safe_reversing)
        # print("do localise:", self.do_localise)
        # print("is impending:", self.is_impending_collision)
        # print("ignore except switch:", self.ignore_except_switch)
        # print("limit swithc:", self.limit_switch.triggered)
        while tick_sum < max_ticks:
            sleep(0.01)
            # Check if there will be a collision
            if not self.ignore_except_switch:
                if not self.safe_reversing and (not self.do_localise and self.is_impending_collision):
                    break

                if not self.safe_reversing and (self.do_localise and self.limit_switch.triggered):
                    break

                if not self.safe_reversing and (not self.do_localise and self.limit_switch.triggered):
                    break

            # Mum im scared pick me up
            if self.ignore_except_switch and self.limit_switch.triggered:
                break

            # Calculate the left tick advantage and tick sum
            left_tick_advantage = self.left_motor.ticks.value - self.right_motor.ticks.value
            tick_sum = self.left_motor.ticks.value + self.right_motor.ticks.value
            tick_percentage = tick_sum / max_ticks

            # In the case of mum im scared pick me up if the time elapsed is greater than the max desired
            # then break
            if time() > start_time + 10:
                break

            # In the case of aligning for depositing and the switch isnt hit, wait for 2 seconds then break to start depositing
            if self.depositing and time() > start_time + 2:
                break

            # Every two ticks slow down the leading motor by 1 speed
            if left_tick_advantage > 0:
                left_motor_speed = max(max_speed - math.floor(left_tick_advantage / (2 / PID_GAIN)), 0)  # Decrease the left motor speed depending on how many ticks ahead the left motor is
                right_motor_speed = max_speed
            elif left_tick_advantage < 0:
                left_motor_speed = max_speed
                right_motor_speed = max(max_speed + math.ceil(left_tick_advantage / (2 / PID_GAIN)), 0)
            else:
                left_motor_speed = max_speed
                right_motor_speed = max_speed

            left_motor_speed *= self.max_speed / 100
            right_motor_speed *= self.max_speed / 100

            self.left_motor.set_speed(left_motor_speed)
            self.right_motor.set_speed(right_motor_speed)

            # During this while loop, continuously update the pose of the robot
            if is_turning == 0:  # 0 = Driving
                current_distance = distance_total * tick_percentage
                self.pose.x = initial_pose.x + current_distance * math.cos(initial_pose.theta) * direction
                self.pose.y = initial_pose.y + current_distance * math.sin(initial_pose.theta) * direction
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

        tick_sum = self.left_motor.ticks.value + self.right_motor.ticks.value
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
            self.tick_check_and_speed_control(drive_ticks, self.slow_speed, 0, np.sign(distance))
        else:
            self.tick_check_and_speed_control(drive_ticks, max_speed, 0, np.sign(distance))

        # Stop the motors
        self.left_motor.stop()
        self.right_motor.stop()
        sleep(0.25)

        # Use the tick count to estimate where the robot is
        tick_sum = self.left_motor.ticks.value + self.right_motor.ticks.value
        measure_distance = 0.5 * tick_sum * self.distance_per_tick

        if distance < 0:
            measure_distance *= -1

        self.pose.x = initial_pose.x + measure_distance * math.cos(initial_pose.theta)
        self.pose.y = initial_pose.y + measure_distance * math.sin(initial_pose.theta)

        if measure_distance < distance:
            return None

        return True

    def calculate_angle_difference(self, coordinate):
        goal_angle = math.atan2(coordinate.y - self.pose.y, coordinate.x - self.pose.x)
        angle_difference = goal_angle - self.pose.theta
        if angle_difference > math.pi:
            angle_difference = angle_difference - 2 * math.pi
        elif angle_difference < -math.pi:
            angle_difference = angle_difference + 2 * math.pi
        return angle_difference

    def drive_to_coordinate(self, coordinate):
        # Check if the robot is already there
        distance_error = calculate_distance_between_points(self.pose, coordinate)
        if distance_error > self.distance_error:  # 3 cm away
            print("\tDriving from: (", round(self.pose.x, 2), round(self.pose.y, 2), ") to (", round(coordinate.x, 2), round(coordinate.y, 2), ")")
            # Do multiple decreasing length turns to dial in to the desired angle
            self.max_tick_factor = 0.9
            for index in range(4):
                # Find angle to turn
                angle_difference = self.calculate_angle_difference(coordinate)
                self.do_turn(angle_difference)
                self.max_tick_factor *= 0.7

            # Turn ultrasonic thread back on for driving forwards or backwards
            self.end_ultrasonic_thread = False
            ultrasonic_thread = Thread(target=self.ultrasonic_thread)
            ultrasonic_thread.start()

            # Set max tick factor back to 0.8 and do multiple decreasing length drives to dial in the desired position
            self.max_tick_factor = 0.8
            for index in range(4):
                # If there is a collision skip the rest of drive to coord function
                if not self.ignore_except_switch and self.is_impending_collision:
                    return None

                # Find distance to drive
                distance = math.hypot(coordinate.x - self.pose.x, coordinate.y - self.pose.y)

                # Find out if you have to drive backwards or forwards to get closer
                angle_difference = self.calculate_angle_difference(coordinate)
                if abs(angle_difference) >= math.pi / 2:
                    distance *= -1
                self.do_drive(distance)
                self.max_tick_factor *= 0.7
            print("\t\tDrive complete")

            # End ultrasonic thread again now that driving is complete
            self.end_ultrasonic_thread = True
            sleep(0.1)  # Wait for thread to die

        drive_pose_accuracy = calculate_distance_between_points(self.pose, coordinate)
        # If there is an end orientation face it
        if coordinate.theta is not None and self.pose.theta != coordinate.theta and drive_pose_accuracy < self.distance_error:
            print("\tTurning from:", round(self.pose.theta * 180 / math.pi, 2), "degrees to:", round(coordinate.theta * 180 / math.pi, 2), "degrees...")
            self.max_tick_factor = 0.9

            # Do multiple decreasing length turns to dial in on the final angle
            for index in range(9):
                angle_difference = coordinate.theta - self.pose.theta
                if angle_difference > math.pi:
                    angle_difference = angle_difference - 2 * math.pi
                elif angle_difference < -math.pi:
                    angle_difference = angle_difference + 2 * math.pi

                # Check if angle difference is low enough for desired
                if calculate_angle_difference(angle1=self.pose.theta, angle2=coordinate.theta) < (self.angle_error * math.pi / 180):
                    break
                self.do_turn(angle_difference)
                self.max_tick_factor *= 0.7

        # If drive was successful check error from waypoint
        waypoint_error_distance = calculate_distance_between_points(self.pose, coordinate)
        waypoint_error_angle = calculate_angle_difference(angle1=self.pose.theta, angle2=coordinate.theta)
        if waypoint_error_distance < self.distance_error and waypoint_error_angle < (self.angle_error * math.pi / 180) and not self.is_impending_collision:  # 3 cm accuracy and 5 degree accuracy
            self.current_goal = None
            self.reversing_counter = 0

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
        if coords.x < 0.15 or coords.x > self.map_size[0] - 0.15 or coords.y < 0.15 or coords.y > self.map_size[1] - 0.15:
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
            print("Obstacle from:", ultrasonic_unit.reading_index, " at distance:", round(sonic_distance, 2), " at coords:", round(coords.x, 2), round(coords.y, 2))
            return None

        # Object not getting closer
        self.sensor_readings[ultrasonic_unit.reading_index][0] = False

    def scan_package_ultrasonic(self):
        # Array for previous readings
        previous_readings = [None] * self.package_scanning_count
        counter = 0
        count = 0
        while True:
            # Sleep for while loop
            sleep(0.001)

            # Do ultrasonic distance scan
            distance = self.package_ultrasonic.measure_dist()
            # Check if distance is None
            if distance is None:
                print("Ultrasonic timed out...")
                continue

            # Add distance to correct array position
            previous_readings[count % self.package_scanning_count] = distance

            # Check if all readings are in the same distance bracket
            package_id = self.similar_distance_bracket(previous_readings)
            if package_id is False:
                count += 1
                continue
            else:
                # If no more packages detected go again until counter reached
                if counter < 1 and package_id == 3:
                    previous_readings = [None] * self.package_scanning_count
                    counter += 1
                    continue
                # If so, create the new package
                return package_id

    def similar_distance_bracket(self, previous_readings):
        similarity_check_list = []
        for reading_index, reading in enumerate(previous_readings):
            # Check for None reading
            if reading is None:
                return False

            # Loop through each bracket
            for jindex, bracket in enumerate(self.distance_brackets):
                # If a reading matches a bracket add the id to the list
                upper_bound = bracket[0]
                lower_bound = bracket[1]

                if upper_bound > reading >= lower_bound:
                    similarity_check_list.append(jindex)

            # If the value wasn't in any bracket add a 3
            if len(similarity_check_list) - 1 != reading_index:
                similarity_check_list.append(3)

        # Check to see if they are all the same
        first_value = similarity_check_list[0]

        for value in similarity_check_list:
            if value != first_value:
                return False

        # If they are all the same, assume correct reading
        return first_value

    def mum_im_scared_pick_me_up(self):
        """
        Find the way home so we can pick him up
        """
        print("MUM IM SCARED PICK ME UP")
        self.is_impending_collision = False

        # Stop driving
        self.left_motor.stop()
        self.right_motor.stop()
        sleep(0.5)

        # Turn right 60 degrees then drive until wall until limit switch is pressed
        while True:
            self.max_tick_factor = 1.0
            self.ignore_except_switch = True
            self.do_turn(-90 * math.pi / 180)
            self.max_tick_factor = 1.0
            self.do_drive(1.0)
            self.ignore_except_switch = False
            self.safe_reversing = True
            self.do_drive(-0.15)
            self.safe_reversing = False

    def establish_connection_to_send(self):
        server_ip = '118.138.20.161'  # Replace with the actual IP address of the receiving computer
        server_port = 137  # Use the same port number as the server

        # Create a socket object and connect to the server
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((server_ip, server_port))

        return client_socket

    def establish_connection_to_receive(self):
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

    def close_connection(self, client_socket, server_socket):
        # Close the client and server sockets
        client_socket.close()
        server_socket.close()

    def send_robot_position(self, client_socket):
        # Send text data to the server
        data = str(self.pose.x) + ' ' + str(self.pose.y) + ' ' + str(self.pose.theta)
        client_socket.send(data.encode())
        print("sent position" , data)

    def receive_robot_position(self, client_socket):
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