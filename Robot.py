import socket
from threading import Thread
from BaseClasses import *
from copy import deepcopy

class Robot:
    def __init__(self, pose=None):
        # Robot crucial variables
        self.pose = pose  # Pose class x, y, Theta, of the robot
        self.current_goal = None  # Current coordinate the robot wants to end at

        # Robot tunable parameters
        self.turn_radius = 0.131  # Metres make bigger to turn more make smaller to turn less
        self.wheel_radius = 0.05401 / 2  # Metres
        self.distance_per_tick = (2 * math.pi * self.wheel_radius) / (74.83 * 24)  # 0.00012265  # Distance per tick in metres make bigger to drive less make smaller to drive more
        self.max_speed = 75  # Upper percentage for maximum speed
        self.slow_speed = 75  # Upper percentage for slower speed
        self.PID_gain = 1  # Raise to make the PID more sensitive, lower to make the PID less sensitive
        self.PID_turning = 1  # Gain for turning PID
        self.distance_error = 0.005  # Metres accurate
        self.angle_error = 0.5  # Degrees accurate
        self.map_size = (1.2, 1.2)  # Map size in xy metres, used to determine if an ultrasonic reading is a wall
        self.return_destination = Pose(0.3, 0.4)  # Place to return to before calibrating

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
        self.limit_switch = None  # Class for the limit switch

        # Movement variables and flags
        self.max_tick_factor = 0.9
        self.is_moving = False  # Boolean for if the robot is moving
        self.is_impending_collision = False  # Goes True if the robot detects something in front of it whilst moving
        self.safe_reversing = False  # Goes true when the robot is reversing after being stuck with an object in front of it
        self.do_localise = False  # Goes true when we want the robot to re-localise

        # Package delivering flags
        self.scanning_timeout = 5  # Seconds to scan for before the program assumes there is no package
        self.package = None  # Package class that was currently scanned
        self.delivering = False  # Flag for if the robot is currently driving to deliver a package
        self.depositing = False  # Flag for when the conveyor motor is depositing a package

        # Ultrasonic variables and flags
        self.sensor_readings = set_default_sensor_readings()  # 5 Sensors by 6 columns

        # Thread related flags
        self.end_all_threads = False  # When this is true, all threads (excluding the main thread) will end
        self.end_ultrasonic_thread = False  # When this is true, the ultrasonic thread will end

        # Time variables and flags
        self.time_flag = False  # Goes true when time is being considered (when waiting 5 seconds to see if the obstacle moves)
        self.stopping_time = None  # Saves the current time when the robot starts waiting 5 seconds

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
                    self.safe_reversing = True
                    self.is_impending_collision = False
                    self.do_drive(-0.1)  # Drive backwards 10 cm
                    self.safe_reversing = False

                if should_it_stay:
                    continue

                self.is_impending_collision = False

            # Drive to current goal
            self.drive_to_coordinate(self.current_goal)

    def ultrasonic_thread(self):
        # ULTRASONIC THREAD and limit switch
        # Saves readings from ultrasonic sensors and limit switch
        while True:
            if self.end_all_threads or self.end_ultrasonic_thread:
                break
            sleep(0.1)

            # Send communication data

            # Update limit switch reading
            self.limit_switch.detect()

            # Update sensor readings which includes a detection flag for collisions
            self.detect_impending_collision(self.front_left_ultrasonic)
            self.detect_impending_collision(self.front_right_ultrasonic)
            self.detect_impending_collision(self.middle_ultrasonic)
            self.detect_impending_collision(self.left_ultrasonic)
            self.detect_impending_collision(self.right_ultrasonic)

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

    def get_current_goal(self):
        if self.package is not None:
            return self.package.destination_pose

    def continuous_scan(self):
        """
        Rotates the conveyor belt and scans constantly until a colour is returned then stops the thread.
        """
        initial_time = time()
        while time() < initial_time + self.scanning_timeout:
            # Do a scan attempt
            scan_result = self.scan_attempt()

            if scan_result is not None:
                # Add this package to the packages variable
                print("Scan complete. Result:", scan_result)
                self.package = Package(scan_result)
                return None

            sleep(0.001)

        # Set package to none
        self.package = None

    def scan_attempt(self):
        """
        Makes a scan attempt using the colour sensor. If a colour is detected return the colour, otherwise return None
        """
        colour_reading = self.colour_sensor.read_colour()
        return colour_reading

    def re_localise(self):
        # Kill ultrasonic and limit switch thread
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
        self.do_drive(-0.15)
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
        self.do_drive(-0.15)

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
        self.max_tick_factor = 1.0
        self.do_drive(1, max_speed=self.slow_speed)

        # Set y pose
        self.pose.y = 1.2 - self.limit_switch.distance
        self.pose.theta = math.pi / 2

        # Kill ultrasonic and limit switch thread
        self.end_ultrasonic_thread = True
        sleep(0.1)

        self.limit_switch.triggered = False

        # Drive backwards 10 cm
        self.max_tick_factor = 1.0
        self.do_drive(-0.03)

        # Turn conveyor belt on
        print("\tDepositing package...")
        # self.conveyor_motor.forward()

        # TODO Fix scanning multiple boxes. Don't want to scan old box twice
        sleep(2)

        # Stop all threads
        self.end_all_threads = True

        # Start continuously scanning colours
        self.continuous_scan()

        # Turn conveyor off
        # self.conveyor_motor.stop()
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

    def tick_check_and_speed_control(self, max_ticks, max_speed, is_turning):
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
        while tick_sum < max_ticks:
            sleep(0.01)
            # Check if there will be a collision
            if not self.do_localise and self.is_impending_collision:
                break

            if self.do_localise and self.limit_switch.triggered:
                break

            # Calculate the left tick advantage and tick sum
            left_tick_advantage = self.left_motor.ticks.value - self.right_motor.ticks.value
            tick_sum = self.left_motor.ticks.value + self.right_motor.ticks.value
            tick_percentage = tick_sum / max_ticks

            # Every two ticks slow down the leading motor by 1 speed
            if left_tick_advantage > 0:
                left_motor_speed = max(max_speed - math.floor(left_tick_advantage / (2 / PID_GAIN)), 0)
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
            self.tick_check_and_speed_control(drive_ticks, self.slow_speed, 0)
        else:
            self.tick_check_and_speed_control(drive_ticks, max_speed, 0)

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
            print("\tDriving from: (", self.pose.x, self.pose.y, ") to (", coordinate.x, coordinate.y, ")")
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
            print("\tTurning from:", self.pose.theta, "degrees to:", coordinate.theta * 180 / math.pi, "degrees...")
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
        # print("Drive error:", waypoint_error_distance * 100, "cm")
        # print("Angle error:", waypoint_error_angle * 180 / math.pi, "degrees")
        if waypoint_error_distance < self.distance_error and waypoint_error_angle < (self.angle_error * math.pi / 180) and not self.is_impending_collision:  # 3 cm accuracy and 5 degree accuracy
            self.current_goal = None

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

    def mum_im_scared_pick_me_up(self):
        """
        Find the way home so we can pick him up
        """
        # Stop driving
        self.left_motor.stop()
        self.right_motor.stop()
        sleep(0.5)

        # Turn right 60 degrees then drive until wall until limit switch is pressed
        while not self.limit_switch.triggered:
            self.do_turn(-60 * math.pi / 180)
            self.do_drive(1)

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
