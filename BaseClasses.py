import RPi.GPIO as GPIO
from time import sleep, time
import math
import numpy as np

class Pose:
    def __init__(self, x=None, y=None, theta=None):
        self.x = x  # x coordinate of objects pose
        self.y = y  # y coordinate of objects pose
        self.theta = theta  # The angle the object is "facing" measured counter-clockwise from the positive x-axis.

class Polygon:
    def __init__(self, vertices=None):
        self.vertices = vertices  # List of Point classes that define the bounding box of the Polygon

class Obstacle:
    def __init__(self, boundary=None, tolerance=None):
        self.boundary = boundary  # A polygon class that makes the bounding box of the obstacle
        self.tolerance = tolerance  # The distance in metres the robot is allowed to the bounding box of the obstacle

class Path:
    def __init__(self):
        self.waypoint_queue = None

class Package:
    def __init__(self, colour=None):
        self.colour = colour  # A unique id number for a destination e.g: "red", "green", "blue"
        self.destination_pose = None
        self.identify_destination()

    def identify_destination(self):
        if self.colour == "red":
            self.destination_pose = Pose(x=0.25, y=1.3, theta=math.pi/2)
        if self.colour == "green":
            self.destination_pose = Pose(x=0.75, y=1.3, theta=math.pi/2)
        if self.colour == "blue":
            self.destination_pose = Pose(x=1.25, y=1.3, theta=math.pi/2)

class Motor:
    def __init__(self, enable_pin=None, input_a=None, input_b=None, encoder_a=None, encoder_b=None, speed=100):
        self.enable_pin = enable_pin
        self.input_a = input_a
        self.input_b = input_b
        self.encoder_a = encoder_a
        self.encoder_b = encoder_b
        GPIO.setup(self.enable_pin, GPIO.OUT)
        GPIO.setup(self.input_a, GPIO.OUT)
        GPIO.setup(self.input_b, GPIO.OUT)
        GPIO.setup(self.encoder_a, GPIO.IN)
        GPIO.setup(self.encoder_b, GPIO.IN)
        self.speed = speed  # Speed from 0 to 100
        self.pwm = GPIO.PWM(self.enable_pin, 1000)
        self.pwm.start(self.speed)
        self.ticks = 0
        self.encoder_state = self.read_encoder()

    def set_speed(self, speed=None):
        self.speed = speed
        self.pwm.ChangeDutyCycle(speed)

    def forward(self):
        GPIO.output(self.input_a, GPIO.HIGH)
        GPIO.output(self.input_b, GPIO.LOW)

    def backward(self):
        GPIO.output(self.input_a, GPIO.LOW)
        GPIO.output(self.input_b, GPIO.HIGH)

    def stop(self):
        GPIO.output(self.input_a, GPIO.LOW)
        GPIO.output(self.input_b, GPIO.LOW)

    def read_encoder(self):
        encoder_a_reading = GPIO.input(self.encoder_a)
        encoder_b_reading = GPIO.input(self.encoder_b)
        new_state = (encoder_a_reading, encoder_b_reading)
        return new_state

    def reset_encoder(self):
        self.ticks = 0
        new_state = self.read_encoder()
        self.encoder_state = new_state

    def update_encoder(self):
        new_state = self.read_encoder()

        # Compare states
        if new_state != self.encoder_state:
            self.encoder_state = new_state
            self.ticks += 1

class ColourSensor:
    def __init__(self, s0, s1, s2, s3, signal):
        self.s0 = s0
        self.s1 = s1
        self.s2 = s2
        self.s3 = s3
        self.signal = signal
        self.num_of_cycles = 10

    def read_colour(self):
        red_count = 0
        green_count = 0
        blue_count = 0
        for index in range(5):
            # Read each colour sensor
            self.read_red()
            red_reading = self.single_reading()

            self.read_green()
            green_reading = self.single_reading()

            self.read_blue()
            blue_reading = self.single_reading()

            if red_reading > 20000 or green_reading > 20000 or blue_reading > 20000:
                # Find the largest
                if red_reading > green_reading and red_reading > blue_reading:
                    red_count += 1
                if green_reading > red_reading and green_reading > blue_reading:
                    green_count += 1
                if blue_reading > green_reading and blue_reading > red_reading:
                    blue_count += 1

        if red_count == 0 and blue_count == 0 and green_count == 0:
            return None

        # Find which colour shows up most
        if red_count > green_count and red_count > blue_count:
            return "red"
        elif green_count > blue_count and green_count > red_count:
            return "green"
        else:
            return "blue"

    def read_red(self):
        GPIO.output(self.s2, GPIO.LOW)
        GPIO.output(self.s3, GPIO.LOW)

    def read_green(self):
        GPIO.output(self.s2, GPIO.HIGH)
        GPIO.output(self.s3, GPIO.HIGH)

    def read_blue(self):
        GPIO.output(self.s2, GPIO.LOW)
        GPIO.output(self.s3, GPIO.HIGH)

    def single_reading(self):
        start_time = time()
        for impulse_count in range(self.num_of_cycles):
            GPIO.wait_for_edge(self.signal, GPIO.FALLING)
        signal_duration = time() - start_time
        reading_value = self.num_of_cycles / signal_duration
        return reading_value

# Measure distance
class Ultrasonic:
    
    def __init__(self, trig_pin = None, echo_pin = None):
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        GPIO.setup(self.trig_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)

    def measure_dist(self):
        GPIO.output(self.trig_pin, True)
        sleep(0.00001)
        GPIO.output(self.trig_pin, False)

        while GPIO.input(self.echo_pin) == 0:
            pulse_start = time.time()

        while GPIO.input(self.echo_pin) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150  # Speed of sound in cm/s

        return distance
    
    def localise(self, front_left_us, front_right_us, rear_left_us, rear_right_us):
        front_left_dist = front_left_us.measure_dist()
        front_right_dist = front_right_us.measure_dist()
        rear_left_dist = rear_left_us.measure_dist()
        rear_right_dist = rear_right_us.measure_dist()

        return front_left_dist, front_right_dist, rear_left_dist, rear_right_dist
    
    def check_intercept_pos(self, robot_pose):
        x, y, th = robot_pose
        x_int = (y - np.tan(th)*x)/(1-np.tan(th))
        
        if x_int>1500 - 40 or x_int<0:
            return False
        else:
            return True
        
    def check_intercept_neg(self, robot_pose):
        x, y, th = robot_pose
        x = x - 1460
        
        x_int = (-y + np.tan(th)*x)/(1+np.tan(th))
        
        if x_int<-1460 or x_int>0:
            return False
        else:
            return True
        
    def detect_obstacle_coords(self, front_left_us = None, front_right_us = None, rear_left_us = None, rear_right_us = None, robot_pose = None):
        front_left_dist, front_right_dist, rear_left_dist, rear_right_dist = 0
        counter = 0
        while counter<3:
            # front_left_dist_check, front_right_dist_check, rear_left_dist_check, rear_right_dist_check = localise()
            # valid_check = validate_measurements()
            # while not valid_check:
            #     front_left_dist_check, front_right_dist_check, rear_left_dist_check, rear_right_dist_check = localise()
            # front_left_dist, front_right_dist, rear_left_dist, rear_right_dist += front_left_dist_check, front_right_dist_check, rear_left_dist_check, rear_right_dist_check
            front_left_dist, front_right_dist, rear_left_dist, rear_right_dist += self.localise(front_left_us, front_right_us, rear_left_us, rear_right_us)
            counter += 1

        front_left_dist = front_left_dist/counter
        front_right_dist = front_right_dist/counter
        rear_left_dist = rear_left_dist/counter
        rear_right_dist = rear_right_dist/counter

        ## Need to find effective map size based on the robot coordinates
        x, y, th = robot_pose
        h1, h2 = 0

        if np.tan(th)>=0:
            if self.check_intercept_pos(robot_pose=robot_pose):
                if th % np.pi/2 >= np.pi/4:
                    if np.sin(th)>=0:
                        h1 = (1460-y)/np.sin(th)
                        h2 = y/np.sin(th)
                    else:
                        h1 = (1460-y)/np.sin(th-np.pi)
                        h2 = y/(np.sin(th-np.pi))
                else: 
                    if np.sin(th)>=0:
                        h1 = (1460-x)/np.cos(th)
                        h2 = x/np.cos(th)
                    else:
                        h1 = (1460-x)/np.cos(th-np.pi)
                        h2 = x/np.cos(th-np.pi)
            else:
                if x<y:
                    if np.sin(th)>=0:
                        h1 = (1460-y)/np.sin(th)
                        h2 = x/np.cos(th)
                    else:
                        h1 = (1460-y)/np.sin(th-np.pi)
                        h2 = x/np.cos(th-np.pi)
                else:
                    if np.sin(th)>=0:
                        h1 = (1460-x)/np.cos(th)
                        h2 = y/np.sin(th)
                    else:
                        h1 = (1460-x)/np.cos(th-np.pi)
                        h2 = y/np.sin(th-np.pi)
        else:
            if self.check_intercept_neg(robot_pose=robot_pose):
                if th % np.pi > 3*np.pi/2:
                    if np.sin(th)>=0:
                        h1 = x/np.cos(np.pi-th)
                        h2 = (1460-x)/np.cos(np.pi-th)
                    else:
                        h1 = x/np.cos(2*np.pi-th)
                        h2 = (1460-x)/np.cos(2*np.pi-th)
                else:
                    if np.sin(th)>=0:
                        h1 = (1460-y)/np.sin(np.pi-th)
                        h2 = y/np.sin(np.pi-th)
                    else:
                        h1 = (1460-y)/np.sin(2*np.pi-th)
                        h2 = y/np.sin(2*np.pi-th)
            else:
                if (x-1460)<-y:
                    if np.sin(th)>=0:
                        h1 = x/np.cos(np.pi-th)
                        h2 = y/np.sin(np.pi-th)
                    else:
                        h1 = x/np.cos(2*np.pi-th)
                        h2 = y/np.sin(2*np.pi-th)
                else:
                    if np.sin(th)>=0:
                        h1 = (1460-y)/np.sin(np.pi-th)
                        h2 = (1460-x)/np.sin(np.pi-th)
                    else:
                        h1 = (1460-y)/np.sin(2*np.pi-th)
                        h2 = (1460-x)/np.sin(2*np.pi-th)
                        

        diag = h1 + h2

        line_left = front_left_dist+rear_left_dist+40
        line_right = front_right_dist+rear_right_dist+40
        line_front = (front_left_dist+front_right_dist)/2

        uncertainty_meas = 15
        if np.abs((line_right+line_left)/2-diag)<uncertainty_meas:
            flag = False
            
        else:
            flag = True
            print("Obstacle detected")
            obstacle_x = x + np.sin(line_front)
            obstacle_y = y + np.cos(line_front)
            obstacle_th = th
            obstacle_coords = np.array([obstacle_x, obstacle_y, obstacle_th])
        return flag, obstacle_coords