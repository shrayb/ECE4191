import RPi.GPIO as GPIO
from time import sleep, time
import math

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
        self.pwm = GPIO.PWM(self.enable_pin, 2000)
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