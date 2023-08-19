import RPi.GPIO as GPIO
from time import sleep, time

class Pose:
    def __init__(self, x=None, y=None, theta=None):
        self.x = x  # x coordinate of objects pose
        self.y = y  # y coordinate of objects pose
        self.theta = theta  # The angle the object is "facing" measured counter-clockwise from the positive x-axis.

class Point:
    def __init__(self, x=None, y=None):
        self.x = x
        self.y = y

class Polygon:
    def __init__(self, vertices=None):
        self.vertices = vertices  # List of Point classes that define the bounding box of the Polygon

class Destination:
    """
    Destination might have to be more complex than id, x, and y. It might need a range where dropping the package is acceptable. It also needs an angle at which the "opening" is
    """
    def __init__(self, uid=None, x=None, y=None, deposit_pose=None):
        self.uid = uid  # A unique id number for a destination e.g: "red", "green", "blue"
        self.x = x  # x coordinate of destination
        self.y = y  # y coordinate of destination
        self.deposit_pose = deposit_pose

class Obstacle:
    def __init__(self, boundary=None, tolerance=None):
        self.boundary = boundary  # A polygon class that makes the bounding box of the obstacle
        self.tolerance = tolerance  # The distance in metres the robot is allowed to the bounding box of the obstacle

class Path:
    def __init__(self):
        self.waypoint_queue = None

class Package:
    def __init__(self, destination=None):
        self.destination = destination

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
            red_reading = self.get_reading()

            self.read_green()
            green_reading = self.get_reading()

            self.read_blue()
            blue_reading = self.get_reading()

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

    def get_reading(self):
        start_time = time()
        for impulse_count in range(self.num_of_cycles):
            GPIO.wait_for_edge(self.signal, GPIO.FALLING)
        signal_duration = time() - start_time
        reading_value = self.num_of_cycles / signal_duration
        return reading_value
