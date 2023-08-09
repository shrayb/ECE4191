import FakeRPi.GPIO as GPIO

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
    def __init__(self, enable_pin=None, input_a=None, input_b=None, speed=100):
        self.enable_pin = enable_pin
        self.input_a = input_a
        self.input_b = input_b
        self.speed = speed  # Speed from 0 to 100
        self.pwm = GPIO.PWM(self.enable_pin, self.speed)

    def set_speed(self, speed=None):
        self.speed = speed
        self.pwm = GPIO.PWM(self.enable_pin, self.speed)

    def forward(self):
        GPIO.output(self.input_a, GPIO.HIGH)
        GPIO.output(self.input_b, GPIO.LOW)

    def backward(self):
        GPIO.output(self.input_a, GPIO.LOW)
        GPIO.output(self.input_b, GPIO.HIGH)

    def stop(self):
        GPIO.output(self.input_a, GPIO.LOW)
        GPIO.output(self.input_b, GPIO.LOW)
