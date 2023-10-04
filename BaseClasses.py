import RPi.GPIO as GPIO
from time import sleep, time
import math

def calculate_distance_between_points(point_1=None, point_2=None):
    return math.sqrt((point_1.x - point_2.x) ** 2 + (point_1.y - point_2.y) ** 2)

def do_segments_intersect(segment1=None, segment2=None):
    p1, q1 = segment1.start, segment1.end
    p2, q2 = segment2.start, segment2.end

    # Find the 4 orientations required for
    # the general and special cases
    o1 = intersection_orientation(p1, q1, p2)
    o2 = intersection_orientation(p1, q1, q2)
    o3 = intersection_orientation(p2, q2, p1)
    o4 = intersection_orientation(p2, q2, q1)

    # General case
    if (o1 != o2) and (o3 != o4):
        return True

    # Special Cases

    # p1 , q1 and p2 are collinear and p2 lies on segment p1q1
    if (o1 == 0) and onSegment(p1, p2, q1):
        return True

    # p1 , q1 and q2 are collinear and q2 lies on segment p1q1
    if (o2 == 0) and onSegment(p1, q2, q1):
        return True

    # p2 , q2 and p1 are collinear and p1 lies on segment p2q2
    if (o3 == 0) and onSegment(p2, p1, q2):
        return True

    # p2 , q2 and q1 are collinear and q1 lies on segment p2q2
    if (o4 == 0) and onSegment(p2, q1, q2):
        return True

    # If none of the cases
    return False

def intersection_orientation(p, q, r):
    # To find the orientation of an ordered triplet (p,q,r)

    val = round(((q.y - p.y) * (r.x - q.x)) - ((q.x - p.x) * (r.y - q.y)), 10)
    if val > 0:
        # Clockwise orientation
        return 1
    elif val < 0:
        # Counterclockwise orientation
        return 2
    else:
        # Collinear orientation
        return 0

def onSegment(p, q, r):
    if ((q.x <= max(p.x, r.x)) and (q.x >= min(p.x, r.x)) and
            (q.y <= max(p.y, r.y)) and (q.y >= min(p.y, r.y))):
        return True
    return False

def find_segment_intersection(segment1, segment2):
    xdiff = Pose(segment1.start.x - segment1.end.x, segment2.start.x - segment2.end.x)
    ydiff = Pose(segment1.start.y - segment1.end.y, segment2.start.y - segment2.end.y)

    def det(a, b):
        return a.x * b.y - a.y * b.x

    div = det(xdiff, ydiff)
    if div == 0:
        return None

    d = Pose(det(segment1.start, segment1.end), det(segment2.start, segment2.end))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return Pose(x, y)

def create_point(point=None, dist=None, angle=None):
    return Pose(point.x + dist * math.cos(angle), point.y + dist * math.sin(angle))

class Pose:
    def __init__(self, x=None, y=None, theta=None):
        self.x = x  # x coordinate of objects pose
        self.y = y  # y coordinate of objects pose
        self.theta = theta  # The angle the object is "facing" measured counter-clockwise from the positive x-axis.

    def equals(self, point):
        if round(point.x, 10) == round(self.x, 10) and round(point.y, 10) == round(self.y, 10):
            return True
        else:
            return False

    def magnitude(self, point=None):
        if point is None:
            return math.sqrt(self.x ** 2 + self.y ** 2)
        else:
            return math.sqrt(point.x ** 2 + point.y ** 2)

    def dot(self, point_2):
        return self.x * point_2.x + self.y * point_2.y

    def multiply(self, scalar):
        return Pose(self.x * scalar, self.y * scalar)

class Segment:
    def __init__(self, start_point=None, end_point=None):
        self.start = start_point
        self.end = end_point

    def length(self):
        return calculate_distance_between_points(self.start, self.end)

class Polygon:
    def __init__(self, vertices=None):
        self.vertices = vertices  # List of Point instances
        self.maximum = 3  # metres
        self.segments = self.calculate_segments()
        self.count = self.calculate_count()
        self.centroid = self.calculate_centroid()

    def calculate_centroid(self):
        x_sum = 0
        y_sum = 0
        for vertex in self.vertices:
            x_sum += vertex.x
            y_sum += vertex.y
        x_avg = x_sum / self.count
        y_avg = y_sum / self.count
        centroid = Pose(x_avg, y_avg)
        return centroid

    def calculate_maximum_length(self):
        max_length = 0
        for vertex1_index in range(len(self.vertices)):
            for vertex2_index in range(len(self.vertices) - 1):
                if vertex1_index == vertex2_index:
                    continue
                vertex1 = self.vertices[vertex1_index]
                vertex2 = self.vertices[vertex2_index]
                length = calculate_distance_between_points(vertex1, vertex2)
                if length > max_length:
                    max_length = length
        return max_length

    def calculate_segments(self):
        segments = []
        for index in range(len(self.vertices)):
            vertex1 = self.vertices[index]
            vertex2 = self.vertices[(index + 1) % len(self.vertices)]
            new_segment = Segment(vertex1, vertex2)
            segments.append(new_segment)
        return segments

    def calculate_count(self):
        count = len(self.vertices)
        return count

    def contains(self, point=None):
        # Cast ray to the right and count how many intersections
        ray_end = create_point(point, self.maximum, 0)
        ray = Segment(point, ray_end)
        intersection_count = 0
        intersections = []
        for segment in self.segments:
            if do_segments_intersect(ray, segment):
                intersection_point = find_segment_intersection(ray, segment)
                if intersection_point is not None:
                    intersection_count += 1
                    intersections.append(intersection_point)

        # If no intersections, point not in the polygon
        if intersection_count == 0:
            return False

        # Filter out duplicate points
        indices_to_remove = []
        for index1, point1 in enumerate(intersections):
            for index2, point2 in enumerate(intersections):
                if index1 != index2 and point1.equals(point2):
                    indices_to_remove.append(index1)

        indices_to_remove.sort()
        for index in indices_to_remove:
            intersections.pop(index)

        # If odd intersection count, point is contained by polygon
        if intersection_count % 2 != 0:
            return True

        # If even intersection count, point is not with polygon
        return False

class Obstacle:
    def __init__(self, boundary=None, tolerance=None):
        self.boundary = boundary  # A polygon class that makes the bounding box of the obstacle
        self.tolerance = tolerance  # The distance in metres the robot is allowed to the bounding box of the obstacle

class Package:
    def __init__(self, colour=None):
        self.colour = colour  # A unique id number for a destination e.g: "red", "green", "blue"
        self.destination_pose = None
        self.return_destination = None
        self.identify_destinations()

    def identify_destinations(self):
        if self.colour == "red":
            self.destination_pose = Pose(x=0.3, y=0.4, theta=math.pi/2)
            self.return_destination = Pose(x=0.2, y=0.9)
        if self.colour == "green":
            self.destination_pose = Pose(x=0.3, y=0.6, theta=math.pi/2)
            self.return_destination = Pose(x=0.6, y=0.9)
        if self.colour == "blue":
            self.destination_pose = Pose(x=0.3, y=0.8, theta=math.pi/2)
            self.return_destination = Pose(x=1, y=0.9)

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
        self.pwm = GPIO.PWM(self.enable_pin, 100)
        self.pwm.start(self.speed)
        self.ticks = 0
        self.encoder_a_state = GPIO.input(self.encoder_a)
        self.encoder_b_state = GPIO.input(self.encoder_b)

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

    def reset_encoder(self):
        self.ticks = 0
        self.encoder_a_state = GPIO.input(self.encoder_a)

    def update_encoder(self):
        new_encoder_a_state = GPIO.input(self.encoder_a)

        if new_encoder_a_state != self.encoder_a_state:
            self.encoder_a_state = new_encoder_a_state
            self.ticks += 1

class ColourSensor:
    def __init__(self, s2=None, s3=None, signal=None):
        self.s2 = s2
        self.s3 = s3
        self.signal = signal
        self.num_of_cycles = 10
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.s2, GPIO.OUT)
        GPIO.setup(self.s3, GPIO.OUT)
        GPIO.setup(self.signal, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        self.ranges = [[28930.91836261603, 21021.067589370134, 25517.98500016965], [21231.786313494456, 23163.507962153875, 28296.01046601533], [17333.979396482864, 20200.26705372916, 23926.500056086454]]  # R G B
        self.colours = ["red", "green", "blue"]
        self.tolerance = 2500
        self.sample_size = 5
        self.minimum_percent = 0.8
        self.minimum_correct = math.ceil(self.sample_size * self.minimum_percent)

    def read_colour(self):
        colour_counts = {"red": 0, "green": 0, "blue": 0}

        for index in range(self.sample_size):
            # Read each colour sensor
            self.read_red()
            red_reading = self.single_reading()
            self.read_green()
            green_reading = self.single_reading()
            self.read_blue()
            blue_reading = self.single_reading()

            readings = [red_reading, green_reading, blue_reading]
            for colour_index, colour_range in enumerate(self.ranges):
                detected_colour = None
                for rbg_val in range(3):
                    difference = readings[rbg_val] - colour_range[rbg_val]
                    if abs(difference) > self.tolerance:
                        detected_colour = "no colour"
                        break
                if detected_colour == "no colour":
                    continue
                detected_colour = self.colours[colour_index]
                colour_counts[detected_colour] += 1
                break

        max_colour = max(colour_counts, key=colour_counts.get)
        if colour_counts[max_colour] >= self.minimum_correct:
            return max_colour
        else:
            return None

    def calibration_call(self):
        # Read each colour sensor
        self.read_red()
        sleep(0.01)
        red_reading = self.single_reading()
        self.read_green()
        sleep(0.01)
        green_reading = self.single_reading()
        self.read_blue()
        sleep(0.01)
        blue_reading = self.single_reading()

        readings = [red_reading, green_reading, blue_reading]
        return readings

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
    
    def __init__(self, trig_pin = None, echo_pin = None, x_offset=None, y_offset=None, theta=None, reading_index=None, maximum_read_distance=None):
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trig_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.theta = theta
        self.maximum_read_distance = maximum_read_distance
        self.reading_index = reading_index
        self.hypot = math.hypot(x_offset, y_offset)
        self.centre_angle = math.atan2(y_offset, x_offset)

    def measure_dist(self):
        GPIO.output(self.trig_pin, True)
        sleep(0.00001)
        GPIO.output(self.trig_pin, False)

        pulse_start = time()
        pulse_end = time()

        initial_time = time()
        while GPIO.input(self.echo_pin) == 0:
            if time() > initial_time + 0.2:
                return None
            pulse_start = time()
            sleep(0.001)

        initial_time = time()

        while GPIO.input(self.echo_pin) == 1:
            if time() > initial_time + 0.2:
                return None
            pulse_end = time()
            sleep(0.001)

        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 171.50  # Speed of sound in m/s

        return distance

class LimitSwitch:
    def __init__(self, distance, switch_pin = None):
        self.distance = distance
        self.triggered = False
        self.pin = switch_pin

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def detect(self):
        if GPIO.input(self.pin) == GPIO.HIGH:
            self.triggered = True
            print("Switch pressed")
        else:
            self.triggered = False
