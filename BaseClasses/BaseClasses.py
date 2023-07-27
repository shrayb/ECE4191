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
    def __init__(self, uid=None, x=None, y=None):
        self.uid = uid  # A unique id number for a destination e.g: 1, 2, or 3
        self.x = x  # x coordinate of destination
        self.y = y  # y coordinate of destination

class Obstacle:
    def __init__(self, boundary=None, tolerance=None):
        self.boundary = boundary  # A polygon class that makes the bounding box of the obstacle
        self.tolerance = tolerance  # The distance in metres the robot is allowed to the bounding box of the obstacle

class Path:
    def __init__(self):
        self.waypoint_queue = None
