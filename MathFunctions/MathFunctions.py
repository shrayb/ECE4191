# Put any math related functions in here such as returning distance between points or angle between three points.
import math

pi = 3.14159265358979

def distance_between_points(point1=None, point2=None):
    return math.sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2)
