import numpy as np
from BaseClasses import Pose, Polygon
class Map:
    def __init__(self, boundary=None, obstacles=None, destinations=None, pickup_location=None):
        self.boundary = boundary
        self.obstacles = obstacles
        self.destinations = destinations
        self.pickup_location = pickup_location
        self.map_grid = np.zeros(150, 150)
        self.obstacle_guess_width = 0.09  # Create an obstacle of 10 cm
        self.obstacle_guess_depth = 0.09  # Create an obstacle of 10 cm

    def add_obstacle_to_grid(self, robot_angle=None, collision_point=None):
        perpendicular_angle = robot_angle + np.pi / 2

        # Find four points of bounding box
        bottom_left = Pose(collision_point.x + 0.5 * self.obstacle_guess_width * np.cos(perpendicular_angle + np.pi), collision_point.y + 0.5 * self.obstacle_guess_width * np.sin(perpendicular_angle + np.pi))
        bottom_right = Pose(collision_point.x + 0.5 * self.obstacle_guess_width * np.cos(perpendicular_angle), collision_point.y + 0.5 * self.obstacle_guess_width * np.sin(perpendicular_angle))
        top_left = Pose(bottom_left.x + self.obstacle_guess_depth * np.cos(robot_angle), bottom_left.y + self.obstacle_guess_depth * np.sin(robot_angle))
        top_right = Pose(bottom_right.x + self.obstacle_guess_depth * np.cos(robot_angle), bottom_right.y + self.obstacle_guess_depth * np.sin(robot_angle))

        vertices = [bottom_left, bottom_right, top_left, top_right]
        bounding_box = Polygon(vertices)

        for point in self.map_grid:

