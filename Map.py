import math

import numpy as np
# import matplotlib.pyplot as plt
from BaseClasses import *
class Map:
    def __init__(self):
        self.obstacle_guess_width = 0.15  # Create an obstacle of 9 cm
        self.obstacle_guess_depth = 0.15  # Create an obstacle of 9 cm
        self.node_gap = 0.050  # 1 cm between each node
        self.map_size = (1.5, 1.5)
        self.x_count = int(math.ceil(self.map_size[0] / self.node_gap))
        self.y_count = int(math.ceil(self.map_size[1] / self.node_gap))
        self.map_grid = np.zeros((self.x_count, self.y_count))
        self.obstacle_polygon = None
        self.path = []

    # def plot_grid(self):
    #     for x_index in range(self.x_count):
    #         for y_index in range(self.y_count):
    #             world_point = Pose(x_index * self.node_gap, y_index * self.node_gap)
    #             value_at_point = self.map_grid[x_index, y_index]
    #             if value_at_point == 0:
    #                 plt.scatter(world_point.x, world_point.y, s=self.node_gap * 100, color="skyblue")
    #             elif value_at_point == 1:
    #                 plt.scatter(world_point.x, world_point.y, s=self.node_gap * 100, color="purple")
    #             elif value_at_point == 2:
    #                 plt.scatter(world_point.x, world_point.y, s=self.node_gap * 100, color="red")
    #             else:
    #                 plt.scatter(world_point.x, world_point.y, s=self.node_gap * 100, color="green")
    #
    #     x_boundary = []
    #     y_boundary = []
    #
    #     if self.obstacle_polygon is not None:
    #         for point in self.obstacle_polygon.vertices:
    #             x_boundary.append(point.x)
    #             y_boundary.append(point.y)
    #         x_boundary.append(self.obstacle_polygon.vertices[0].x)
    #         y_boundary.append(self.obstacle_polygon.vertices[0].y)
    #         plt.plot(x_boundary, y_boundary, color="black")
    #
    #     plt.axis('equal')
    #     plt.show()

    def add_obstacle_to_grid(self, robot_angle=None, collision_point=None):
        perpendicular_angle = robot_angle + np.pi / 2

        # Find four points of bounding box
        bottom_left = Pose(collision_point.x + 0.5 * self.obstacle_guess_width * np.cos(perpendicular_angle + np.pi), collision_point.y + 0.5 * self.obstacle_guess_width * np.sin(perpendicular_angle + np.pi))
        bottom_right = Pose(collision_point.x + 0.5 * self.obstacle_guess_width * np.cos(perpendicular_angle), collision_point.y + 0.5 * self.obstacle_guess_width * np.sin(perpendicular_angle))
        top_left = Pose(bottom_left.x + self.obstacle_guess_depth * np.cos(robot_angle), bottom_left.y + self.obstacle_guess_depth * np.sin(robot_angle))
        top_right = Pose(bottom_right.x + self.obstacle_guess_depth * np.cos(robot_angle), bottom_right.y + self.obstacle_guess_depth * np.sin(robot_angle))

        vertices = [bottom_left, top_left, top_right, bottom_right]
        bounding_box = Polygon(vertices)
        self.obstacle_polygon = bounding_box

        # Check if any point in the map grid overlap with the bounding box
        for x_index in range(self.x_count):
            for y_index in range(self.y_count):
                world_point = Pose(x_index * self.node_gap, y_index * self.node_gap)
                if bounding_box.contains(world_point):
                    self.map_grid[x_index, y_index] = 1

    def plan_path(self, robot_pose: Pose, goal_coordinate: Pose):
        # Create straight line from start to goal
        path_start = robot_pose
        path_end = goal_coordinate

        # Check if a collision will occur
        will_collide = self.check_for_collision([path_start, path_end])

        if not will_collide:
            return [path_start, path_end]

        # Create intermediate points and move them around
        is_solution_found = False
        intermediate_point_count = 1
        updated_points = []
        waypoints = []
        while not is_solution_found:
            intermediate_points, position_array, distance_array = create_intermediate_points(path_start, path_end, intermediate_point_count)
            perpendicular_angle = math.atan2(goal_coordinate.y - robot_pose.y, goal_coordinate.x - robot_pose.x) + math.pi / 2
            while will_collide:
                updated_points = []
                for index in range(intermediate_point_count):
                    current_distance = 0
                    if position_array[index] == 1:
                        current_distance = distance_array[index]
                    elif position_array[index] == -1:
                        current_distance = -distance_array[index]
                    new_point = create_point(intermediate_points[index], current_distance, perpendicular_angle)
                    updated_points.append(new_point)

                # Check if new points collide
                waypoints = [path_start]
                waypoints.extend(updated_points)
                waypoints.append(path_end)
                if self.check_for_collision(waypoints):
                    position_array = increment_base_3_number(position_array)
                    # Check if it has done all permutations
                    is_complete = True
                    for bit in position_array:
                        if bit != 0:
                            is_complete = False
                            break

                    # If done all permutations, increment one to the point count
                    if is_complete:
                        intermediate_point_count += 1
                        break
                    else:
                        continue

                # If they don't collide, break
                is_solution_found = True
                break

        self.path = waypoints
        return waypoints

    def check_for_collision(self, waypoints: list):
        # Check if the given waypoints will collide with any of the 1s in the map grid.
        # Clear the 2s
        for x_index in range(self.x_count):
            for y_index in range(self.y_count):
                if self.map_grid[x_index, y_index] == 2:
                    self.map_grid[x_index, y_index] = 0

        # Add 1 to the value of each node along the path
        for index in range(len(waypoints) - 1):
            point_1 = waypoints[index]
            point_2 = waypoints[index + 1]
            angle_to_point_2 = math.atan2(point_2.y - point_1.y, point_2.x - point_1.x)
            distance_between = calculate_distance_between_points(point_1, point_2)
            number_of_checks = math.ceil((distance_between / self.node_gap) * 1.3)
            distance_per_check = distance_between / number_of_checks

            for check in range(number_of_checks):
                point_to_check = create_point(point_1, distance_per_check * check, angle_to_point_2)
                if self.point_is_out_of_bounds(point_to_check):
                    continue
                node_x, node_y = self.find_closest_node(point_to_check)
                grid_value = self.map_grid[node_x, node_y]

                # Lay out the robot path
                if grid_value == 0:
                    self.map_grid[node_x, node_y] = 2
                elif grid_value == 1:
                    self.map_grid[node_x, node_y] = 3
        # Check every value, if any are greater than 1, then there is a collision
        is_collision = False
        for x_index in range(self.x_count):
            for y_index in range(self.y_count):
                if self.map_grid[x_index, y_index] == 3:
                    is_collision = True
                    self.map_grid[x_index, y_index] = 1

        # If not then there is no collision
        return is_collision

    def point_is_out_of_bounds(self, point):
        if 0 < point.x < self.map_size[0] and 0 < point.y < self.map_size[1]:
            return False
        else:
            return True

    def find_closest_node(self, point: Pose):
        x_coord = int(math.floor(point.x / self.node_gap))
        y_coord = int(math.floor(point.y / self.node_gap))
        return x_coord, y_coord

def increment_base_3_number(input_list: list):
    bits = len(input_list)
    output = input_list

    def update_bit(digit):
        if digit == 0:
            return 1
        elif digit == 1:
            return -1
        else:
            return 0

    carry_flag = True
    for bit in range(bits):
        if carry_flag:
            output[bit] = update_bit(output[bit])
        if output[bit] != 0:
            carry_flag = False

    return output

def create_intermediate_points(start: Pose, end: Pose, number_of_points: int):
    start_end_distance = calculate_distance_between_points(start, end)
    each_point_distance = start_end_distance / (number_of_points + 1)

    start_end_angle = math.atan2(end.y - start.y, end.x - start.x)
    intermediate_points = []
    position_array = []
    distance_array = []
    for index in range(number_of_points):
        current_angled_distance = each_point_distance * (index + 1)
        new_point = create_point(start, current_angled_distance, start_end_angle)
        intermediate_points.append(new_point)
        position_array.append(0)
        distance_array.append(current_angled_distance * 0.5)

    return intermediate_points, position_array, distance_array


if __name__ == "__main__":
    print("START")
    the_map = Map()
    the_map.add_obstacle_to_grid(3 * math.pi / 4, Pose(0.5, 0.5))
    path = the_map.plan_path(Pose(0.010, 0.010, math.pi / 2), Pose(1, 1.25))
    print("PATH FOUND")
    #the_map.plot_grid()