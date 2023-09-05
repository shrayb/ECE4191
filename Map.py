import math

import matplotlib.pyplot as plt
from BaseClasses import *
from copy import deepcopy
import numpy as np

class Map:
    def __init__(self):
        self.obstacle_guess_width = 0.20  # Create an obstacle of 20 cm wide
        self.obstacle_guess_depth = 0.20  # Create an obstacle of 20 cm deep
        self.node_gap = 0.050  # Metres between each node
        self.map_size = (1.5, 1.5)
        self.x_count = int(math.ceil(self.map_size[0] / self.node_gap))
        self.y_count = int(math.ceil(self.map_size[1] / self.node_gap))
        self.map_grid = np.zeros((self.x_count, self.y_count))
        self.obstacle_polygon = None
        self.path = []
        self.robot_size = 0.3  # Metres diameter

    def plot_grid(self):
        free_space_x = []
        free_space_y = []

        drive_x = []
        drive_y = []

        obstacle_x = []
        obstacle_y = []

        collision_x = []
        collision_y = []

        x_boundary = []
        y_boundary = []

        for x_index in range(self.x_count):
            for y_index in range(self.y_count):
                value_at_point = self.map_grid[x_index, y_index]
                if value_at_point == 0:
                    free_space_x.append(x_index / self.x_count * self.map_size[0])
                    free_space_y.append(y_index / self.y_count * self.map_size[1])
                elif value_at_point == 1:
                    drive_x.append(x_index / self.x_count * self.map_size[0])
                    drive_y.append(y_index / self.y_count * self.map_size[1])
                elif value_at_point == 2:
                    obstacle_x.append(x_index / self.x_count * self.map_size[0])
                    obstacle_y.append(y_index / self.y_count * self.map_size[1])
                else:
                    collision_x.append(x_index / self.x_count * self.map_size[0])
                    collision_y.append(y_index / self.y_count * self.map_size[1])

        if self.obstacle_polygon is not None:
            for point in self.obstacle_polygon.vertices:
                x_boundary.append(point.x)
                y_boundary.append(point.y)
            x_boundary.append(self.obstacle_polygon.vertices[0].x)
            y_boundary.append(self.obstacle_polygon.vertices[0].y)
            plt.plot(x_boundary, y_boundary, color="black")

        plt.scatter(free_space_x, free_space_y, s=self.node_gap * 100, color="skyblue")
        plt.scatter(drive_x, drive_y, s=self.node_gap * 100, color="purple")
        plt.scatter(obstacle_x, obstacle_y, s=self.node_gap * 100, color="red")
        plt.scatter(collision_x, collision_y, s=self.node_gap * 100, color="green")

        plt.axis('equal')
        plt.show()

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
        bot_left_x, bot_left_y = self.find_closest_node(self.obstacle_polygon.vertices[0])
        top_left_x, top_left_y = self.find_closest_node(self.obstacle_polygon.vertices[1])
        top_right_x, top_right_y = self.find_closest_node(self.obstacle_polygon.vertices[2])
        bot_right_x, bot_right_y = self.find_closest_node(self.obstacle_polygon.vertices[3])

        lowest_x = min([bot_left_x, top_left_x, top_right_x, bot_right_x])
        highest_x = max([bot_left_x, top_left_x, top_right_x, bot_right_x])
        lowest_y = min([bot_left_y, top_left_y, top_right_y, bot_right_y])
        highest_y = max([bot_left_y, top_left_y, top_right_y, bot_right_y])

        # Check the points around the boundary if they are in the obstacle
        for x_index in range(lowest_x, highest_x + 1):
            x_index = max(0, min(x_index, self.x_count))
            for y_index in range(lowest_y, highest_y + 1):
                y_index = max(0, min(y_index, self.y_count))
                world_point = Pose(x_index * self.node_gap, y_index * self.node_gap)
                if bounding_box.contains(world_point):
                    self.map_grid[x_index, y_index] = 1
        # Check if any point in the map grid overlap with the bounding box
        # # TODO I think this is slow as hell, try to make it faster somehow
        # print(self.obstacle_polygon.vertices[0].x, self.obstacle_polygon.vertices[0].y)
        # print(self.obstacle_polygon.vertices[1].x, self.obstacle_polygon.vertices[1].y)
        # print(self.obstacle_polygon.vertices[2].x, self.obstacle_polygon.vertices[2].y)
        # print(self.obstacle_polygon.vertices[3].x, self.obstacle_polygon.vertices[3].y)
        # for x_index in range(self.x_count):
        #     for y_index in range(self.y_count):
        #         world_point = Pose(x_index * self.node_gap, y_index * self.node_gap)
        #         if bounding_box.contains(world_point):
        #             self.map_grid[x_index, y_index] = 1


    def plan_path(self, robot_pose: Pose, goal_coordinate: Pose):
        print("Creating path from:", robot_pose.x, robot_pose.y, "| to:", goal_coordinate.x, goal_coordinate.y)
        # Create straight line from start to goal
        path_start = robot_pose
        path_end = goal_coordinate

        # Check if a collision will occur
        will_collide = self.check_for_collision([path_end], robot_pose)
        print("Will it collide:", will_collide)
        if not will_collide:
            self.path = [path_end]
            return self.path

        # Create intermediate points and move them around
        is_solution_found = False
        intermediate_point_count = 1
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

                    # Make sure the new intermediate point doesn't go outside the bounds of the map
                    if new_point.x < 0 + self.robot_size / 2 + 0.05:
                        new_point.x = self.robot_size / 2 + 0.05
                    if new_point.x > self.map_size[0] - self.robot_size / 2 - 0.05:
                        new_point.x = self.map_size[0] - self.robot_size / 2 - 0.05
                    if new_point.y < 0 + self.robot_size / 2 + 0.05:
                        new_point.y = self.robot_size / 2 + 0.05
                    if new_point.y > self.map_size[1] - self.robot_size / 2 - 0.05:
                        new_point.y = self.map_size[1] - self.robot_size / 2 - 0.05

                    updated_points.append(new_point)

                # Check if new points collide
                waypoints = updated_points
                waypoints.append(path_end)
                if self.check_for_collision(waypoints, robot_pose):
                    position_array = increment_base_3_number(position_array)

                    # Check if it has done all permutations
                    bit_combinations_complete = True
                    for bit in position_array:
                        if bit != 0:
                            bit_combinations_complete = False
                            break

                    if not bit_combinations_complete:
                        continue

                    # If done all permutations, increment one to the point count
                    intermediate_point_count += 1
                    break

                # If they don't collide, break. That is the solution
                print("Solution Found:")
                for point in waypoints:
                    print("\t", point.x, point.y)
                is_solution_found = True
                break

        # Pop first point as it is where you will start
        print("Exited path planning while loop")
        waypoints.pop(0)

        self.path = waypoints
        return self.path

    def check_for_collision(self, waypoints: list, robot_pose: Pose):
        if len(waypoints) == 0:
            return None

        # Clear the drive 2s from the previous attempt
        for x_index in range(self.x_count):
            for y_index in range(self.y_count):
                if self.map_grid[x_index, y_index] == 2:
                    self.map_grid[x_index, y_index] = 0

        copy_waypoints = [robot_pose]
        copy_waypoints.extend(waypoints)

        # Add 1 to the value of each node along the path
        for index in range(len(copy_waypoints) - 1):
            point_1 = copy_waypoints[index]
            point_2 = copy_waypoints[index + 1]

            # If the two points of a segment equal each other, continue to next iteration
            if point_1.equals(point_2):
                return True

            # Find how often to check along the drive path to fill in nodes on grid
            angle_to_point_2 = math.atan2(point_2.y - point_1.y, point_2.x - point_1.x)
            distance_between = calculate_distance_between_points(point_1, point_2)
            number_of_checks = math.ceil((distance_between / self.node_gap) * 1.3)
            distance_per_check = distance_between / number_of_checks

            # Fill in drive path
            for check in range(number_of_checks):
                point_to_check = create_point(point_1, distance_per_check * check, angle_to_point_2)
                if self.point_is_out_of_bounds(point_to_check):
                    continue
                node_x, node_y = self.find_closest_node(point_to_check)
                grid_value = self.map_grid[node_x, node_y]

                # Lay out the robot path. Put a 2 if that's somewhere the robot will drive. Put a 3 if there is an obstacle where we want to drive
                if grid_value == 0:
                    self.map_grid[node_x, node_y] = 2
                elif grid_value == 1:
                    self.map_grid[node_x, node_y] = 3

        # Check every value, if there is 3, there is a collision. Make sure to remove the 3 though for next time
        is_collision = False
        for x_index in range(self.x_count):
            for y_index in range(self.y_count):
                if self.map_grid[x_index, y_index] == 3:
                    is_collision = True
                    self.map_grid[x_index, y_index] = 1

        # If not then there is no collision
        return is_collision

        # 0 is free
        # 1 is obstacle
        # 2 is drive
        # 3 is collision

    def point_is_out_of_bounds(self, point):
        if 0 <= point.x <= self.map_size[0] and 0 <= point.y <= self.map_size[1]:
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
        distance_array.append(current_angled_distance * 0.8)

    return intermediate_points, position_array, distance_array


if __name__ == "__main__":
    print("START")
    the_map = Map()
    the_map.add_obstacle_to_grid(3 * math.pi / 4, Pose(0.5, 0.5))
    the_map.add_obstacle_to_grid(3 * math.pi / 4, Pose(3 / 30, 20 / 30))
    the_map.add_obstacle_to_grid(3 * math.pi / 4, Pose(15 / 30, 2.5 / 30))
    the_map.add_obstacle_to_grid(3 * math.pi / 4, Pose(5 / 30, 11 / 30))
    the_map.add_obstacle_to_grid(3 * math.pi / 4, Pose(5 / 30, 38 / 30))
    path = the_map.plan_path(Pose(0.010, 0.010, math.pi / 2), Pose(1, 1.25))
    print("PATH FOUND")
    the_map.plot_grid()
