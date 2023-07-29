from BaseClasses.BaseClasses import *
from Map.Map import *
from Robot.Robot import *
from MathFunctions.MathFunctions import *

from threading import Thread

def run_main_logic():
    # Creat class instances
    arena_map = Map()
    start_pose = Pose(1, 3, pi / 2)
    robot = Robot(start_pose)

    # Enter logic loop
    logic_loop(robot=robot, arena_map=arena_map)

def logic_loop(robot=None, arena_map=None):
    # Enter state selector
    robot.state = state_selector(current_state=robot.state)

    # Enter state handler
    state_handler(robot=robot, arena_map=arena_map)

def state_handler(robot=None, arena_map=None):
    if robot.get_state() == "waiting":
        handle_waiting(robot=robot)

    if robot.get_state() == "idle":
        handle_idle(robot=robot)

    if robot.get_state() == "scanning":
        handle_scanning(robot=robot)

    if robot.get_state() == "delivering":
        handle_delivering(robot=robot, arena_map=arena_map)

def handle_waiting(robot=None):
    # TODO
    # If start button has been pressed, change sub-state to "ready"
    robot.set_state(sub_state="ready")

def handle_idle(robot=None):
    if robot.get_sub_state() is None:
        robot.set_state(sub_state="checking")

    if robot.get_sub_state() == "checking":
        # Check the package detecting sensors
        # TODO
        # If package detecting sensors are activated
        # Set the sub-state to "found"
        robot.set_state(sub_state="found")

def handle_scanning(robot=None):
    if robot.get_sub_state() is None:
        robot.set_state(sub_state="scanning")

    if robot.get_sub_state() == "scanning":
        # Do a scan attempt
        # If scan is successful, set the sub-state to "success"
        robot.set_state(sub_state="success")

def handle_delivering(robot=None, arena_map=None):
    if robot.get_sub_state() is None:
        robot.set_state(sub_state="planning")

    if robot.get_sub_state() == "planning":
        # Plan a path with the given obstacles and other parameters
        robot.plan_path(arena_map=arena_map, end_pose=robot.packages[0].destination.deposit_pose)

        # Check if there is a valid path planned
        if robot.path is not None:
            # Start driving to destination
            robot.set_state(sub_state="moving")

    if robot.get_sub_state() == "moving":
        # Tick moving flag
        robot.is_moving = True

        # Start thread to control movement
        moving_thread = Thread(target=robot.follow_path)

def state_selector(current_state=None, current_sub_state=None):
    """
    Determines how the current state affects the next state
    :param current_state: The current state of the robot
    :param current_substate: The current sub-state of the robot if there is one
    :return: The next state of the robot
    """
    # If state is delivering
    if current_state == "delivering":
        # and substate is stuck, replan the route
        if current_substate == "stuck":
            return "planning", None

    # If state is finished, the robot task is complete

def wait_for_start_button():
    # If start button is pressed, break otherwise wait.
    pass

if __name__ == "__main__":
    run_main_logic()