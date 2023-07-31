from BaseClasses.BaseClasses import *
from Map.Map import *
from Robot.Robot import *
from MathFunctions.MathFunctions import *

from threading import Thread

# List for storing all threads. This is done in global as we can wait for a thread
# or delete them at any time during execution
# Store threads like this: <name, thread>
#   .append(["movement", Thread(target=robot.do_something)])
# Current thread types:
#   "movement", "scanning", "txrx"
global_thread_list = {}

def run_main_logic():
    # Creat class instances
    arena_map = Map()
    start_pose = Pose(1, 3, pi / 2)
    robot = Robot(start_pose)

    # Initialise extra threads
    global_thread_list["movement"] = None
    global_thread_list["scanning"] = None
    global_thread_list["txrx"] = None

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

    if robot.get_state() == "delivering" or robot.get_state() == "returning":
        handle_delivering_and_returning(robot=robot, arena_map=arena_map)

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
        scan_result = robot.scan_attempt()

        # If scan is successful, set the sub-state to "success"
        if scan_result:
            robot.set_state(sub_state="success")

    if robot.get_sub_state() == "success":
        # Set state to make robot begin delivering the package
        robot.set_state(state="delivering")

def handle_delivering_and_returning(robot=None, arena_map=None):
    if robot.get_sub_state() is None:
        robot.set_state(sub_state="planning")

    if robot.get_sub_state() == "planning":
        # Plan a path with the given obstacles and other parameters
        robot.plan_path(arena_map=arena_map, end_pose=robot.get_current_goal(arena_map=arena_map))

        # Check if there is a valid path planned
        if robot.path is not None:
            # Start driving to destination
            robot.set_state(sub_state="moving")

    if robot.get_sub_state() == "moving":
        # Check if robot isn't moving
        if not robot.is_moving and not robot.movement_complete:
            # Tick moving flag
            robot.is_moving = True

            # Start thread to control movement
            global_thread_list["movement"] = Thread(target=robot.follow_path)
            global_thread_list["movement"].start()
        elif not robot.is_moving and robot.movement_complete:
            # Reset movement complete flag
            robot.movement_complete = False

            # Robot successfully arrived at position
            robot.set_state(sub_state="positioned")

    if robot.get_sub_state() == "stuck":
        # Make a beep sound
        robot.sound(sound="beep")

        # Set robot sub-state to planning once again
        robot.set_state(sub_state="planning")

    if robot.get_sub_state() == "positioned":
        # Set sub-state to depositing
        robot.set_state(sub_state="depositing")

    if robot.get_state() == "delivering" and robot.get_sub_state() == "depositing":
        if not robot.depositing:
            robot.depositing = True
            # Start the package depositing process
            robot.deposit_package()

        if not robot.depositing:
            robot.set_state(sub_state="completed")

    if robot.get_sub_state() == "completed":
        if not robot.scanning_flag:
            # Tick continuous scan flag
            robot.scanning_flag = True

            # Start thread to begin scanning for packages
            global_thread_list["scanning"] = Thread(target=robot.continuous_scan)
            global_thread_list["scanning"].start()

        # Return to pick-up location
        robot.set_state(state="returning")

def wait_for_start_button():
    # If start button is pressed, break otherwise wait.
    pass

if __name__ == "__main__":
    run_main_logic()