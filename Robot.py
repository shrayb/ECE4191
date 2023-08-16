from BaseClasses import Pose

class Robot:
    def __init__(self, pose=Pose(), state="waiting"):
        self.pose = pose
        self.state = state  # Current state of robot:
        # "waiting": Robot is waiting for the start button to be pressed
        # "idle": Robot is stationary, waiting for package to be placed on top of it
        # "scanning": Package has been placed on robot and now attempts to scan it
        # "delivering": Package is being delivered to a destination
        # "returning": Robot is returning to package pickup location
        # "finished": Robot has delivered all packages
        self.sub_state = None  # Sub-state of robots current state
        # Sub-states of "waiting":
        #   "waiting": Robot is waiting for user to press start button on the robot.
        #   "ready": User has pressed the start button on the robot
        # Sub-states of "idle":
        #   "checking": Robot is checking the package detecting sensors
        #   "found": Robot has found a package on top of the scanner
        # Sub-states of "scanning":
        #   "scanning": A scanning attempt is being made
        #   "success": Scanning succeeded and tag has been decoded
        self.scanning_flag = False
        # Sub-states of "delivering" and "returning":
        #   "planning": Robot is planning its route through an arena. This will happen at the beginning and anytime an obstacle interferes with the current route
        #   "moving": Robot is moving along its planned route
        self.is_close_to_crashing = False  # Goes True if the robot detects something in front while moving
        self.is_moving = False  # Boolean for if the robot is in transit
        #   "stuck": Robot is stuck and has nowhere to travel
        #   "positioned": Robot has arrived to its destination
        #   "completed": Robot has completed the state task
        # Sub-states of "delivering"
        #   "depositing": The robot is depositing the package into the destination
        self.path = None  # Path class that holds all the information about the robots current path
        self.packages = None  # List of packages and their destinations in order of how they are placed. index 0 is the first package
        self.depositing = False
        self.left_motor = None  # Motor class for the left motor
        self.right_motor = None  # Motor class for the right motor
        self.conveyor_motor = None  # Motor class for the conveyor belt motor

    def get_current_goal(self, arena_map=None):
        if self.packages is not None:
            return self.packages[0].destination.deposit_pose
        else:
            return arena_map.pickup_location

    def continuous_scan(self):
        # As long as state is returning, attempt scanning
        while self.state == "returning" and self.scanning_flag:
            # Do scan attempt
            scan_result = self.scan_attempt()

            # If scan result is False scan again
            if not scan_result:
                continue

            # If scan is successful,

    def scan_attempt(self):
        """
        Makes a scan attempt using the RF scanner, if a package is detected return True, otherwise return False
        """
        # Do one attempt to scan the RF tag of a potential package
        # TODO
        pass

    def follow_path(self):
        # As long as the is_moving flag is True
        while self.is_moving:
            # Make required moves to follow path
            # TODO
            pass
        # When the is_moving flag is False for any reason, stop the robot's movement
        # TODO

    def deposit_package(self):
        # Deposit the next package
        # TODO
        pass

    def sound(self, sound=None):
        if sound == "beep":
            # Play beep sound
            # TODO
            pass

    def plan_path(self, arena_map=None):
        # TODO
        pass

    def handle_txrx(self):
        # Transmits and receives data to and from other robots
        # TODO
        pass

    def set_state(self, state=None, sub_state=None):
        if state is not None:
            self.state = state
            self.sub_state = sub_state

        if state is None and sub_state is not None:
            self.sub_state = sub_state

    def get_state(self):
        return self.state

    def get_sub_state(self):
        return self.sub_state

    def set_pose(self, x=None, y=None, theta=None):
        if x is not None:
            self.pose.x = x
        if y is not None:
            self.pose.y = y
        if theta is not None:
            self.pose.theta = theta

    def get_pose(self):
        return self.pose