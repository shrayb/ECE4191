import Robot.Robot as Robot

pi = 3.14159265358979

class Navigation:
    def __init__(self, robot=None):
        self.robot = robot

    def drive_to_coordinates(self, x=None, y=None, angle=None):
        """
        Drives the robot to the coordinates specified.
        :param x: x coordinate of desired location
        :param y: y coordinate of desired location
        :param angle: Angle the robot must face when the function stops running
        :return: True when completed successfully or False when failed
        """


    def check_for_obstacle(self):
        pass

def run_program():
    new_pose = Robot.Pose(1, 3, pi / 2)
    new_robot = Robot.Robot(new_pose)
    navigation = Navigation(new_robot)

if __name__ == "__main__":
    run_program()