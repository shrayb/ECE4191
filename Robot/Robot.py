class Robot:
    def __init__(self, pose=Pose()):
        self.pose = pose

    def set_pose(self, x=None, y=None, theta=None):
        if x is not None:
            self.pose.x = x
        if y is not None:
            self.pose.y = y
        if theta is not None:
            self.pose.theta = theta

    def get_pose(self):
        return self.pose
