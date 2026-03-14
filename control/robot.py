class RobotInterface:
    def move_cartesian(self, pose, speed: float):
        raise NotImplementedError

    def move_joint(self, joints):
        raise NotImplementedError

    def stop(self):
        raise NotImplementedError


class MockRobot(RobotInterface):
    def __init__(self):
        self.last_pose = None
        self.history = []
        self.stopped = False

    def move_cartesian(self, pose, speed: float):
        self.last_pose = pose
        self.history.append(("cartesian", pose, float(speed)))
        return True

    def move_joint(self, joints):
        self.history.append(("joint", list(joints), None))
        return True

    def stop(self):
        self.stopped = True
        self.history.append(("stop", None, None))
        return True
