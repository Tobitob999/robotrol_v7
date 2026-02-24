class GripperInterface:
    def open(self):
        raise NotImplementedError

    def close(self, force: float):
        raise NotImplementedError

    def is_object_grasped(self) -> bool:
        raise NotImplementedError


class MockGripper(GripperInterface):
    def __init__(self, config: dict):
        self.config = config
        self.closed = False
        self.object_grasped = False

    def open(self):
        self.closed = False
        self.object_grasped = False
        return True

    def close(self, force: float):
        self.closed = True
        self.object_grasped = bool(self.config.get("mock_grasped", True))
        return True

    def is_object_grasped(self) -> bool:
        return self.object_grasped
