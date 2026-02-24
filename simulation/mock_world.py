from control.transforms import make_transform, rotation_matrix_from_quaternion


class MockWorld:
    def __init__(self, base_pose_cfg: dict):
        self.base_pose_cfg = base_pose_cfg

    def pose_for_cycle(self, index: int):
        position = self.base_pose_cfg.get("position_mm", [0.0, 0.0, 0.0])
        quaternion = self.base_pose_cfg.get("quaternion_xyzw", [0.0, 0.0, 0.0, 1.0])
        R = rotation_matrix_from_quaternion(quaternion)
        T = make_transform(R, position)
        return T, position, quaternion
