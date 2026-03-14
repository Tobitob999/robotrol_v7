from control.errors import ControlError


class TrajectoryExecutor:
    def __init__(self, robot, gripper, robot_cfg: dict):
        self.robot = robot
        self.gripper = gripper
        self.robot_cfg = robot_cfg

    def execute_pick(self, grasp_plan):
        self.gripper.open()
        self.robot.move_cartesian(grasp_plan.approach_pose, self.robot_cfg.get("approach_speed", 0.0))
        self.robot.move_cartesian(grasp_plan.grasp_pose, self.robot_cfg.get("grasp_speed", 0.0))
        self.gripper.close(self.robot_cfg.get("gripper", {}).get("close_force", 0.0))
        if not self.gripper.is_object_grasped():
            raise ControlError("Grasp failed")
        self.robot.move_cartesian(grasp_plan.lift_pose, self.robot_cfg.get("lift_speed", 0.0))
        return True

    def execute_place(self, place_plan):
        self.robot.move_cartesian(place_plan.approach_pose, self.robot_cfg.get("place_speed", 0.0))
        self.robot.move_cartesian(place_plan.place_pose, self.robot_cfg.get("place_speed", 0.0))
        self.gripper.open()
        self.robot.move_cartesian(place_plan.retreat_pose, self.robot_cfg.get("retract_speed", 0.0))
        return True
