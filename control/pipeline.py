from dataclasses import dataclass

from control.fsm import PickPlaceStateMachine


@dataclass
class PipelineContext:
    perception: object
    grasp_planner: object
    place_planner: object
    executor: object
    robot: object
    gripper: object
    stack_index: int = 0
    object_pose: object = None
    grasp_plan: object = None
    place_plan: object = None

    def detect(self):
        self.object_pose = self.perception.detect_object_pose()

    def plan_grasp(self):
        self.grasp_plan = self.grasp_planner.plan(self.object_pose)

    def execute_pick(self):
        self.executor.execute_pick(self.grasp_plan)

    def plan_place(self):
        self.place_plan = self.place_planner.plan(self.stack_index)

    def execute_place(self):
        self.executor.execute_place(self.place_plan)
        self.stack_index += 1

    def safe_stop(self):
        self.robot.stop()
        self.gripper.open()


class PickPlacePipeline:
    def __init__(self, perception, grasp_planner, place_planner, executor, robot, gripper):
        self.context = PipelineContext(
            perception=perception,
            grasp_planner=grasp_planner,
            place_planner=place_planner,
            executor=executor,
            robot=robot,
            gripper=gripper,
        )
        self.state_machine = PickPlaceStateMachine()

    def run_cycle(self) -> bool:
        return self.state_machine.run_cycle(self.context)
