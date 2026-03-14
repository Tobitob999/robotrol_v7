from dataclasses import dataclass
from typing import List


@dataclass
class GraspPlan:
    approach_pose: List[List[float]]
    grasp_pose: List[List[float]]
    lift_pose: List[List[float]]


@dataclass
class PlacePlan:
    approach_pose: List[List[float]]
    place_pose: List[List[float]]
    retreat_pose: List[List[float]]
