from control.transforms import (
    make_transform,
    extract_translation,
    extract_rotation,
    normalize_vector,
    cross,
)
from planning.types import GraspPlan
from control.errors import PlanningError


AXIS_VECTORS = {
    "x": [1.0, 0.0, 0.0],
    "y": [0.0, 1.0, 0.0],
    "z": [0.0, 0.0, 1.0],
}


def _axis_from_config(axis_name: str, sign: int):
    axis = AXIS_VECTORS.get((axis_name or "z").lower())
    if axis is None:
        axis = AXIS_VECTORS["z"]
    return [axis[0] * sign, axis[1] * sign, axis[2] * sign]


def _build_orientation_from_normal(normal):
    z_axis = normalize_vector(normal)
    if z_axis is None:
        raise PlanningError("Invalid grasp normal")
    ref = [0.0, 0.0, 1.0]
    if abs(z_axis[2]) > 0.95:
        ref = [1.0, 0.0, 0.0]
    x_axis = normalize_vector(cross(ref, z_axis))
    if x_axis is None:
        raise PlanningError("Failed to build grasp orientation")
    y_axis = cross(z_axis, x_axis)
    return [
        [x_axis[0], y_axis[0], z_axis[0]],
        [x_axis[1], y_axis[1], z_axis[1]],
        [x_axis[2], y_axis[2], z_axis[2]],
    ]


class GraspPlanner:
    def __init__(self, cube_cfg: dict):
        self.cube_cfg = cube_cfg

    def plan(self, object_pose) -> GraspPlan:
        obj_T = object_pose.matrix
        obj_pos = extract_translation(obj_T)
        obj_R = extract_rotation(obj_T)

        axis_name = self.cube_cfg.get("grasp_normal_axis", "z")
        sign = int(self.cube_cfg.get("grasp_normal_sign", 1))
        normal_obj = _axis_from_config(axis_name, sign)
        normal_base = [
            obj_R[0][0] * normal_obj[0] + obj_R[0][1] * normal_obj[1] + obj_R[0][2] * normal_obj[2],
            obj_R[1][0] * normal_obj[0] + obj_R[1][1] * normal_obj[1] + obj_R[1][2] * normal_obj[2],
            obj_R[2][0] * normal_obj[0] + obj_R[2][1] * normal_obj[1] + obj_R[2][2] * normal_obj[2],
        ]
        normal_base = normalize_vector(normal_base)
        if normal_base is None:
            raise PlanningError("Normal vector could not be normalized")

        grasp_offset = float(self.cube_cfg.get("grasp_offset_mm", 0.0))
        approach_dist = float(self.cube_cfg.get("approach_distance_mm", 0.0))
        lift_dist = float(self.cube_cfg.get("lift_distance_mm", 0.0))

        grasp_pos = [
            obj_pos[0] + normal_base[0] * grasp_offset,
            obj_pos[1] + normal_base[1] * grasp_offset,
            obj_pos[2] + normal_base[2] * grasp_offset,
        ]
        approach_pos = [
            grasp_pos[0] + normal_base[0] * approach_dist,
            grasp_pos[1] + normal_base[1] * approach_dist,
            grasp_pos[2] + normal_base[2] * approach_dist,
        ]
        lift_pos = [
            grasp_pos[0],
            grasp_pos[1],
            grasp_pos[2] + lift_dist,
        ]

        R = _build_orientation_from_normal(normal_base)
        approach_T = make_transform(R, approach_pos)
        grasp_T = make_transform(R, grasp_pos)
        lift_T = make_transform(R, lift_pos)

        return GraspPlan(
            approach_pose=approach_T,
            grasp_pose=grasp_T,
            lift_pose=lift_T,
        )
