from control.transforms import make_transform
from planning.types import PlacePlan


class PlacePlanner:
    def __init__(self, grid_cfg: dict, cube_cfg: dict):
        self.grid_cfg = grid_cfg
        self.cube_cfg = cube_cfg

    def plan(self, index: int) -> PlacePlan:
        rows = int(self.grid_cfg.get("rows", 1))
        cols = int(self.grid_cfg.get("cols", 1))
        spacing = self.grid_cfg.get("spacing_mm", [0.0, 0.0])
        origin = self.grid_cfg.get("origin_mm", [0.0, 0.0, 0.0])
        edge = float(self.cube_cfg.get("edge_length_mm", 0.0))
        approach_dist = float(self.cube_cfg.get("approach_distance_mm", 0.0))
        lift_dist = float(self.cube_cfg.get("lift_distance_mm", 0.0))

        if rows <= 0 or cols <= 0:
            raise ValueError("Invalid grid dimensions")

        layer = index // (rows * cols)
        slot = index % (rows * cols)
        row = slot // cols
        col = slot % cols

        x = float(origin[0]) + float(spacing[0]) * col
        y = float(origin[1]) + float(spacing[1]) * row
        z = float(origin[2]) + edge * layer

        R = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]

        place_T = make_transform(R, [x, y, z])
        approach_T = make_transform(R, [x, y, z + approach_dist])
        retreat_T = make_transform(R, [x, y, z + lift_dist])

        return PlacePlan(
            approach_pose=approach_T,
            place_pose=place_T,
            retreat_pose=retreat_T,
        )
