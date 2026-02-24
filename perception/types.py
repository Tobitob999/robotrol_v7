from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class ObjectPose:
    frame_id: str
    parent_frame: str
    matrix: List[List[float]]
    position_mm: Tuple[float, float, float]
    quaternion_xyzw: Tuple[float, float, float, float]
    confidence: float
