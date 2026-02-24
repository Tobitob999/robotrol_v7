from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass
class BoardDetection:
    found: bool
    homography: Optional[List[List[float]]]
    warp_image: Optional[object]
    corners: Optional[List[Tuple[float, float]]]


@dataclass
class PieceDetection:
    board: List[List[str]]
    confidence: float


@dataclass
class ChessState:
    board: List[List[str]]
    fen: str
    confidence: float
