"""Corner computation for the square (pure Python, no ROS)."""
import math
from dataclasses import dataclass
from typing import List

from .constants import START_X, START_Y


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    yaw: float


def square_corners(side: float, x0: float = START_X, y0: float = START_Y) -> List[Waypoint]:
    """The 4 goToPose targets of a square whose first corner is (x0, y0).

    The turtle starts at (x0, y0) facing +x (angle 0 after a reset) and walks
    right -> up -> left -> down. At each corner the final yaw already points along
    the next side, so the next goal starts without a long rotation, and the last
    goal restores the original heading (0).
    """
    return [
        Waypoint(x0 + side, y0, math.pi / 2),
        Waypoint(x0 + side, y0 + side, math.pi),
        Waypoint(x0, y0 + side, -math.pi / 2),
        Waypoint(x0, y0, 0.0),
    ]
