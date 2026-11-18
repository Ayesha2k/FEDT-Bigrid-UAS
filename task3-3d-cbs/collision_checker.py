"""3D collision and downwash checks for multi-drone planning."""

from dataclasses import dataclass
from math import dist, hypot
from typing import Sequence


@dataclass(frozen=True)
class SafetyConfig:
    collision_distance: float = 0.25
    downwash_horizontal_radius: float = 0.40
    downwash_vertical_threshold: float = 0.60

    def __post_init__(self):
        if (
            self.collision_distance <= 0
            or self.downwash_horizontal_radius <= 0
            or self.downwash_vertical_threshold <= 0
        ):
            raise ValueError("Safety distances must be positive")


def check_conflict(
    position_a: Sequence[float],
    position_b: Sequence[float],
    config: SafetyConfig = SafetyConfig(),
):
    """Return conflict information, or None if positions are safe."""

    if len(position_a) != 3 or len(position_b) != 3:
        raise ValueError("Positions must contain [x, y, z]")

    distance_3d = dist(position_a, position_b)

    horizontal_distance = hypot(
        position_a[0] - position_b[0],
        position_a[1] - position_b[1],
    )

    vertical_distance = abs(position_a[2] - position_b[2])

    if distance_3d < config.collision_distance:
        return {
            "type": "collision",
            "distance_3d": distance_3d,
        }

    if (
        vertical_distance < config.downwash_vertical_threshold
        and horizontal_distance < config.downwash_horizontal_radius
    ):
        return {
            "type": "downwash",
            "horizontal_distance": horizontal_distance,
            "vertical_distance": vertical_distance,
        }

    return None


if __name__ == "__main__":
    cfg = SafetyConfig()

    examples = [
        ("Safe separation", [0, 0, 1], [1, 0, 1]),
        ("Physical collision", [0, 0, 1], [0.1, 0, 1]),
        ("Downwash conflict", [0, 0, 1], [0.3, 0, 1.2]),
        ("Vertically separated", [0, 0, 1], [0, 0, 1.7]),
    ]

    for name, a, b in examples:
        print(f"{name}: {check_conflict(a, b, cfg)}")
