"""Recipes: a sequence of ramp/hold segments.

A recipe is just a list of segments. Each segment ramps the setpoint from
wherever it currently is up (or down) to ``target`` at ``ramp_rate`` °C/min,
then holds there for ``hold_min`` minutes. This is deliberately simple — no
pattern memory, no soak/path-finding logic.

JSON on disk looks like:

    {
        "name": "to-1100",
        "segments": [
            {"target": 1100, "ramp_rate": 20, "hold_min": 60}
        ]
    }
"""

import json
import os
from dataclasses import asdict, dataclass, field
from typing import List


@dataclass
class Segment:
    target: float          # °C to ramp toward
    ramp_rate: float       # °C/min (magnitude; direction is inferred)
    hold_min: float = 0.0  # minutes to hold once target is reached

    def __post_init__(self):
        self.target = float(self.target)
        self.ramp_rate = abs(float(self.ramp_rate))
        self.hold_min = max(0.0, float(self.hold_min))


@dataclass
class Recipe:
    name: str
    segments: List[Segment] = field(default_factory=list)

    @property
    def final_target(self) -> float:
        return self.segments[-1].target if self.segments else 0.0

    def to_dict(self) -> dict:
        return {"name": self.name, "segments": [asdict(s) for s in self.segments]}

    @classmethod
    def from_dict(cls, data: dict) -> "Recipe":
        return cls(
            name=data.get("name", "recipe"),
            segments=[Segment(**s) for s in data.get("segments", [])],
        )

    def save(self, path: str):
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, indent=2)

    @classmethod
    def load(cls, path: str) -> "Recipe":
        with open(path) as f:
            return cls.from_dict(json.load(f))


def build_simple(target: float, ramp_rate: float, hold_min: float,
                 name: str = "default") -> Recipe:
    """One-segment ramp-and-hold recipe (the common case)."""
    return Recipe(name=name, segments=[Segment(target, ramp_rate, hold_min)])


def list_recipes(recipes_dir: str) -> List[str]:
    """Return sorted paths of *.json recipe files in ``recipes_dir``."""
    if not os.path.isdir(recipes_dir):
        return []
    files = [f for f in os.listdir(recipes_dir) if f.endswith(".json")]
    return [os.path.join(recipes_dir, f) for f in sorted(files)]
