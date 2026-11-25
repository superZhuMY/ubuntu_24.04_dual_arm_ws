from dataclasses import dataclass
from enum import Enum, auto


class Skill(Enum):
    OPEN_GRIPPER = auto()
    CLOSE_GRIPPER = auto()
    MOVE_TCP = auto()
    EXTEND_J3 = auto()
    RETRACT_J3 = auto()
    MOVE_HOME = auto()


@dataclass(frozen=True)
class Step:
    skill: Skill
    stage: str
    progress: float
    use_j3_standoff: bool = False
    left_delay_sec: float = 0.0
    right_delay_sec: float = 0.0
    execute_left: bool = True
    execute_right: bool = True
