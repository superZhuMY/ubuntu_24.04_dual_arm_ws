"""Pick a tomato and release it after returning HOME."""

from .step import Skill, Step


def tomato_pick() -> tuple[Step, ...]:
    return (
        Step(Skill.OPEN_GRIPPER, 'PREPARE_GRIPPER', 0.0),
        Step(Skill.MOVE_TCP, 'APPROACH', 0.25, use_j3_standoff=True),
        Step(Skill.EXTEND_J3, 'PICK', 0.5),
        Step(Skill.CLOSE_GRIPPER, 'END_EFFECTOR', 0.5),
        Step(Skill.RETRACT_J3, 'RETREAT', 0.75),
        Step(Skill.MOVE_HOME, 'HOME', 1.0),
        Step(Skill.OPEN_GRIPPER, 'RELEASE_GRIPPER', 1.0),
    )
