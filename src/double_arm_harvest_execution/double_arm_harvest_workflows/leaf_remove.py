"""Remove a leaf with a staggered retreat for the two-arm hold/operate pair."""

from .step import Skill, Step


def leaf_remove(
    *,
    execute_left: bool,
    execute_right: bool,
    left_role: str,
    right_role: str,
    left_retreat_delay_sec: float,
    right_retreat_delay_sec: float,
) -> tuple[Step, ...]:
    stagger_retreat = (
        execute_left and execute_right and left_role == 'hold' and right_role == 'operate'
    )
    return (
        Step(Skill.OPEN_GRIPPER, 'PREPARE_GRIPPER', 0.0),
        Step(Skill.MOVE_TCP, 'APPROACH', 0.25, use_j3_standoff=True),
        Step(Skill.EXTEND_J3, 'PICK', 0.5),
        Step(Skill.CLOSE_GRIPPER, 'END_EFFECTOR', 0.5),
        Step(
            Skill.RETRACT_J3,
            'RETREAT',
            0.75,
            left_delay_sec=left_retreat_delay_sec if stagger_retreat else 0.0,
            right_delay_sec=right_retreat_delay_sec if stagger_retreat else 0.0,
        ),
        Step(Skill.MOVE_HOME, 'HOME', 1.0),
        Step(Skill.OPEN_GRIPPER, 'RELEASE_GRIPPER', 1.0),
    )
