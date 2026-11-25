import pytest

from double_arm_harvest_workflows.leaf_remove import leaf_remove
from double_arm_harvest_workflows.tomato_pick import tomato_pick


EXPECTED_STEPS = (
    ('PREPARE_GRIPPER', 'OPEN_GRIPPER', 0.0),
    ('APPROACH', 'MOVE_TCP', 0.25),
    ('PICK', 'EXTEND_J3', 0.5),
    ('END_EFFECTOR', 'CLOSE_GRIPPER', 0.5),
    ('RETREAT', 'RETRACT_J3', 0.75),
    ('HOME', 'MOVE_HOME', 1.0),
    ('RELEASE_GRIPPER', 'OPEN_GRIPPER', 1.0),
)


def assert_stage_sequence(steps, retreat_delays):
    assert tuple((step.stage, step.skill.name, step.progress) for step in steps) == (
        EXPECTED_STEPS
    )
    assert [step.stage for step in steps if step.use_j3_standoff] == ['APPROACH']
    assert [(step.left_delay_sec, step.right_delay_sec) for step in steps] == [
        retreat_delays if step.stage == 'RETREAT' else (0.0, 0.0)
        for step in steps
    ]


def test_tomato_pick_retracts_both_arms_without_a_stagger():
    assert_stage_sequence(tomato_pick(), (0.0, 0.0))


@pytest.mark.parametrize(
    'execute_left, execute_right, left_role, right_role, retreat_delays',
    [
        (True, True, 'hold', 'operate', (3.0, 0.25)),
        (True, False, 'operate', '', (0.0, 0.0)),
        (False, True, '', 'operate', (0.0, 0.0)),
        (True, True, 'operate', 'operate', (0.0, 0.0)),
    ],
)
def test_leaf_retreat_stagger_applies_only_to_hold_and_operate_pair(
    execute_left, execute_right, left_role, right_role, retreat_delays
):
    steps = leaf_remove(
        execute_left=execute_left,
        execute_right=execute_right,
        left_role=left_role,
        right_role=right_role,
        left_retreat_delay_sec=3.0,
        right_retreat_delay_sec=0.25,
    )
    assert_stage_sequence(steps, retreat_delays)
