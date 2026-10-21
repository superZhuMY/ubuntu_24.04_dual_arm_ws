# Task-Driven Dual-Arm Architecture

## Goal

Move task selection from launch-time configuration to each action goal while preserving the verified MoveIt planning, synchronized FollowJointTrajectory dispatch, feedback-gated streaming executor, STM32 protocol, and gripper controller.

## Runtime architecture

One task node serves `/execute_task` and the temporary `/execute_dual_harvest` compatibility endpoint. Both convert requests to one internal task request and share the same exclusive execution gate, cancellation path, task state, motion skill, and gripper skill. `TomatoPickTask` and `LeafRemoveTask` provide task-specific timing policy to one shared stage sequence; this avoids duplicating the identical pick / leaf operation order. Leaf mode retains its dual-arm delayed RETREAT.

`MotionSkill` owns TCP offset handling and MoveIt pose / named-target / J3 planning. `GripperSkill` owns the existing gripper Action request, timeout, and cancellation wait. `TaskExecutor` retains the synchronized trajectory dispatch, `StageBarrier`, and failure lifecycle so the stage gate and cancellation recovery remain coordinated.

Launch supplies the environment only. `double_arm_bringup/robot.launch.py` assembles existing components for `fake`, `readonly`, or `real`; real motor and gripper motion require explicit enables. The task executor never chooses tasks from launch parameters.

## Interfaces

`TaskTarget` represents an executable TCP pose and carries target ID, object type, stamped pose, confidence, and source. A separate detection target represents a perception pose and cannot be sent directly as an executable TCP goal. `ExecuteTask` carries task ID, task type, per-arm targets, enable flags, and arm roles; its feedback and result report task state, stage, per-arm progress, outcome, and failure reason.

The legacy `ExecuteDualHarvest` schema remains unchanged. `LegacyAdapter` maps it to `tomato_pick` by default or `leaf_remove` when `legacy_task_type:=leaf`; the new action always takes its task type from each goal.

First-version roles preserve current behavior: tomato-pick enabled arms operate; dual-arm leaf removal uses left hold and right operate. In the current choreography, `hold` is the arm whose RETREAT starts after the operating arm, while both enabled arms retain the existing approach, J3 stroke, gripper, and HOME stages. This makes role selection part of the leaf retreat policy without changing the verified action order. Unsupported role combinations are rejected. Tasks execute serially; the two arms may execute a motion stage concurrently.

## Behavior and safety constraints

- Preserve stage order: validate, open, approach, J3 pick stroke, close, J3 retreat, HOME, release.
- Preserve single seeded IK, J3-only strokes, current feedback checks, MoveIt planning, shared-start dispatch, `TrajectoryDispatcher`, `StageBarrier`, and `FailurePolicy` utilities.
- Leaf RETREAT keeps the existing right-first / left-delayed behavior; single-arm execution is never staggered.
- Cancellation and failure stop stage advancement and cancel outstanding downstream goals. Do not automatically retry, return HOME, or open the gripper after a failure.
- Keep the task gate closed until downstream goals have reached terminal results; report timeout/recovery-required if cancellation is not confirmed.
- Keep fake, readonly, and real launch topologies distinct. Readonly starts no motion controllers, task executor, or gripper command server. Real starts with motor and gripper enables disabled.
- Keep vision, policy, reinforcement learning, and agents outside the executor. Future policy clients submit `ExecuteTask` goals and consume results.

## Phased acceptance

1. Add the unified action and legacy conversion, proving each action selects and reports the correct task.
2. Extract the executor, task workflows, and shared skills without changing stage order or synchronization.
3. Add the unified bringup entry and verify mode-specific node composition and safety parameters.
4. Convert manual YAML through the common executable-target type and add a minimal perception-target type and conversion boundary.

Acceptance distinguishes static checks, successful build, unit tests, fake integration tests, and physical robot verification. No physical verification is claimed without connected hardware and recorded execution.
