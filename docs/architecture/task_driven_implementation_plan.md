# Task-Driven Architecture Implementation Plan

**Goal:** Enable serial, per-goal selection of tomato picking and leaf removal while preserving current motion and hardware behavior.

**Architecture:** Add a single unified task Action and an in-process legacy adapter, then split orchestration, task workflows, and motion/gripper skills inside the current execution package. Add one bringup package that includes the existing stable launch components and mode-specific safety gates. Keep detection data distinct from executable TCP targets.

**Tech Stack:** ROS 2 Jazzy, rclcpp_action, rosidl, MoveIt 2, ros2_control, launch, ament/colcon.

**Spec:** `docs/architecture/task_driven_architecture.md`

## Global Constraints

- Preserve current stage order, seeded IK, J3 strokes, synchronized dispatch, arrival feedback, cancellation, and HOME behavior.
- Preserve the STM32 protocol, streaming implementation, hardware interfaces, and gripper Modbus implementation.
- Keep dual-arm leaf RETREAT timing and skip staggering for single-arm requests.
- No more than one task may own the arms at a time; do not accept a replacement task while downstream goals remain outstanding.
- Real motor and gripper motion default to disabled; readonly launches no command path.
- Do not report physical hardware validation without executing on the connected robot.

## Review Focus

- Old goals with only one enabled arm must not require or reserve the disabled arm's target.
- Cancellation or timeout while a downstream goal is awaiting acceptance must not release the task gate early.
- Switching task types in one process must not leak cancellation, target, stage, feedback, or role state.
- Readonly must remain command-free even when user-provided launch overrides request motion enables.
- Detection poses must never be treated as executable TCP poses without target processing.

---

### Task 1: Unified Action and Legacy Conversion

**Files:**
- Create `src/double_arm_harvest_interfaces/msg/TaskTarget.msg`, `DetectedTarget.msg`, and `action/ExecuteTask.action`.
- Modify `src/double_arm_harvest_interfaces/CMakeLists.txt`.
- Modify `src/double_arm_harvest_execution/src/dual_harvest_executor.cpp` and its header.
- Modify manual sender and add focused conversion/action tests.

**Interfaces:**
- `ExecuteTask.Goal`: task ID/type, left/right executable targets, arm enable flags, arm roles.
- Feedback: task ID/state, current stage, overall and per-arm progress, message.
- Result: task ID, overall/per-arm success, failed stage, error code and message.
- Legacy conversion: existing goal maps to `tomato_pick` unless `legacy_task_type:=leaf`, which maps to `leaf_remove`.

- [x] Add interface-generation assertions and conversion tests for tomato, leaf, left-only, and right-only requests.
- [x] Run test-first checks before adding the new interface and task behavior.
- [x] Add the interfaces without changing `ExecuteDualHarvest.action`.
- [x] Add both action servers and convert each goal to the shared internal request type; keep task execution serialized.
- [x] Add the manual sender task-type option and retain its existing YAML fields and arm switches.
- [x] Build/test the interfaces and execution package; verify legacy and unified requests report expected terminal states.

### Task 2: Extract Workflow and Skill Responsibilities

**Files:**
- Add task executor, tomato-pick task, leaf-remove task, motion skill, gripper skill, and shared task context under `src/double_arm_harvest_execution/include` and `src`.
- Modify executor targets in `src/double_arm_harvest_execution/CMakeLists.txt` and `src/double_arm_harvest_execution/src/main.cpp`.
- Extend unit and fake integration coverage.

**Interfaces:**
- `TaskExecutor` owns Action lifecycle, the single-task gate, the shared stage sequence, and stage dispatch synchronization.
- `TomatoPickTask` and `LeafRemoveTask` provide task-specific behavior policy to the shared sequence, avoiding duplicated operation order.
- `MotionSkill` owns TCP stage-pose generation and MoveIt pose / named-target / J3 planning; dispatch reuses `TrajectoryDispatcher`, `StageBarrier`, and existing failure policy.
- `GripperSkill` owns the existing gripper Action request, timeout, and cancellation behavior.

- [x] Add tests for task strategy selection, stage order, arm roles, task serialization, cancellation, and failure cleanup.
- [x] Run test-first checks for the task strategy and shared skills.
- [x] Keep Action lifecycle / task gate in `TaskExecutor`; extract request parsing, task timing policy, `MotionSkill`, and `GripperSkill` in compilable steps.
- [x] Prove same-process tomato → leaf → tomato; preserve left/right-only and dual-arm execution.
- [x] Prove leaf delays only dual-arm RETREAT by the configured offset and preserves right-first timing.
- [x] Build the package and run its C++ unit and fake integration tests.

### Task 3: Unified Mode-Based Bringup

**Files:**
- Create `src/double_arm_bringup` with `robot.launch.py`, mode-specific include fragments, package metadata, and configuration.
- Modify current harvest launch files to act as compatibility wrappers without task selection.
- Add launch configuration tests.

**Interfaces:**
- `robot.launch.py mode:=fake|readonly|real` selects the environment.
- `allow_motor_enable` and `gripper_allow_motion` default false; readonly forces both false and omits command nodes/controllers.
- Real mode includes the existing STM32/MoveIt streaming launch and existing gripper launch.

- [x] Add launch tests asserting mode mappings, readonly state-only topology, and closed real motion gates.
- [x] Run launch configuration tests before finalizing the unified launch.
- [x] Implement the bringup package using `IncludeLaunchDescription` and existing stable launch components.
- [x] Verify fake integration uses streaming and mock grippers; verify readonly has no motion or gripper command nodes; verify real defaults locked.
- [x] Build all affected packages and run launch tests.

### Task 4: Manual and Perception Target Boundary

**Files:**
- Modify manual target conversion and its tests in `src/double_arm_harvest_execution`.
- Add minimal target conversion validation utilities and message documentation in `src/double_arm_harvest_interfaces`.
- Update architecture and bringup usage docs.

**Interfaces:**
- `DetectedTarget` carries ID/type, perception pose with frame/time, confidence, and source.
- `TaskTarget` carries the separately processed executable TCP pose with frame/time and provenance.
- The manual YAML converter produces `TaskTarget`; no detector, camera driver, or autonomous decision loop is added.

- [x] Test that manual YAML becomes the same executable-target structure sent through `ExecuteTask`.
- [x] Test that detection metadata is represented separately from the executable TCP target type.
- [x] Keep perception-to-TCP conversion outside the executor; no camera or detection algorithm is added.
- [x] Run relevant unit tests, build, and fake integration sequences; physical verification remains untested.

## Verification Commands

- Configure Jazzy: `source /opt/ros/jazzy/setup.bash`.
- Build changed packages and dependencies: `colcon build --packages-up-to double_arm_bringup double_arm_harvest_execution double_arm_end_effector double_arm_sparse_execution`.
- Run focused tests during each task with `colcon test --packages-select <changed-package>` and inspect `colcon test-result --verbose`.
- Run all fake launch integration tests after Tasks 2–4; report them separately from unit tests and physical robot verification.
