# Task Architecture Integration Plan

> **For agentic workers:** Use executing-plans to implement this plan task by task, then requesting-code-review for a fresh review before merging.

**Goal:** Integrate the existing task architecture and current hardware improvements without losing either workspace's behavior, then merge the verified result into local main.

**Architecture:** TaskExecutor serves /execute_task and the legacy /execute_dual_harvest endpoint through one exclusive task lifecycle. MotionSkill and GripperSkill own planning and gripper requests; task strategies preserve the existing tomato/leaf choreography. Hardware calibration, raw gripper commands, register reads, and failure-pausing behavior are retained from the hardware workspace.

**Tech Stack:** ROS 2 Jazzy, C++17, rclcpp_action, MoveIt 2, ros2_control, Python launch, ament/colcon.

**Spec:** Existing task design at /home/zmy/.codex/worktrees/b670/ubuntu_24.04_dual_arm_ws/docs/architecture/task_driven_architecture.md, supplemented by the integration decisions below. Copy that design into the integrated tree with the refactor snapshot.

## Current state

- Local and remote main remain at 454c4b5.
- Integration is isolated on codex/integrate-task-architecture in the a48e worktree.
- Baseline fixes are committed as 68b955d: deterministic in-flight snapshot testing, explicit timeout-retry coverage, and optional textures installation. All 15 hardware test targets passed; the two relevant backend tests passed 10 repeated runs.
- af12001 merges the existing committed gripper branch into the integration branch only. It has not been merged into main.
- Hardware and refactor workspaces still hold their original uncommitted changes; those workspaces must not be reset, cleaned, or have their indexes rewritten.

## Global constraints

- Preserve stage order: validate, open, approach, J3 pick stroke, close, J3 retreat, HOME, release.
- Keep task execution serialized while allowing synchronized per-stage arm motion.
- Preserve existing STM32 protocol, units, calibrated directions, current gripper mapping, and enabled-side behavior.
- Real motion gates default false. Readonly starts no task or gripper command server and no motion controllers.
- No automatic replay, retry of a task stage, HOME, or gripper opening after task failure.
- Preserve transport-level ACK retries; they are distinct from replaying a task stage.
- Do not delete branches/worktrees or push to remote as part of this plan. Local main merge is the final step after checks pass.
- Snapshot exclusions are path-specific: repository-root .zcode/, .vscode/, .idea/, .env, .env.*, build/, install/, log/, .git/, .superpowers/, plus generated __pycache__/ and *.pyc files. Never read or copy SSH keys, credential stores, or token files outside the selected source paths. Record any additional exclusion by exact path and reason in the ledger; do not use a blanket "host-local configuration" exclusion.
- YAML, xacro, udev, and launch files listed in the hardware preservation manifest below are versioned product configuration and must be included, even when their values identify this machine's physical wiring.
- Apply each input's delta against common base 8f0462c through a three-way merge into the integration branch. Never replace the integration tree with a source workspace directory or an entire snapshot tree.
- Preserve commits 454c4b5 and 68b955d in integration/main ancestry and preserve their tested behavior after conflict resolution.

## Proposed integration decisions requiring plan confirmation

1. Preserve the latest hardware workspace's PAUSED-on-failure behavior, including VALIDATE failures of an already accepted task as well as planning, motion, and gripper failures. Admission-time rejection remains rejection. Publish unified feedback state PAUSED and the original failed stage; retain the busy gate until task cancellation. Cancel outstanding downstream goals before waiting in PAUSED; downstream recovery gates remain closed until termination is confirmed. Do not resume or re-run a relative J3 stroke automatically.
2. A task canceled with unconfirmed downstream termination reports recovery-required and keeps subsequent tasks blocked. Only a terminal result or explicit goal rejection releases the corresponding downstream recovery gate.
3. Keep /execute_dual_harvest schema compatible. New task selection belongs to each /execute_task goal; retain the documented legacy task-type mapping.
4. Keep the hardware workspace's raw gripper controls, register-read service, current physical port/register/calibration settings, and manual sender loop-stop behavior. Rebuild all interface consumers together.
5. Keep docs/vision_pipeline as ROS1 reference material only; do not register it as a ROS2 package or start it from bringup.

## Review focus

- A late acceptance/result after cancellation must not start motion unnoticed or clear a newer task's gate.
- A disabled arm may have an empty target and must not reserve an ID or receive commands.
- A failed or paused task must not advance, replay, return HOME, or release the gripper automatically.
- PAUSED coverage must include a failure after motion has begun; VALIDATE-only failure is insufficient. A cancel acknowledgment alone is not proof of downstream termination.
- New and legacy clients must share one busy/recovery gate and report cancellation consistently.
- Real and readonly modes must retain their motion locks under explicit launch overrides.

## Task 1: Snapshot both input workspaces

**Inputs:** /home/zmy/tomato/ubuntu_24.04_dual_arm_ws and /home/zmy/.codex/worktrees/b670/ubuntu_24.04_dual_arm_ws, both based on 8f0462c.

- [ ] Record input HEADs and hashes of selected tracked/untracked source and documentation files.
- [ ] Create reproducible source snapshots using a temporary Git index; leave original branches, indexes, and working files untouched.
- [ ] Include src, docs, and README changes using the explicit exclusions above. Include every hardware preservation manifest file; record hashes and the required values. Classify additional changed/untracked files individually in the ledger.
- [ ] Record snapshot commit IDs and selected paths in an integration ledger.
- [ ] Verify both snapshot parents/common base are 8f0462c before application. If input HEADs changed, record and review the changed merge base before applying anything; never silently assume the old base.
- [ ] Merge the hardware snapshot's delta, then the refactor snapshot's delta, into the existing integration branch with three-way Git merges. Preserve the current integration branch as ours; source snapshots are theirs. Do not use recursive directory copying or blanket ours/theirs conflict resolution.
- [ ] For each conflict, ledger the path, base/ours/theirs IDs, behavior selected, reason, and test that proves the resolution. Pay particular attention to stm32_backend.hpp, executor/CLI failure handling, interface generation, and launch defaults.
- [ ] Verify 454c4b5 and 68b955d remain ancestors; keep shutdown-cleanup, immutable in-flight snapshot, ACK-timeout retry, and optional-textures behavior as explicit regression checks after merging.

**Acceptance:** Both input working states and indexes remain unchanged; selected source files are represented in reachable snapshot commits. Both source deltas are reconciled against 8f0462c, conflict choices are documented, and neither existing integration repair is overwritten.

### Hardware preservation manifest

Paths below are repository-relative and must be retained from the hardware snapshot, with architecture changes reconciled separately. Ports stay configurable; operator-supplied explicit overrides may replace devices, but defaults and intermediate forwarding must not revert the values below.

| Required file | Required behavior/values |
| --- | --- |
| src/double_arm_end_effector/config/dual_gripper.yaml | device=/dev/serial/by-path/pci-0000:06:00.3-usb-0:2.2.3:1.0-port0; baud_rate=9600; slave_id=3; left_register=1; right_register=3; left_open_value=60; left_closed_value=40; right_open_value=25; right_closed_value=40. Preserve remaining serial/timing fields from the snapshot. |
| src/double_arm_end_effector/udev/99-tcp-gripper.rules | USB socket 1-2.2.2 -> tcp_stm32; 1-2.2.3 -> tcp_gripper; preserve access mode/group. |
| src/double_arm_robot_moveit_config/config/double_arm_robot.ros2_control.xacro | stm32_device default uses /dev/serial/by-path/pci-0000:06:00.3-usb-0:2.2.2:1.0-port0; preserve protocol/unit and calibration forwarding. |
| src/double_arm_robot_moveit_config/config/double_arm_robot.urdf.xacro | Same STM32 device default; forward it unchanged to ros2_control. |
| src/double_arm_harvest_execution/launch/harvest_stm32.launch.py | Preserve the STM32 .2 and gripper .3 by-path defaults; use explicit false-by-default motor/gripper motion gates from the refactor. |
| src/double_arm_robot_moveit_config/launch/stm32_moveit.launch.py | Preserve STM32 .2 by-path default, baud=115200, and joint directions 1,1,1,1,-1,1,1,1,1,1,-1,1; retain false-by-default motor enable. |
| src/double_arm_robot_moveit_config/launch/f2_stm32_readonly.launch.py | Preserve STM32 .2 by-path default; retain readonly topology and closed motion gates. |
| src/double_arm_end_effector/launch/gripper_calibration.launch.py and src/double_arm_end_effector/launch/dual_gripper.launch.py | Preserve calibration/raw/register-read entry points and reconcile their defaults/forwarding with the YAML and motion gates. |
| src/double_arm_harvest_execution/config/manual_targets.yaml | Preserve hardware workspace target coordinates/orientations as captured in the snapshot; adapt conversion to TaskTarget without changing the numeric poses. |

- [ ] Record the original hashes and parsed values of all manifest files. For newly introduced src/double_arm_bringup/launch/robot.launch.py, ensure its defaults and forwarding agree with this manifest instead of reintroducing /dev/ttyUSB0 or old register/calibration values.
- [ ] Resolve the full real launch parameter chain in a non-actuating test: unified bringup -> harvest -> STM32/xacro and gripper node. Assert the effective devices, slave/register IDs, open/closed values, baud rates, calibrated directions, and closed motion gates, not only text in individual files.
- [ ] Exercise explicit test device overrides and confirm they reach both the appropriate node and robot_description. Do not open real devices or install udev rules during these tests.

## Task 2: Integrate interfaces and hardware support

**Files:** double_arm_harvest_interfaces, double_arm_end_effector, double_arm_hardware, current robot xacro configuration.

- [ ] Combine ExecuteTask, TaskTarget, DetectedTarget, raw DualGripperCommand fields, and ReadGripperRegisters without dropping existing fields.
- [ ] Retain current gripper mapping, calibration tools, and STM32 pause/hold behavior.
- [ ] Run interface schema, raw gripper action, mapping, STM32 backend, and hardware lifecycle tests.
- [ ] Commit the coherent interface/hardware integration.

**Acceptance:** Interface generation and all affected consumers compile together; raw/mock gripper tests and hardware tests pass without physical-device access.

## Task 3: Integrate the unified executor lifecycle

**Files:** TaskExecutor header/source, task_request, task_behavior, MotionSkill, GripperSkill, StageBarrier, associated tests; add src/double_arm_harvest_execution/test/test_task_recovery_integration.py and register it in the package CMakeLists.txt.

- [ ] Adopt the shared task request, new/legacy action adapters, motion/gripper skills, and per-goal strategy selection.
- [ ] Write failing tests for PAUSED feedback and busy-gate retention, then adapt the shared lifecycle to preserve hardware failure-pausing behavior, explicitly including accepted-task VALIDATE failures.
- [ ] Implement the deterministic fault-injection acceptance matrix below for both action endpoints; observe a motion command or accepted downstream operation before injecting execution faults. Assert downstream cancellation/termination and absence of subsequent stage dispatches, rather than merely observing PAUSED text.
- [ ] Test cancellation during acceptance, during motion, and while paused; unconfirmed termination must block the next task until a late terminal result arrives.
- [ ] Test that goal_accept_timeout_sec ends acceptance waiting independently of stage_timeout_sec.
- [ ] Retain the promise-based fast-gripper-result fix and its success/failure/rejection/late-result regression cases.
- [ ] Test tomato -> leaf -> tomato, left-only/right-only requests with an empty disabled target, and no state leakage across requests.
- [ ] Commit the verified executor integration.

**Acceptance:** All rows of the fault matrix pass for new and legacy entry points. VALIDATE failures pause; failures after motion starts cancel the peer and never advance/replay a stage. Both action endpoints remain blocked while the task is PAUSED or any downstream operation is unresolved. A new task is accepted only after the original task is canceled and every outstanding downstream operation has terminated or been rejected; both completion orderings are tested. No stale callback affects a later dispatch.

### Deterministic fault-injection acceptance matrix

Use mock hardware and controllable FollowJointTrajectory/gripper action servers. Synchronize injections with goal acceptance/first-command events, not arbitrary sleeps. Record dispatches, feedback, cancellation requests, terminal results, and attempted new requests in one event trace. For each origin (/execute_task and /execute_dual_harvest), attempt new requests through both entry points at each blocked checkpoint. Set explicit fixture acceptance/action/cancel timeouts; state the stop/cancel observation deadline in terms of the configured cancel_timeout_sec plus a bounded event-processing allowance, and fail if it is exceeded rather than waiting indefinitely.

| Scenario | Injection and required observations |
| --- | --- |
| VALIDATE failure | Submit an accepted task with an invalid executable target. Assert PAUSED and failed stage VALIDATE, zero downstream motion/gripper goals, rejection of new tasks through both entry points, then cancellation and successful subsequent admission. This preserves the current hardware behavior but does not substitute for execution-fault tests. |
| One arm fails during motion | Start a dual-arm stage and confirm both arms have accepted goals and the peer has begun motion. Fail one arm while the peer remains active. Assert peer cancellation, a terminal peer result and bounded stopping/holding in the fake executor, PAUSED at the original stage, and no later-stage trajectories, HOME, or gripper release. New requests stay rejected even after the peer stops, until the paused task is canceled. |
| Gripper timeout | Confirm an enabled gripper goal was accepted, then withhold its result beyond gripper_action_timeout_sec. Assert cancellation is requested, PAUSED at that gripper stage, and no next-stage arm trajectory or further gripper command. Deliver a late success/failure result while paused and assert it neither advances the stage nor unlocks the task. Then cancel the task and prove subsequent admission. Cover PREPARE_GRIPPER and END_EFFECTOR so the latter has already executed arm motion. |
| Cancellation not confirmed | Start arm motion and separately a gripper operation in parameterized cases; accept the task cancellation but withhold the downstream terminal result beyond cancel_timeout_sec (an optional cancel acknowledgment must not count as termination). Assert recovery-required reporting and rejection of requests through both entry points after the task terminates. Deliver the late terminal result and prove the next task can start without stale feedback/result contamination. Also withhold goal acceptance, then deliver acceptance after cancellation and prove the late accepted goal is canceled and remains blocking until its terminal result/rejection. |

For every execution-fault row, assert that only allowed cancellation/hold operations occur after the fault: no new stage goal, automatic retry, HOME, or open/release command. Test both orderings: downstream termination before task cancellation, and task cancellation before downstream termination. Fake stop/hold evidence and action terminal states do not constitute proof that physical gripper motion has stopped; the legacy gripper protocol has no stop register, so physical verification remains separate.

## Task 4: Reconcile launch, CLI, and documentation

**Files:** double_arm_bringup/launch/robot.launch.py, harvest_fake.launch.py, harvest_stm32.launch.py, stm32_moveit.launch.py, send_manual_harvest_goal.py, architecture/development/bringup/interface docs.

- [ ] Combine current hardware device defaults with fake/readonly/real bringup and explicit real motion gates.
- [ ] Keep per-goal task type selection and new target conversion in the CLI while preserving paused-result waiting and loop stop after unsuccessful outcomes.
- [ ] Add/adjust launch tests for mode topology and locked real/readonly behavior, plus CLI tests for pause and failure handling.
- [ ] Add src/double_arm_bringup/test/test_effective_bringup_parameters.py for the resolved hardware manifest and register it with the bringup package's test runner.
- [ ] Run the manifest's effective-parameter checks through the final unified launch, including explicit device overrides; record final resolved values in the ledger.
- [ ] Update examples and remove contradictory old task-mode, hardware mapping, and failure behavior descriptions.
- [ ] Commit launch/CLI/documentation integration.

**Acceptance:** Documented commands resolve to the intended mode; no real device is opened by fake tests; readonly remains command-free.

## Task 5: Validate and merge locally

- [ ] Build a clean source snapshot, including dependencies for double_arm_bringup, double_arm_hardware, double_arm_harvest_execution, double_arm_end_effector, and double_arm_sparse_execution. Select system Python and dependencies to avoid Miniconda library collisions.
- [ ] Run all tests for affected packages with colcon test; inspect colcon test-result --verbose. Report every remaining failure by name.
- [ ] Run fake integration covering legacy/new interfaces, both task types, single arms, serialization, pause/cancel, delayed acceptance/result, and subsequent recovery.
- [ ] Require every fault-injection matrix row and every hardware manifest effective-parameter check to pass; a VALIDATE-only pause test does not satisfy motion-failure acceptance.
- [ ] Verify 454c4b5 and 68b955d remain ancestors of the final integration commit and proposed main. Rerun both executor shutdown/SIGINT cases, InFlightSnapshotNotMutated, RetriesLatestTargetAfterAckTimeout, and clean install without textures; check an available textures directory is still installed. These are behavior checks, not source-text equality checks.
- [ ] Obtain an independent whole-change review; fix substantive findings with regression tests and rerun affected checks.
- [ ] Verify input workspaces remain unchanged, diff has no conflict markers/whitespace errors, and the integration branch is committed and clean.
- [ ] Merge the verified integration into local main; preserve source branches/worktrees and report the final commit and test evidence. Do not push until separately authorized.

**Acceptance:** Clean build and automated checks pass, review has no unresolved substantive findings, and local main contains the integrated changes. Physical robot execution and recalibration are explicitly unverified.
