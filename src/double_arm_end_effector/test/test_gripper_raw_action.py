"""Exercise raw calibration commands through the real controller in mock mode.

Run on an isolated ROS_DOMAIN_ID; this fixture never opens a hardware port.
"""

import os
import importlib.util
from pathlib import Path
import subprocess
import time

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from double_arm_harvest_interfaces.action import DualGripperCommand
from rclpy.action import ActionClient


@pytest.fixture(scope="module")
def controller():
    executable = os.environ.get("GRIPPER_CONTROLLER_EXECUTABLE") or str(
        Path(get_package_prefix("double_arm_end_effector")) /
        "lib/double_arm_end_effector/dual_gripper_controller")
    process = subprocess.Popen([
        executable, "--ros-args", "-p", "mock_hardware:=true",
        "-p", "slave_id:=3", "-p", "left_register:=1", "-p", "right_register:=3",
        "-p", "write_delay_ms:=0", "-p", "left_open_value:=40",
        "-p", "left_closed_value:=60", "-p", "right_open_value:=25",
        "-p", "right_closed_value:=40",
    ])
    rclpy.init()
    node = rclpy.create_node("gripper_calibration_test")
    client = ActionClient(node, DualGripperCommand, "/dual_gripper/command")
    try:
        assert client.wait_for_server(timeout_sec=10.0), "mock controller unavailable"
        yield node, client
    finally:
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def send(controller, goal):
    node, client = controller
    sent = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, sent, timeout_sec=5.0)
    assert sent.done(), "goal acknowledgement timed out"
    handle = sent.result()
    if not handle.accepted:
        return None
    future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    assert future.done(), "mock command timed out"
    return future.result().result


@pytest.mark.parametrize("side,value", [("left", 47), ("right", 73), ("right", 0), ("left", 65535)])
def test_raw_value_is_not_rescaled_or_written_to_other_channel(controller, side, value):
    goal = DualGripperCommand.Goal()
    assert hasattr(goal, "use_raw_values"), "raw calibration commands are not supported"
    goal.use_raw_values = True
    goal.command_left = side == "left"
    goal.command_right = side == "right"
    goal.left_raw_value = value if side == "left" else 0
    goal.right_raw_value = value if side == "right" else 0
    result = send(controller, goal)
    assert result is not None and result.success
    assert result.left_pulse == (value if side == "left" else -1)
    assert result.right_pulse == (value if side == "right" else -1)
    assert result.left_success == (side == "left")
    assert result.right_success == (side == "right")


def test_existing_normalized_harvest_command_keeps_calibrated_mapping(controller):
    goal = DualGripperCommand.Goal()
    goal.command_left = goal.command_right = True
    goal.left_position = goal.right_position = 0.5
    result = send(controller, goal)
    assert result is not None and result.success
    assert (result.left_pulse, result.right_pulse) == (50, 33)


def test_invalid_normalized_command_and_unselected_channels_are_rejected(controller):
    goal = DualGripperCommand.Goal()
    assert send(controller, goal) is None
    goal.command_left = True
    goal.left_position = float("nan")
    assert send(controller, goal) is None


def test_raw_mode_still_rejects_excessive_settle_time(controller):
    goal = DualGripperCommand.Goal()
    goal.use_raw_values = True
    goal.command_left = True
    goal.left_raw_value = 50
    goal.duration_ms = 60001
    assert send(controller, goal) is None


def test_register_read_returns_written_values_without_a_motion_goal(controller):
    try:
        from double_arm_harvest_interfaces.srv import ReadGripperRegisters
    except ImportError:
        pytest.fail("controller does not expose register reads")
    goal = DualGripperCommand.Goal()
    goal.use_raw_values = True
    goal.command_left = goal.command_right = True
    goal.left_raw_value, goal.right_raw_value = 51, 31
    assert send(controller, goal).success
    node, _ = controller
    reader = node.create_client(ReadGripperRegisters, "/dual_gripper/read_registers")
    try:
        assert reader.wait_for_service(timeout_sec=5)
        future = reader.call_async(ReadGripperRegisters.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        assert future.result().success
        assert (future.result().left_value, future.result().right_value) == (51, 31)
    finally:
        node.destroy_client(reader)


def test_raw_goal_shares_busy_guard_and_can_cancel_remaining_work(controller):
    node, client = controller
    goal = DualGripperCommand.Goal()
    goal.use_raw_values = True
    goal.command_left = True
    goal.left_raw_value = 47
    goal.duration_ms = 2000
    first = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, first, timeout_sec=5.0)
    handle = first.result()
    assert handle.accepted
    assert send(controller, goal) is None
    canceled = handle.cancel_goal_async()
    rclpy.spin_until_future_complete(node, canceled, timeout_sec=5.0)
    assert canceled.result().goals_canceling
    result = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result, timeout_sec=5.0)
    assert not result.result().result.success
    assert result.result().status == 5  # action_msgs/GoalStatus.STATUS_CANCELED
    goal.duration_ms = 0
    assert send(controller, goal).success


def test_gui_sends_selected_raw_value_and_records_config_without_automatic_motion(controller):
    import tkinter as tk

    existing = DualGripperCommand.Goal()
    existing.use_raw_values = True
    existing.command_left = existing.command_right = True
    existing.left_raw_value, existing.right_raw_value = 50, 30
    assert send(controller, existing).success

    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("Tk display unavailable; run on a desktop to exercise the GUI")
    root.withdraw()
    path = Path(__file__).resolve().parents[1] / "scripts/gripper_calibration_gui.py"
    spec = importlib.util.spec_from_file_location("gripper_calibration_gui", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    app = module.GripperCalibrationGui(root, controller[0])

    def wait_until(predicate):
        deadline = time.monotonic() + 8.0
        while not predicate() and time.monotonic() < deadline:
            root.update()
            time.sleep(0.01)
        assert predicate(), app.status.get()

    try:
        wait_until(lambda: app.registers_loaded)
        assert app.values["left"].get() == "50"
        assert app.values["right"].get() == "30"
        assert not app.busy
        app.duration.set("0")
        app.values["right"].set("1.5")
        app.send("right")
        assert not app.busy
        assert "整数" in app.status.get()
        # Simulate a snapshot from a previous controller. A new send must read
        # the actual mode/endpoints and stop before acting on stale metadata.
        actual_config = dict(app.config)
        app.config["mock_hardware"] = False
        app.config["allow_motion"] = True
        app.config["left_register"] = 99
        app.values["right"].set("73")
        app.send("right")
        # A reconnect refresh can finish while the send's preflight is pending.
        # The decision must use the snapshot from the click, not this new cache.
        app.config = actual_config
        wait_until(lambda: not app.busy)
        assert app.config["mock_hardware"] is True
        assert app.config["left_register"] == 1
        assert app.last_written["right"].get() == "最后确认写值：—"
        wait_until(lambda: app.registers_loaded)
        app.values["right"].set("72")
        app.adjust("right", 1)
        app.send("right")
        assert app.busy
        wait_until(lambda: not app.busy)
        assert app.last_written["right"].get() == "最后确认写值：73"
        assert app.last_written["left"].get() == "最后确认写值：—"

        app.values["left"].set("47")
        app.record("left", "open")
        app.values["left"].set("52")
        app.load_preset("left", "open")
        assert app.values["left"].get() == "47"
        assert not app.busy
        assert app.last_written["left"].get() == "最后确认写值：—"
        app.copy_calibration()
        assert root.clipboard_get() == (
            "    left_open_value: 47\n    left_closed_value: 60\n"
            "    right_open_value: 25\n    right_closed_value: 40\n")

        # Model a parameter request lost during a controller restart. Expiring
        # the unanswered future must permit discovery and register reads again.
        from rclpy.task import Future
        pending = Future()
        app.config = None
        app.registers_loaded = False
        app.parameter_pending = True
        pending.add_done_callback(app._parameters_done)
        app._initial_parameters_timeout(pending)
        assert pending.cancelled() and not app.parameter_pending
        wait_until(lambda: app.config is not None and app.registers_loaded)
    finally:
        app.client.destroy()
        root.destroy()
