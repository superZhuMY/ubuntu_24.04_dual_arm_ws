"""Verify both executor entry points exit cleanly after ROS handles SIGINT."""

import signal
import subprocess
import sys
import threading

import pytest

pytest.importorskip("rclpy")


@pytest.mark.parametrize("module", [
    "sparse_trajectory_executor",
    "streaming_trajectory_executor",
])
def test_sigint_exits_without_cleanup_errors(module):
    script = f"""
from double_arm_sparse_execution import {module} as entry

class ReadyExecutor(entry.MultiThreadedExecutor):
    def spin(self):
        print("EXECUTOR_SPINNING", flush=True)
        return super().spin()

entry.MultiThreadedExecutor = ReadyExecutor
entry.main()
"""
    process = subprocess.Popen(
        [sys.executable, "-c", script],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )
    ready = threading.Event()
    lines = []

    def read_output():
        for line in process.stdout:
            lines.append(line)
            if "EXECUTOR_SPINNING" in line:
                ready.set()

    reader = threading.Thread(target=read_output, daemon=True)
    reader.start()
    try:
        assert ready.wait(15.0), "executor did not start: " + "".join(lines)
        process.send_signal(signal.SIGINT)
        process.wait(timeout=15.0)
        reader.join(timeout=2.0)
        output = "".join(lines)
        assert process.returncode == 0, output
        assert "Traceback" not in output, output
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5.0)
        reader.join(timeout=2.0)
        process.stdout.close()
