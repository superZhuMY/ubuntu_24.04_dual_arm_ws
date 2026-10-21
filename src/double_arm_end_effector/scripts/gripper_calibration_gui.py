#!/usr/bin/python3
"""Tune gripper register values using the existing controller's Action.

Tk owns the main thread; nonblocking ROS polling and all callbacks run on the
same thread. This process never opens the serial port or sends on startup.
"""

import tkinter as tk
from tkinter import ttk

import rclpy
from action_msgs.msg import GoalStatus
from double_arm_harvest_interfaces.action import DualGripperCommand
from double_arm_harvest_interfaces.srv import ReadGripperRegisters
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient


class GripperCalibrationGui:
    PARAMETER_NAMES = (
        "device", "baud_rate",
        "slave_id", "left_register", "right_register", "left_open_value",
        "left_closed_value", "right_open_value", "right_closed_value",
        "mock_hardware", "allow_motion",
    )

    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.client = ActionClient(node, DualGripperCommand, "/dual_gripper/command")
        self.parameters = AsyncParameterClient(node, "/dual_gripper_controller")
        self.reader = node.create_client(ReadGripperRegisters, "/dual_gripper/read_registers")
        self.read_pending = False
        self.read_attempted = False
        self.registers_loaded = False
        self.config = None
        self.parameter_pending = False
        self.busy = False
        self.goal_handle = None
        self.active_side = None
        self.command_mock = False
        self.active_register = None
        self.values = {}
        self.calibration = {}
        self.endpoints = {}
        self.cards = {}
        self.last_written = {}
        self.register_values = {}
        self.send_buttons = []
        self.status = tk.StringVar(value="等待夹爪节点连接…")
        self.connection = tk.StringVar(value="未连接")
        self.duration = tk.StringVar(value="800")
        self._build_window()
        self.root.after(50, self._poll)

    def _build_window(self):
        self.root.title("双夹爪开合标定")
        self.root.minsize(760, 520)
        frame = ttk.Frame(self.root, padding=20)
        frame.pack(fill="both", expand=True)
        ttk.Label(frame, text="夹爪开合数值调试", font=("sans", 18, "bold")).pack(anchor="w")
        ttk.Label(frame, textvariable=self.connection).pack(anchor="w", pady=(8, 4))
        ttk.Label(frame, text="输入设备原始寄存器值，观察实际角度；修改数值后点击「发送」才动作。"
                  ).pack(anchor="w", pady=(0, 12))

        cards = ttk.Frame(frame)
        cards.pack(fill="x")
        for column, (side, title) in enumerate((("left", "左夹爪"), ("right", "右夹爪"))):
            cards.columnconfigure(column, weight=1)
            card = ttk.LabelFrame(cards, text=title, padding=12)
            self.cards[side] = card
            card.grid(row=0, column=column, sticky="nsew", padx=(0, 8) if column == 0 else (8, 0))
            self.endpoints[side] = tk.StringVar(value="通道信息等待读取")
            ttk.Label(card, textvariable=self.endpoints[side]).pack(anchor="w")
            self.register_values[side] = tk.StringVar(value="寄存器读值：未读取")
            ttk.Label(card, textvariable=self.register_values[side]).pack(anchor="w")
            self.values[side] = tk.StringVar(value="")
            ttk.Label(card, text="目标原始值（整数 0–65535）").pack(anchor="w", pady=(10, 3))
            ttk.Spinbox(card, textvariable=self.values[side], from_=0, to=65535,
                        increment=1, width=16, font=("sans", 16)).pack(anchor="w")
            steps = ttk.Frame(card)
            steps.pack(anchor="w", pady=7)
            for step in (-5, -1, 1, 5):
                ttk.Button(steps, text=f"{step:+d}", width=5,
                           command=lambda s=side, d=step: self.adjust(s, d)).pack(side="left", padx=2)
            send = ttk.Button(card, text=f"发送到{title}", command=lambda s=side: self.send(s))
            send.pack(fill="x", pady=4)
            send.state(["disabled"])
            self.send_buttons.append(send)
            self.last_written[side] = tk.StringVar(value="最后确认写值：—")
            ttk.Label(card, textvariable=self.last_written[side]).pack(anchor="w", pady=(3, 10))

            self.calibration[side] = {}
            for kind, label in (("open", "打开值"), ("closed", "闭合值")):
                row = ttk.Frame(card)
                row.pack(fill="x", pady=3)
                variable = tk.StringVar(value="")
                self.calibration[side][kind] = variable
                ttk.Label(row, text=label, width=7).pack(side="left")
                ttk.Entry(row, textvariable=variable, width=7).pack(side="left", padx=3)
                ttk.Button(row, text="载入", width=5,
                           command=lambda s=side, k=kind: self.load_preset(s, k)).pack(side="left")
                ttk.Button(row, text="记录当前值", width=10,
                           command=lambda s=side, k=kind: self.record(s, k)).pack(side="left", padx=3)

        tools = ttk.Frame(frame)
        tools.pack(fill="x", pady=(15, 8))
        ttk.Label(tools, text="写入后等待（毫秒）：").pack(side="left")
        ttk.Entry(tools, textvariable=self.duration, width=7).pack(side="left")
        self.cancel_button = ttk.Button(tools, text="取消后续写入", command=self.cancel)
        self.cancel_button.pack(side="right")
        self.cancel_button.state(["disabled"])
        ttk.Button(tools, text="复制标定配置", command=self.copy_calibration).pack(side="right", padx=8)
        ttk.Button(tools, text="读取寄存器", command=self.read_registers).pack(side="right")
        ttk.Label(frame, text="记录/载入只修改输入框。复制四个标定值到 dual_gripper.yaml，重新构建并重启夹爪节点生效。",
                  wraplength=730).pack(anchor="w", pady=4)
        ttk.Label(frame, text="写入确认来自 Modbus 应答，不是实际角度反馈。取消和关闭窗口无法撤回已发出的动作。",
                  wraplength=730).pack(anchor="w", pady=4)
        ttk.Separator(frame).pack(fill="x", pady=10)
        ttk.Label(frame, textvariable=self.status, wraplength=730).pack(anchor="w")

    @staticmethod
    def integer(text, label, upper=65535):
        try:
            value = int(text)
        except ValueError:
            raise ValueError(f"{label}必须是 0–{upper} 的整数") from None
        if not 0 <= value <= upper:
            raise ValueError(f"{label}必须在 0–{upper} 之间")
        return value

    def adjust(self, side, delta):
        try:
            value = self.integer(self.values[side].get(), "目标值")
            self.values[side].set(str(max(0, min(65535, value + delta))))
        except ValueError as error:
            self.status.set(str(error))

    def load_preset(self, side, kind):
        self.values[side].set(self.calibration[side][kind].get())

    def record(self, side, kind):
        try:
            value = self.integer(self.values[side].get(), "目标值")
            self.calibration[side][kind].set(str(value))
            self.status.set("已记录候选标定值；点击「复制标定配置」后写入配置文件。")
        except ValueError as error:
            self.status.set(str(error))

    def copy_calibration(self):
        try:
            lines = []
            for side in ("left", "right"):
                opened = self.integer(self.calibration[side]["open"].get(), "打开值")
                closed = self.integer(self.calibration[side]["closed"].get(), "闭合值")
                if opened == closed:
                    raise ValueError("同一夹爪的打开值和闭合值不能相同")
                lines.extend((f"    {side}_open_value: {opened}",
                              f"    {side}_closed_value: {closed}"))
            self.root.clipboard_clear()
            self.root.clipboard_append("\n".join(lines) + "\n")
            self.status.set("已复制：替换 dual_gripper.yaml 中对应四项，重新构建并重启夹爪节点。")
        except (ValueError, tk.TclError) as error:
            self.status.set(str(error))

    def read_parameters(self):
        if self.busy or self.parameter_pending:
            return
        if not self.parameters.services_are_ready():
            self.status.set("夹爪参数服务未连接，请先启动 dual_gripper_controller。")
            return
        self.parameter_pending = True
        future = self.parameters.get_parameters(list(self.PARAMETER_NAMES))
        future.add_done_callback(self._parameters_done)
        self.root.after(5000, lambda: self._initial_parameters_timeout(future))

    def _initial_parameters_timeout(self, future):
        if not future.done():
            future.cancel()
            self.parameter_pending = False
            self.config = None
            self.status.set("读取节点参数超时，等待夹爪节点重新连接…")

    def _parameters_done(self, future):
        if future.cancelled():
            return
        self.parameter_pending = False
        try:
            self._apply_config(self._decode_parameters(future), reset_values=True)
            self.read_attempted = False
            self.registers_loaded = False
            self.status.set("已读取通道配置，正在等待读取设备寄存器现有值…")
        except Exception as error:
            self.config = None
            self.status.set(f"读取参数失败：{error}")

    def read_registers(self):
        if self.busy or self.read_pending or self.config is None:
            return
        if not self.reader.service_is_ready():
            self.status.set("等待读寄存器服务；请确认夹爪节点已更新并重启。")
            return
        self.read_pending = True
        self.read_attempted = True
        self.status.set("正在读取通道寄存器…")
        self._update_controls()
        future = self.reader.call_async(ReadGripperRegisters.Request())
        future.add_done_callback(self._registers_done)
        self.root.after(5000, lambda: self._read_timeout(future))

    def _read_timeout(self, future):
        if not future.done():
            future.cancel()
            self.read_pending = False
            self.registers_loaded = False
            self.status.set("读取寄存器超时，请检查串口、从站 ID 和通道。")

    def _registers_done(self, future):
        if future.cancelled():
            return
        self.read_pending = False
        try:
            result = future.result()
            if not result.success:
                raise RuntimeError(result.message)
            for side, value in (("left", result.left_value), ("right", result.right_value)):
                self.values[side].set(str(value))
                self.register_values[side].set(f"寄存器读值：{value}")
            self.registers_loaded = True
            self.status.set(f"读取成功：通道寄存器现有值 {result.left_value} / {result.right_value}，未发送运动命令。")
        except Exception as error:
            self.registers_loaded = False
            self.status.set(f"读取寄存器失败：{error}")
        self._update_controls()

    def _decode_parameters(self, future):
        values = future.result().values
        if len(values) != len(self.PARAMETER_NAMES) or any(v.type == 0 for v in values):
            raise ValueError("夹爪节点缺少标定参数")
        return {
            name: (value.string_value if name == "device" else
                   value.bool_value if name in ("mock_hardware", "allow_motion") else value.integer_value)
            for name, value in zip(self.PARAMETER_NAMES, values)
        }

    def _apply_config(self, config, reset_values=False):
        self.config = config
        for index, side in enumerate(("left", "right")):
            self.cards[side].configure(text=f"通道 {config[side + '_register']}")
            self.send_buttons[index].configure(text=f"发送到通道 {config[side + '_register']}")
            self.endpoints[side].set(
                f"从站 {config['slave_id']} / 通道寄存器 {config[side + '_register']}")
            if reset_values:
                for kind in ("open", "closed"):
                    self.calibration[side][kind].set(str(config[f"{side}_{kind}_value"]))

    def send(self, side):
        if self.busy:
            return
        if not self.client.server_is_ready() or self.config is None or not self.registers_loaded or self.read_pending:
            self.status.set("请先等待寄存器读取成功，再输入目标值发送。")
            return
        if not self.config["allow_motion"] and not self.config["mock_hardware"]:
            self.status.set("节点运动未启用：启动时设置 allow_motion:=true。")
            return
        try:
            value = self.integer(self.values[side].get(), "目标值")
            duration = self.integer(self.duration.get(), "等待时间", 60000)
            self.busy = True
            self.active_side = side
            self.status.set("正在核对当前节点模式、通道和标定参数…")
            self._update_controls()
            # A restarted controller can have changed from mock to real. Refresh
            # immediately before each send, even if DDS discovery saw no gap.
            expected_config = dict(self.config)
            pending = self.parameters.get_parameters(list(self.PARAMETER_NAMES))
            pending.add_done_callback(
                lambda f: self._send_checked(f, side, value, duration, expected_config))
            self.root.after(5000, lambda: self._parameter_timeout(pending))
        except Exception as error:
            self._finished(f"未发送：{error}")

    def _parameter_timeout(self, future):
        if not future.done():
            future.cancel()
            self._finished("参数查询超时，未发送命令。请重新连接夹爪节点。")

    def _send_checked(self, future, side, value, duration, expected_config):
        if future.cancelled():
            return
        try:
            config = self._decode_parameters(future)
            if config != expected_config:
                self._apply_config(config)
                self.registers_loaded = False
                self.read_attempted = False
                self._finished("节点模式、通道或标定参数发生变化，已刷新显示；请检查后重新点击发送。")
                return
            self._apply_config(config)
            if not config["mock_hardware"] and not config["allow_motion"]:
                self._finished("节点运动未启用：allow_motion=false，未发送命令。")
                return
            self.command_mock = config["mock_hardware"]
            self.active_register = config[side + "_register"]
            goal = DualGripperCommand.Goal()
            goal.use_raw_values = True
            goal.command_left = side == "left"
            goal.command_right = side == "right"
            goal.left_raw_value = value if side == "left" else 0
            goal.right_raw_value = value if side == "right" else 0
            goal.duration_ms = duration
            self.status.set(f"正在下发通道 {self.active_register} 原始值 {value}…")
            self.client.send_goal_async(goal).add_done_callback(self._goal_done)
        except Exception as error:
            self._finished(f"未发送：{error}")

    def _goal_done(self, future):
        try:
            self.goal_handle = future.result()
            if not self.goal_handle.accepted:
                self._finished("命令被拒绝：请检查节点是否忙碌或 allow_motion 是否启用。")
                return
            self.cancel_button.state(["!disabled"])
            self.goal_handle.get_result_async().add_done_callback(self._result_done)
        except Exception as error:
            self._finished(f"下发失败：{error}")

    def _result_done(self, future):
        try:
            response = future.result()
            result = response.result
            if response.status == GoalStatus.STATUS_SUCCEEDED and result.success:
                side = self.active_side
                value = result.left_pulse if side == "left" else result.right_pulse
                self.last_written[side].set(f"最后确认写值：{value}")
                mode = "模拟写入" if self.command_mock else "Modbus 写入"
                self._finished(f"通道 {self.active_register} {mode}确认，原始值 {value}。")
            else:
                self._finished(f"命令未完成：{result.message}")
        except Exception as error:
            self._finished(f"读取结果失败：{error}")

    def cancel(self):
        if self.goal_handle is not None and self.busy:
            self.cancel_button.state(["disabled"])
            self.goal_handle.cancel_goal_async()
            self.status.set("已请求取消后续写入，正在等待结果；已发出的动作无法撤回。")

    def _finished(self, text):
        self.busy = False
        self.goal_handle = None
        self.status.set(text)
        self._update_controls()

    def _update_controls(self):
        connected = self.client.server_is_ready()
        enabled = (connected and self.config is not None and self.registers_loaded and not self.read_pending and
                   (self.config["allow_motion"] or self.config["mock_hardware"]) and not self.busy)
        for button in self.send_buttons:
            button.state(["!disabled"] if enabled else ["disabled"])
        if not self.busy:
            self.cancel_button.state(["disabled"])
        if not connected:
            self.connection.set("未连接：请启动夹爪节点")
        elif self.config is None:
            self.connection.set("已连接，正在读取通道和标定值")
        else:
            mode = "模拟模式（不访问串口）" if self.config["mock_hardware"] else "真实夹爪"
            if not self.config["allow_motion"] and not self.config["mock_hardware"]:
                lock = "运动锁定：allow_motion=false"
            elif not self.registers_loaded or self.read_pending:
                lock = "待读取寄存器"
            else:
                lock = "命令执行中" if self.busy else "可发送"
            self.connection.set(
                f"节点已连接 · {self.config['device']} · {self.config['baud_rate']} bps · {mode} · {lock}")

    def _poll(self):
        if not rclpy.ok(context=self.node.context):
            self.root.destroy()
            return
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
        except ExternalShutdownException:
            self.root.destroy()
            return
        if not self.client.server_is_ready():
            self.config = None
            self.registers_loaded = False
            self.read_attempted = False
            if self.busy:
                self.status.set("夹爪连接中断，已发送命令的执行结果尚未确认。")
        if self.config is None and not self.parameter_pending and self.parameters.services_are_ready():
            self.read_parameters()
        if self.config is not None and not self.read_attempted and not self.busy:
            self.read_registers()
        self._update_controls()
        self.root.after(50, self._poll)


def main():
    rclpy.init()
    node = Node("gripper_calibration_gui")
    root = None
    try:
        if not hasattr(DualGripperCommand.Goal(), "use_raw_values"):
            raise RuntimeError("夹爪接口尚未重新构建，请构建工作区并 source install/setup.bash")
        root = tk.Tk()
        GripperCalibrationGui(root, node)
        root.mainloop()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (tk.TclError, RuntimeError) as error:
        node.get_logger().error(str(error))
        return 1
    finally:
        if root is not None:
            try:
                root.destroy()
            except tk.TclError:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
