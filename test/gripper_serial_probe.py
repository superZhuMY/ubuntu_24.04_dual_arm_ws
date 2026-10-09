#!/usr/bin/env python3
"""串行总线夹爪舵机通信探测工具。

用途:插上 USB/TTL 单总线调试板后,判断串口链路是否真的能跟舵机对话。
关键点:单总线半双工板会把主机自己发出的信号反射回 RX,形成"回声"。
本工具用内容校验区分「回声」和「舵机真实应答」,避免被回声骗过。

协议依据 src/double_arm_end_effector:
    #dddPRAD!       读位置查询
    #dddPdddd!      位置应答(例 #001P1623!)
    #dddPddddTdddd! 运动指令
串口固定 115200 8N1 无流控。

用法:
    python3 test/gripper_serial_probe.py /dev/ttyUSB1
    python3 test/gripper_serial_probe.py /dev/ttyUSB1 --scan-ids 0 1 2 3
    python3 test/gripper_serial_probe.py /dev/ttyUSB1 --baud-sweep
    python3 test/gripper_serial_probe.py /dev/ttyUSB1 --binary-ping
    python3 test/gripper_serial_probe.py /dev/ttyUSB1 --listen 2
    python3 test/gripper_serial_probe.py /dev/ttyUSB1 --watch 10
"""
from __future__ import annotations

import argparse
import re
import sys
import time

try:
    import serial
except ImportError:  # pragma: no cover
    sys.exit("缺少 pyserial,请先: pip install pyserial")

# 舵机位置应答,例 "#001P1623!"
REPLY_RE = re.compile(rb"#(\d{3})P(\d{4})!")
# 我们自己的查询帧被反射回来,例 "#001PRAD!"
ECHO_RE = re.compile(rb"#\d{3}P(?:RAD|DST)!")
# 我们自己的运动指令被反射回来
MOVE_ECHO_RE = re.compile(rb"#\d{3}P\d{4}T\d{4}!")

DEFAULT_BAUDS = [115200, 1000000, 500000, 250000, 128000, 230400, 57600, 38400, 19200, 9600]


def classify(buf: bytes) -> str:
    """把收到的字节分类成 应答 / 回声 / 混合 / 无。"""
    if not buf:
        return "无"
    kinds = set()
    if REPLY_RE.search(buf):
        kinds.add("舵机应答")
    if ECHO_RE.search(buf) and not REPLY_RE.search(buf):
        kinds.add("自身回声")
    elif ECHO_RE.search(buf) and REPLY_RE.search(buf):
        kinds.add("回声+应答")
    if MOVE_ECHO_RE.search(buf):
        kinds.add("运动回声")
    return "/".join(sorted(kinds)) if kinds else "垃圾/乱码"


def exchange(ser: "serial.Serial", frame: bytes, reply_window: float) -> tuple[bytes, bytes]:
    """发送一帧,返回 (回声窗口内读到的, 之后继续读到的)。"""
    ser.reset_input_buffer()
    ser.write(frame)
    ser.flush()
    time.sleep(0.05)          # 足够 9 字节回声回来
    first = ser.read(4096)
    t0 = time.time()
    rest = b""
    while time.time() - t0 < reply_window:
        chunk = ser.read(4096)
        if chunk:
            rest += chunk
            break
        time.sleep(0.01)
    return first, rest


def scan_ids(port: str, baud: int, ids, reply_window: float, raw: bool) -> bool:
    print(f"\n== 扫描 ASCII 位置查询  port={port} baud={baud} ==")
    found = False
    with serial.Serial(port, baud, timeout=0) as ser:
        time.sleep(0.3)
        initial = ser.read(4096)
        if initial:
            print(f"  空闲时就有数据({len(initial)}B): {initial!r}  <- 总线上有别的东西在发")
        for sid in ids:
            frame = b"#%03dPRAD!" % sid
            first, rest = exchange(ser, frame, reply_window)
            allb = first + rest
            kind = classify(allb)
            hit = "舵机应答" in kind
            found = found or hit
            mark = "  <== 命中" if hit else ""
            print(f"  ID {sid:>3}: {kind:<12} 收 {len(allb):>2}B{mark}")
            if raw or hit:
                if first:
                    print(f"         回声窗: {first!r}  hex={first.hex(' ')}")
                if rest:
                    print(f"         后续  : {rest!r}  hex={rest.hex(' ')}")
    return found


def sweep_bauds(port: str, bauds, ids, reply_window: float) -> bool:
    print(f"\n== 扫描波特率 port={port} ==")
    found = False
    for baud in bauds:
        try:
            with serial.Serial(port, baud, timeout=0) as ser:
                time.sleep(0.15)
                for sid in ids:
                    first, rest = exchange(ser, b"#%03dPRAD!" % sid, reply_window)
                    allb = first + rest
                    kind = classify(allb)
                    if "舵机应答" in kind:
                        found = True
                        print(f"  {baud:>7} ID{sid}: 命中 -> {allb!r}")
                        break
                else:
                    print(f"  {baud:>7}: 无应答(回声 {len(first)}B)")
        except serial.SerialException as exc:
            print(f"  {baud:>7}: 打开失败 {exc}")
    return found


def binary_ping(port: str, bauds, ids) -> bool:
    """尝试 Feetech/Waveshare 风格二进制 ping: FF FF ID 02 01 CHK。"""
    print(f"\n== 二进制 ping (FF FF ID 02 01 CHK) port={port} ==")
    found = False

    def checksum(body: bytes) -> int:
        return (~sum(body)) & 0xFF

    for baud in bauds:
        try:
            with serial.Serial(port, baud, timeout=0.25) as ser:
                for sid in ids:
                    frame = bytes([0xFF, 0xFF, sid, 0x02, 0x01, checksum(bytes([sid, 0x02, 0x01]))])
                    ser.reset_input_buffer()
                    ser.write(frame)
                    ser.flush()
                    time.sleep(0.12)
                    resp = ser.read(256)
                    if resp:
                        print(f"  {baud:>7} ID{sid:#04x}: {resp!r} hex={resp.hex(' ')}")
                        found = True
        except serial.SerialException as exc:
            print(f"  {baud:>7}: 打开失败 {exc}")
    if not found:
        print("  所有波特率/ID 均无任何回包")
    return found


def listen(port: str, baud: int, seconds: float) -> bytes:
    print(f"\n== 只监听 {seconds}s(不发送) port={port} baud={baud} ==")
    with serial.Serial(port, baud, timeout=0.2) as ser:
        ser.reset_input_buffer()
        t0 = time.time()
        got = b""
        while time.time() - t0 < seconds:
            chunk = ser.read(4096)
            if chunk:
                got += chunk
                print(f"  +{(time.time()-t0)*1000:7.1f}ms  {chunk!r} hex={chunk.hex(' ')}")
        print(f"  合计 {len(got)}B" + ("" if got else "  <- 总线空闲"))
    return got


def watch(port: str, baud: int, seconds: float, ids) -> bool:
    print(f"\n== 持续查询 {seconds}s port={port} baud={baud} ids={ids} ==")
    found = False
    deadline = time.time() + seconds
    n = 0
    with serial.Serial(port, baud, timeout=0) as ser:
        while time.time() < deadline:
            for sid in ids:
                n += 1
                first, rest = exchange(ser, b"#%03dPRAD!" % sid, 0.15)
                allb = first + rest
                if "舵机应答" in classify(allb):
                    print(f"  [{n}] ID{sid} 命中: {allb!r}")
                    found = True
            time.sleep(0.05)
    print(f"  共查询 {n} 次," + ("有应答" if found else "始终只有回声/无数据"))
    return found


def fingerprint(port: str, baud: int, seconds: float, ids) -> None:
    """实时打印回声指纹,用于接线诊断。

    拔下舵机的 3P 线时观察输出:
      - 指纹【不变】  -> 回声来自调试板内部 TX->RX 回灌,信号线根本没接到舵机
      - 指纹【变化/消失】 -> 回声确实经过舵机那根线,舵机在总线上但不应答
    """
    print(f"实时指纹 port={port} baud={baud} 持续 {seconds}s(可中途拔插舵机 3P 线)")
    print("  Ctrl-C 结束\n")
    frame = b"#%03dPRAD!" % ids[0]
    n = 0
    t_start = time.time()
    with serial.Serial(port, baud, timeout=0) as ser:
        deadline = t_start + seconds
        while time.time() < deadline:
            n += 1
            ser.reset_input_buffer()
            ser.write(frame)
            ser.flush()
            time.sleep(0.05)
            echo = ser.read(4096)
            time.sleep(0.1)
            extra = ser.read(4096)
            fp = echo.hex(" ") if echo else "(无回声)"
            tag = " <<< 有舵机应答!" if "舵机应答" in classify(extra) else ""
            print(f"  [{time.time()-t_start:6.1f}s #{n:>4}] 回声 {len(echo):>2}B: {fp}{tag}")
            if extra:
                print(f"         后续: {extra!r}{tag}")
            time.sleep(0.2)


def parse_ids(values) -> list[int]:
    return [int(v) for v in values]


def main() -> int:
    ap = argparse.ArgumentParser(description="串行总线夹爪舵机通信探测")
    ap.add_argument("port", help="串口设备,如 /dev/ttyUSB1")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--scan-ids", nargs="+", default=["0", "1", "2", "3"])
    ap.add_argument("--reply-window", type=float, default=0.5, help="每次查询等待应答的秒数")
    ap.add_argument("--raw", action="store_true", help="打印每次交换的原始字节")
    ap.add_argument("--baud-sweep", action="store_true")
    ap.add_argument("--binary-ping", action="store_true")
    ap.add_argument("--listen", type=float, default=0.0, help="只监听该秒数,不发送")
    ap.add_argument("--watch", type=float, default=0.0, help="持续查询该秒数")
    ap.add_argument("--fingerprint", type=float, default=0.0,
                    help="实时打印回声指纹该秒数,用于拔插接线诊断")
    args = ap.parse_args()

    ids = parse_ids(args.scan_ids)

    if args.fingerprint:
        fingerprint(args.port, args.baud, args.fingerprint, ids)
        return 0

    if args.listen:
        listen(args.port, args.baud, args.listen)
        return 0

    if args.watch:
        return 0 if watch(args.port, args.baud, args.watch, ids) else 1

    found = scan_ids(args.port, args.baud, ids, args.reply_window, args.raw)
    if args.baud_sweep:
        found = sweep_bauds(args.port, DEFAULT_BAUDS, ids, args.reply_window) or found
    if args.binary_ping:
        found = binary_ping(args.port, DEFAULT_BAUDS[:4], [1, 0, 0xFE]) or found

    print("\n== 结论 ==")
    if found:
        print("  检测到舵机应答,串口通信成功。")
        return 0
    print("  未检测到任何舵机应答:总线只有自身回声,舵机侧无响应。")
    print("  按 docs/dual_gripper_integration.md 第 2 节检查:")
    print("    1) 舵机独立 5~8.4V 供电(LED 是否亮),不要靠 USB 供电")
    print("    2) 调试板 GND / 舵机 GND / 电源负极 共地")
    print("    3) 单总线信号线是否接到舵机信号脚")
    print("    4) 舵机 ID 是否已设为 1(出厂默认多为 0)")
    print("    5) 是否用的厂家半双工调试板,而非普通 CH340 全双工适配器")
    return 1


if __name__ == "__main__":
    sys.exit(main())
