#!/usr/bin/env python3
"""
SlamWalker 步行机构里程计标定工具

标定流程：
  第 1 步: 遥控机器人走一段直线，量实际距离 → 得到 ticks_per_meter
  第 2 步: 遥控机器人原地转一圈，量实际角度 → 得到 effective_wheel_base

用法:
  1. 先在另一个终端启动 serial_bridge_node 和 teleop
  2. 运行本脚本: python3 ~/walker_ws/scripts/calibrate.py
  3. 按提示操作
"""
import serial
import time
import os
import select
import errno
import threading

PORT = '/dev/arduino'
BAUD = 115200

def open_serial():
    s = serial.Serial(PORT, BAUD, timeout=0, dsrdtr=False, rtscts=False)
    time.sleep(2)
    s.reset_input_buffer()
    return s

class TickRecorder:
    def __init__(self, ser):
        self.ser = ser
        self.total_L = 0
        self.total_R = 0
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        self.total_L = 0
        self.total_R = 0
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)

    def _run(self):
        buf = b''
        while not self._stop.is_set():
            try:
                ready, _, _ = select.select([self.ser.fileno()], [], [], 0.2)
                if not ready:
                    continue
                chunk = os.read(self.ser.fileno(), 512)
                if not chunk:
                    continue
                buf += chunk
                while b'\n' in buf:
                    lb, buf = buf.split(b'\n', 1)
                    line = lb.decode(errors='replace').strip()
                    if line.startswith('O'):
                        try:
                            parts = line[1:].split(',')
                            self.total_L += int(parts[0])
                            self.total_R += int(parts[1])
                        except (ValueError, IndexError):
                            pass
            except OSError as e:
                if e.errno == errno.EAGAIN:
                    time.sleep(0.01)
                    continue
                break

def main():
    print("=" * 55)
    print("   SlamWalker 步行机构里程计标定")
    print("=" * 55)
    print()
    print("  前提: 另一个终端已启动 serial_bridge + teleop")
    print("  本工具只监听编码器数据，不会发送控制命令。")
    print()

    # ── Step 1: 直线标定 ──────────────────────────────
    print("【第 1 步：直线标定】")
    print("  在地上标记起点。用遥控让机器人走一段直线。")
    print("  走完后量一下实际走了多远（米）。")
    input("\n  准备好后按 Enter 开始记录...")

    ser = open_serial()
    rec = TickRecorder(ser)
    rec.start()

    print("\n  >>> 正在记录编码器 — 现在遥控机器人直行！")
    input("  >>> 走完后按 Enter 停止记录...\n")
    rec.stop()

    tL, tR = rec.total_L, rec.total_R
    avg_ticks = (abs(tL) + abs(tR)) / 2.0
    print(f"  编码器: 左={tL}, 右={tR}, 平均={avg_ticks:.0f}")

    actual_dist = 0
    while True:
        try:
            actual_dist = float(input("  请输入实际行走距离（米，例如 1.0）: "))
            if actual_dist > 0:
                break
            print("  距离必须 > 0")
        except ValueError:
            print("  请输入数字")

    ticks_per_meter = avg_ticks / actual_dist
    print(f"\n  ★ ticks_per_meter = {ticks_per_meter:.1f}")

    # ── Step 2: 转弯标定 ──────────────────────────────
    print("\n" + "=" * 55)
    print("【第 2 步：原地旋转标定】")
    print("  让机器人原地旋转（建议转完整一圈 360°）。")
    input("\n  准备好后按 Enter 开始记录...")

    rec.start()
    print("\n  >>> 正在记录 — 现在遥控机器人原地转圈！")
    input("  >>> 转完后按 Enter 停止记录...\n")
    rec.stop()

    tL2, tR2 = rec.total_L, rec.total_R
    print(f"  编码器: 左={tL2}, 右={tR2}")

    actual_angle = 0
    while True:
        try:
            actual_angle = float(input("  请输入实际旋转角度（度，例如 360）: "))
            if actual_angle > 0:
                break
            print("  角度必须 > 0")
        except ValueError:
            print("  请输入数字")

    actual_rad = actual_angle * 3.14159265 / 180.0
    # For in-place rotation: one side goes forward, the other backward
    # d_theta = (d_right - d_left) / wheel_base
    # |d_right - d_left| in meters = |tR - tL| / ticks_per_meter
    tick_diff_meters = abs(tR2 - tL2) / ticks_per_meter
    if actual_rad > 0 and tick_diff_meters > 0:
        effective_base = tick_diff_meters / actual_rad
        print(f"\n  ★ effective_wheel_base = {effective_base:.4f} m")
    else:
        effective_base = None
        print("  数据不足，跳过")

    ser.close()

    # ── Summary ───────────────────────────────────────
    print("\n" + "=" * 55)
    print("   标定结果")
    print("=" * 55)
    print(f"  ticks_per_meter    = {ticks_per_meter:.1f}")
    if effective_base:
        print(f"  effective_wheel_base = {effective_base:.4f} m")
    print()
    print("  使用方法（启动 serial_bridge_node 时传参）:")
    print(f"    -p ticks_per_meter:={ticks_per_meter:.1f}")
    if effective_base:
        print(f"    -p wheel_base:={effective_base:.4f}")
    print()
    print("  例如:")
    cmd = f"ros2 run slamwalker_bridge serial_bridge_node --ros-args"
    cmd += f" -p ticks_per_meter:={ticks_per_meter:.1f}"
    if effective_base:
        cmd += f" -p wheel_base:={effective_base:.4f}"
    print(f"    {cmd}")
    print()


if __name__ == '__main__':
    main()
