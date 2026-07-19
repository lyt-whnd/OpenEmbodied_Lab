#!/usr/bin/env python3

import time
import serial


PORT = "/dev/ttyUSB0"
BAUDRATE = 115200


def send_cmd(ser, cmd, delay=1.0):
    print(f"\nTX: {cmd}")
    ser.write((cmd + "\r\n").encode("utf-8"))
    ser.flush()

    resp = ser.readline().decode(errors="ignore").strip()
    print("RX:", resp if resp else "<no response>")

    time.sleep(delay)


def wait_user(text):
    input(f"\n{text}\n按 Enter 继续...")


def main():
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    time.sleep(1)

    print("=== Gimbal Direction Test ===")
    print("请看云台实际运动方向。")
    print("理论方向：")
    print("yaw 角度减少 -> 向右")
    print("yaw 角度增加 -> 向左")
    print("pitch 角度减少 -> 向上")
    print("pitch 角度增加 -> 向下")

    send_cmd(ser, "#CENTER", 1.5)
    send_cmd(ser, "#GET", 0.5)

    wait_user("测试 1：yaw 减少，应该向右")
    send_cmd(ser, "#MOVE,-20,0", 1.5)
    send_cmd(ser, "#GET", 0.5)

    wait_user("回中")
    send_cmd(ser, "#CENTER", 1.5)

    wait_user("测试 2：yaw 增加，应该向左")
    send_cmd(ser, "#MOVE,20,0", 1.5)
    send_cmd(ser, "#GET", 0.5)

    wait_user("回中")
    send_cmd(ser, "#CENTER", 1.5)

    wait_user("测试 3：pitch 减少，应该向上")
    send_cmd(ser, "#MOVE,0,-20", 1.5)
    send_cmd(ser, "#GET", 0.5)

    wait_user("回中")
    send_cmd(ser, "#CENTER", 1.5)

    wait_user("测试 4：pitch 增加，应该向下")
    send_cmd(ser, "#MOVE,0,20", 1.5)
    send_cmd(ser, "#GET", 0.5)

    wait_user("最后回中")
    send_cmd(ser, "#CENTER", 1.5)
    send_cmd(ser, "#GET", 0.5)

    ser.close()
    print("\n测试结束。")


if __name__ == "__main__":
    main()