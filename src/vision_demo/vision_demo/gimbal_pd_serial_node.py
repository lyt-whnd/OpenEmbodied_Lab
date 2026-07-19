#!/usr/bin/env python3

import time
import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class GimbalPDSerialNode(Node):
    def __init__(self):
        super().__init__('gimbal_pd_serial_node')

        self.declare_parameter('target_topic', '/target_position')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        self.declare_parameter('control_hz', 10.0)

        self.declare_parameter('kp_x', 0.015)
        self.declare_parameter('kd_x', 0.006)

        self.declare_parameter('kp_y', 0.015)
        self.declare_parameter('kd_y', 0.006)

        self.declare_parameter('dead_zone_x', 30.0)
        self.declare_parameter('dead_zone_y', 30.0)

        self.declare_parameter('max_step', 3)
        self.declare_parameter('target_lost_area', 1.0)

        self.target_topic = self.get_parameter(
            'target_topic'
        ).get_parameter_value().string_value

        self.serial_port = self.get_parameter(
            'serial_port'
        ).get_parameter_value().string_value

        self.baudrate = self.get_parameter(
            'baudrate'
        ).get_parameter_value().integer_value

        self.control_hz = self.get_parameter(
            'control_hz'
        ).get_parameter_value().double_value

        self.kp_x = self.get_parameter(
            'kp_x'
        ).get_parameter_value().double_value

        self.kd_x = self.get_parameter(
            'kd_x'
        ).get_parameter_value().double_value

        self.kp_y = self.get_parameter(
            'kp_y'
        ).get_parameter_value().double_value

        self.kd_y = self.get_parameter(
            'kd_y'
        ).get_parameter_value().double_value

        self.dead_zone_x = self.get_parameter(
            'dead_zone_x'
        ).get_parameter_value().double_value

        self.dead_zone_y = self.get_parameter(
            'dead_zone_y'
        ).get_parameter_value().double_value

        self.max_step = self.get_parameter(
            'max_step'
        ).get_parameter_value().integer_value

        self.target_lost_area = self.get_parameter(
            'target_lost_area'
        ).get_parameter_value().double_value

        self.error_x = 0.0
        self.error_y = 0.0
        self.target_area = 0.0

        self.prev_error_x = 0.0
        self.prev_error_y = 0.0

        self.has_target_msg = False
        self.last_target_time = self.get_clock().now()

        self.ser = serial.Serial(
            port=self.serial_port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
        )

        time.sleep(1.0)

        self.sub = self.create_subscription(
            Point,
            self.target_topic,
            self.target_callback,
            10
        )

        self.timer = self.create_timer(
            1.0 / self.control_hz,
            self.control_loop
        )

        self.get_logger().info(
            f'gimbal_pd_serial_node started: {self.serial_port}, {self.baudrate}'
        )

    def target_callback(self, msg: Point):
        self.error_x = msg.x
        self.error_y = msg.y
        self.target_area = msg.z
        self.has_target_msg = True
        self.last_target_time = self.get_clock().now()

    def control_loop(self):
        if not self.has_target_msg:
            return

        now = self.get_clock().now()
        dt_msg = (now - self.last_target_time).nanoseconds / 1e9

        if dt_msg > 0.5:
            self.send_line('#STOP')
            return

        if self.target_area < self.target_lost_area:
            self.send_line('#STOP')
            return

        ex = self.error_x
        ey = self.error_y

        if abs(ex) < self.dead_zone_x:
            ex = 0.0

        if abs(ey) < self.dead_zone_y:
            ey = 0.0

        dt = 1.0 / self.control_hz

        dx = (ex - self.prev_error_x) / dt
        dy = (ey - self.prev_error_y) / dt

        self.prev_error_x = ex
        self.prev_error_y = ey

        if ex == 0.0 and ey == 0.0:
            self.send_line('#STOP')
            return

        control_x = self.kp_x * ex + self.kd_x * dx
        control_y = self.kp_y * ey + self.kd_y * dy

        # 图像坐标：
        # error_x > 0 目标在右边
        # error_y > 0 目标在下边
        #
        # 你的云台方向：
        # yaw 角度减少 -> 向右
        # pitch 角度增加 -> 向下
        #
        # 所以：
        # dyaw   = -control_x
        # dpitch =  control_y

        dyaw = int(round(-control_x))
        dpitch = int(round(control_y))

        dyaw = self.clamp(dyaw, -self.max_step, self.max_step)
        dpitch = self.clamp(dpitch, -self.max_step, self.max_step)

        if dyaw == 0 and ex != 0.0:
            dyaw = -1 if ex > 0 else 1

        if dpitch == 0 and ey != 0.0:
            dpitch = 1 if ey > 0 else -1

        self.send_line(f'#MOVE,{dyaw},{dpitch}')

    def send_line(self, line: str):
        try:
            self.ser.write((line + '\r\n').encode('utf-8'))
            self.ser.flush()

            resp = self.ser.readline().decode(
                'utf-8',
                errors='ignore'
            ).strip()

            if resp:
                self.get_logger().info(f'TX: {line} | RX: {resp}')
            else:
                self.get_logger().info(f'TX: {line}')

        except Exception as e:
            self.get_logger().error(f'Serial error: {e}')

    @staticmethod
    def clamp(value, min_value, max_value):
        if value < min_value:
            return min_value
        if value > max_value:
            return max_value
        return value

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = GimbalPDSerialNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()