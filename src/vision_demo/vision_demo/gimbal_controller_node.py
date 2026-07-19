#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class GimbalControllerNode(Node):
    def __init__(self):
        super().__init__('gimbal_controller_node')

        self.declare_parameter('target_topic', '/target_position')
        self.declare_parameter('cmd_topic', '/gimbal_cmd')

        self.declare_parameter('kp_x', 0.002)
        self.declare_parameter('kp_y', 0.002)

        self.declare_parameter('dead_zone_x', 30.0)
        self.declare_parameter('dead_zone_y', 30.0)

        self.declare_parameter('max_yaw_cmd', 0.5)
        self.declare_parameter('max_pitch_cmd', 0.5)

        self.target_topic = self.get_parameter(
            'target_topic'
        ).get_parameter_value().string_value

        self.cmd_topic = self.get_parameter(
            'cmd_topic'
        ).get_parameter_value().string_value

        self.kp_x = self.get_parameter(
            'kp_x'
        ).get_parameter_value().double_value

        self.kp_y = self.get_parameter(
            'kp_y'
        ).get_parameter_value().double_value

        self.dead_zone_x = self.get_parameter(
            'dead_zone_x'
        ).get_parameter_value().double_value

        self.dead_zone_y = self.get_parameter(
            'dead_zone_y'
        ).get_parameter_value().double_value

        self.max_yaw_cmd = self.get_parameter(
            'max_yaw_cmd'
        ).get_parameter_value().double_value

        self.max_pitch_cmd = self.get_parameter(
            'max_pitch_cmd'
        ).get_parameter_value().double_value

        self.subscription = self.create_subscription(
            Point,
            self.target_topic,
            self.target_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Point,
            self.cmd_topic,
            10
        )

        self.get_logger().info(
            f'Gimbal controller started. '
            f'sub: {self.target_topic}, pub: {self.cmd_topic}'
        )

    def target_callback(self, msg: Point):
        error_x = msg.x
        error_y = msg.y
        area = msg.z

        yaw_cmd = 0.0
        pitch_cmd = 0.0

        # area <= 0 表示目标丢失
        if area <= 0.0:
            self.publish_cmd(0.0, 0.0)
            self.get_logger().info(
                'target lost, gimbal stop'
            )
            return

        # 水平方向死区
        if abs(error_x) > self.dead_zone_x:
            yaw_cmd = self.kp_x * error_x

        # 垂直方向死区
        if abs(error_y) > self.dead_zone_y:
            pitch_cmd = self.kp_y * error_y

        # 限幅，防止命令太大
        yaw_cmd = self.clamp(
            yaw_cmd,
            -self.max_yaw_cmd,
            self.max_yaw_cmd
        )

        pitch_cmd = self.clamp(
            pitch_cmd,
            -self.max_pitch_cmd,
            self.max_pitch_cmd
        )

        self.publish_cmd(yaw_cmd, pitch_cmd)

        self.get_logger().info(
            f'error=({error_x:.1f}, {error_y:.1f}), '
            f'area={area:.0f}, '
            f'cmd=({yaw_cmd:.3f}, {pitch_cmd:.3f})'
        )

    def publish_cmd(self, yaw_cmd, pitch_cmd):
        cmd = Point()
        cmd.x = float(yaw_cmd)
        cmd.y = float(pitch_cmd)
        cmd.z = 0.0

        self.cmd_pub.publish(cmd)

    @staticmethod
    def clamp(value, min_value, max_value):
        if value < min_value:
            return min_value
        if value > max_value:
            return max_value
        return value


def main(args=None):
    rclpy.init(args=args)

    node = GimbalControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()