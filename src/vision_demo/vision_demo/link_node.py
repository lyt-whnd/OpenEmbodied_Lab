#!/usr/bin/env python3
"""ROS 2 owner of the process-wide ESP32 RobotLink connection."""

import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt8MultiArray

from vision_demo.protocol_v1 import (
    ApplicationMessage,
    MessageFlag,
    ProtocolError,
    ServiceId,
    SystemOpcode,
)
from vision_demo.robot_link import RobotLink, SendPolicy
from vision_demo.transport.websocket import WebSocketTransport


class RobotLinkNode(Node):
    """Bridge bounded ROS packet topics to one full-duplex WebSocket."""

    def __init__(self):
        """Initialize the sole link and ROS-facing packet topics."""
        super().__init__('robot_link_node')

        self.declare_parameter(
            'websocket_url',
            'ws://192.168.1.100/ws',
        )
        self.declare_parameter('reconnect_delay_sec', 1.0)
        self.declare_parameter('send_queue_size', 8)
        self.declare_parameter(
            'command_topic',
            '/robot_link/tx',
        )
        self.declare_parameter(
            'receive_topic',
            '/robot_link/rx',
        )

        websocket_url = str(
            self.get_parameter('websocket_url').value
        )
        reconnect_delay_sec = float(
            self.get_parameter('reconnect_delay_sec').value
        )
        send_queue_size = int(
            self.get_parameter('send_queue_size').value
        )
        command_topic = str(
            self.get_parameter('command_topic').value
        )
        receive_topic = str(
            self.get_parameter('receive_topic').value
        )

        self.receive_publisher = self.create_publisher(
            UInt8MultiArray,
            receive_topic,
            10,
        )
        self.command_subscription = self.create_subscription(
            UInt8MultiArray,
            command_topic,
            self._command_callback,
            10,
        )

        self.link = RobotLink(
            transport_factory=lambda: WebSocketTransport(
                websocket_url,
            ),
            reconnect_delay_sec=reconnect_delay_sec,
            queue_size=send_queue_size,
        )
        self.link.register_observer(self._received_message)
        self.link.register_handler(
            int(ServiceId.SYSTEM),
            int(SystemOpcode.PONG),
            self._pong_received,
        )
        self.link.start()

        self.get_logger().info(
            f'RobotLink started: {websocket_url}; '
            f'tx={command_topic}, rx={receive_topic}, '
            f'queue={send_queue_size}'
        )

    def _command_callback(self, ros_message: UInt8MultiArray) -> None:
        try:
            packet = bytes(ros_message.data)
            message = ApplicationMessage.decode(packet)
        except (ProtocolError, TypeError, ValueError) as error:
            self.get_logger().warning(
                f'Rejected outbound V1 packet: {error}'
            )
            return

        policy = SendPolicy.FIFO

        if message.flags & int(MessageFlag.REALTIME):
            policy = SendPolicy.BEST_EFFORT

        if not self.link.send_packet(packet, policy):
            self.get_logger().warning(
                'RobotLink outbound FIFO is full'
            )

    def _received_message(
        self,
        message: ApplicationMessage,
        packet: bytes,
    ) -> None:
        ros_message = UInt8MultiArray()
        ros_message.data = list(packet)
        self.receive_publisher.publish(ros_message)

    def _pong_received(
        self,
        message: ApplicationMessage,
        packet: bytes,
    ) -> None:
        del packet
        self.get_logger().debug(
            f'RobotLink PONG seq={message.seq}, src={message.src}'
        )

    def destroy_node(self):
        """Stop the connection worker before destroying ROS resources."""
        self.link.stop()
        return super().destroy_node()


def main(args=None):
    """Run the process-wide RobotLink node."""
    rclpy.init(args=args)
    node = RobotLinkNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
