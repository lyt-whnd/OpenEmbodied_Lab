"""ROS 2 wrapper that separates generic telemetry into per-sensor topics."""

import struct
from typing import Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

from vision_demo.protocol_v1 import (
    ApplicationMessage,
    ProtocolError,
    ServiceId,
    TelemetryOpcode,
)
from vision_demo.sample_block import decode_sample_block
from vision_demo.telemetry import decode_telemetry_batch


class SensorBridgeNode(Node):
    """Publish low-rate state and high-rate samples on distinct topics."""

    def __init__(self):
        super().__init__('sensor_bridge')
        self._publishers: Dict[str, object] = {}
        self.create_subscription(
            UInt8MultiArray,
            '/robot_link/rx',
            self._received,
            10,
        )

    def _publisher(self, topic: str):
        publisher = self._publishers.get(topic)
        if publisher is None:
            publisher = self.create_publisher(
                UInt8MultiArray,
                topic,
                10,
            )
            self._publishers[topic] = publisher
        return publisher

    def _publish(self, topic: str, payload: bytes) -> None:
        message = UInt8MultiArray()
        message.data = list(payload)
        self._publisher(topic).publish(message)

    def _received(self, raw: UInt8MultiArray) -> None:
        try:
            message = ApplicationMessage.decode(bytes(raw.data))
            if message.service != int(ServiceId.TELEMETRY):
                return
            if message.opcode == int(TelemetryOpcode.BATCH):
                for record in decode_telemetry_batch(message.payload):
                    topic = (
                        f'/robot/sensors/'
                        f'{record.sensor_type:04x}/'
                        f'{record.instance_id}'
                    )
                    envelope = struct.pack(
                        '<BIH',
                        record.schema_version,
                        record.timestamp_ms,
                        record.sample_seq,
                    ) + record.data
                    self._publish(topic, envelope)
            elif message.opcode == int(
                TelemetryOpcode.SAMPLE_BLOCK
            ):
                block = decode_sample_block(message.payload)
                topic = (
                    f'/robot/samples/'
                    f'{block.sensor_type:04x}/'
                    f'{block.instance_id}'
                )
                for timestamp, sample in zip(
                    block.timestamps_ms,
                    block.samples,
                ):
                    envelope = struct.pack(
                        '<BIH',
                        block.schema_version,
                        timestamp,
                        block.drop_count,
                    ) + sample
                    self._publish(topic, envelope)
        except (ProtocolError, ValueError, TypeError) as error:
            self.get_logger().warning(
                f'rejected telemetry payload: {error}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = SensorBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
