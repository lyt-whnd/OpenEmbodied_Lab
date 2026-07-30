"""Map low-rate records and high-rate blocks to independent topic names."""

from typing import Callable

from vision_demo.sample_block import SampleBlock, SampleBlockDispatcher
from vision_demo.telemetry import (
    ReceivedSensorRecord,
    TelemetryDispatcher,
)


TopicPublisher = Callable[[str, bytes, int], None]


class SensorTopicBridge:
    """Transport-neutral core used by a future ROS 2 publisher wrapper."""

    def __init__(self, publish: TopicPublisher):
        self.telemetry = TelemetryDispatcher()
        self.samples = SampleBlockDispatcher()
        self._publish = publish

    @staticmethod
    def state_topic(sensor_type: int, instance_id: int) -> str:
        return f'/robot/sensors/{sensor_type:04x}/{instance_id}'

    @staticmethod
    def sample_topic(sensor_type: int, instance_id: int) -> str:
        return f'/robot/samples/{sensor_type:04x}/{instance_id}'

    def register_state(
        self,
        sensor_type: int,
        instance_id: int,
        schema_version: int,
    ) -> None:
        def forward(received: ReceivedSensorRecord) -> None:
            self._publish(
                self.state_topic(sensor_type, instance_id),
                received.record.data,
                received.record.timestamp_ms,
            )

        self.telemetry.register(
            (sensor_type, instance_id, schema_version),
            forward,
        )

    def register_samples(
        self,
        sensor_type: int,
        instance_id: int,
        schema_version: int,
    ) -> None:
        def forward(block: SampleBlock) -> None:
            for timestamp, sample in zip(
                block.timestamps_ms,
                block.samples,
            ):
                self._publish(
                    self.sample_topic(sensor_type, instance_id),
                    sample,
                    timestamp,
                )

        self.samples.register(
            (sensor_type, instance_id, schema_version),
            forward,
        )
