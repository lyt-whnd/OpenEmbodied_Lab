#!/usr/bin/env python3
"""ROS 2 PD controller using the robot V1 binary WebSocket protocol."""

import queue
import threading
import time

from geometry_msgs.msg import Point

import rclpy
from rclpy.node import Node

from vision_demo.protocol_v1 import (
    ApplicationMessage,
    MessageFlag,
    MotionMovePayload,
    MotionOpcode,
    NodeId,
    SequenceGenerator,
    ServiceId,
)

import websocket


class GimbalPDWebSocketNode(Node):
    """
    Send PD motion output using binary V1 application messages.

    The current controller fills the head-rate fields and leaves the tracked
    base velocity fields at zero.
    """

    def __init__(self):
        """Initialize parameters, ROS callbacks, and the socket worker."""
        super().__init__(
            'gimbal_pd_websocket_node'
        )

        self.declare_parameter(
            'target_topic',
            '/target_position',
        )
        self.declare_parameter(
            'websocket_url',
            'ws://192.168.1.100/ws',
        )

        self.declare_parameter(
            'control_hz',
            10.0,
        )

        self.declare_parameter('kp_x', 0.015)
        self.declare_parameter('kd_x', 0.006)
        self.declare_parameter('kp_y', 0.015)
        self.declare_parameter('kd_y', 0.006)

        self.declare_parameter(
            'dead_zone_x',
            30.0,
        )
        self.declare_parameter(
            'dead_zone_y',
            30.0,
        )

        self.declare_parameter(
            'max_step',
            3,
        )
        self.declare_parameter(
            'target_lost_area',
            1.0,
        )
        self.declare_parameter(
            'target_timeout_sec',
            0.5,
        )
        self.declare_parameter(
            'command_refresh_sec',
            0.5,
        )
        self.declare_parameter(
            'reconnect_delay_sec',
            2.0,
        )
        self.declare_parameter(
            'motion_valid_ms',
            300,
        )
        self.declare_parameter(
            'control_epoch',
            1,
        )

        self.target_topic = str(
            self.get_parameter(
                'target_topic'
            ).value
        )
        self.websocket_url = str(
            self.get_parameter(
                'websocket_url'
            ).value
        )

        self.control_hz = float(
            self.get_parameter(
                'control_hz'
            ).value
        )

        self.kp_x = float(
            self.get_parameter('kp_x').value
        )
        self.kd_x = float(
            self.get_parameter('kd_x').value
        )
        self.kp_y = float(
            self.get_parameter('kp_y').value
        )
        self.kd_y = float(
            self.get_parameter('kd_y').value
        )

        self.dead_zone_x = float(
            self.get_parameter(
                'dead_zone_x'
            ).value
        )
        self.dead_zone_y = float(
            self.get_parameter(
                'dead_zone_y'
            ).value
        )

        self.max_step = int(
            self.get_parameter(
                'max_step'
            ).value
        )
        self.target_lost_area = float(
            self.get_parameter(
                'target_lost_area'
            ).value
        )
        self.target_timeout_sec = float(
            self.get_parameter(
                'target_timeout_sec'
            ).value
        )
        self.command_refresh_sec = float(
            self.get_parameter(
                'command_refresh_sec'
            ).value
        )
        self.reconnect_delay_sec = float(
            self.get_parameter(
                'reconnect_delay_sec'
            ).value
        )
        self.motion_valid_ms = int(
            self.get_parameter(
                'motion_valid_ms'
            ).value
        )
        self.control_epoch = int(
            self.get_parameter(
                'control_epoch'
            ).value
        )

        if self.control_hz <= 0.0:
            raise ValueError(
                'control_hz must be greater than 0'
            )

        if not 1 <= self.motion_valid_ms <= 0xFFFF:
            raise ValueError(
                'motion_valid_ms must be in [1, 65535]'
            )

        if not 0 <= self.control_epoch <= 0xFFFF:
            raise ValueError(
                'control_epoch must be in [0, 65535]'
            )

        self.error_x = 0.0
        self.error_y = 0.0
        self.target_area = 0.0

        self.prev_error_x = 0.0
        self.prev_error_y = 0.0

        self.has_target_message = False
        self.last_target_time = (
            self.get_clock().now()
        )

        self.last_queued_command = None
        self.last_queue_time = 0.0

        # 队列长度为 1，新命令覆盖尚未发送的旧命令。
        self.command_queue = queue.Queue(
            maxsize=1
        )

        self.sequence = SequenceGenerator()
        self.stop_event = threading.Event()

        self.subscription = (
            self.create_subscription(
                Point,
                self.target_topic,
                self.target_callback,
                10,
            )
        )

        self.control_timer = self.create_timer(
            1.0 / self.control_hz,
            self.control_loop,
        )

        self.websocket_thread = (
            threading.Thread(
                target=self.websocket_loop,
                name='esp32_websocket_client',
                daemon=True,
            )
        )

        self.websocket_thread.start()

        self.get_logger().info(
            f'PD V1 WebSocket node started: '
            f'{self.target_topic} -> '
            f'{self.websocket_url}'
        )

    def target_callback(self, message):
        """Store the most recently observed image-space target error."""
        self.error_x = float(message.x)
        self.error_y = float(message.y)
        self.target_area = float(message.z)

        self.has_target_message = True
        self.last_target_time = (
            self.get_clock().now()
        )

    def control_loop(self):
        """Generate and queue the latest control command."""
        command = self.build_command()
        now = time.monotonic()

        command_unchanged = (
            command
            == self.last_queued_command
        )

        refresh_not_due = (
            now - self.last_queue_time
            < self.command_refresh_sec
        )

        if (
            command_unchanged
            and refresh_not_due
        ):
            return

        self.queue_latest_command(command)

        self.last_queued_command = command
        self.last_queue_time = now

    def build_command(self):
        """Build one V1 motion opcode and payload from the PD state."""
        if not self.has_target_message:
            self.reset_pd_state()
            return self.stop_command()

        target_age = (
            self.get_clock().now()
            - self.last_target_time
        ).nanoseconds / 1e9

        if target_age > self.target_timeout_sec:
            self.reset_pd_state()
            return self.stop_command()

        if (
            self.target_area
            < self.target_lost_area
        ):
            self.reset_pd_state()
            return self.stop_command()

        error_x = self.error_x
        error_y = self.error_y

        if abs(error_x) < self.dead_zone_x:
            error_x = 0.0

        if abs(error_y) < self.dead_zone_y:
            error_y = 0.0

        dt = 1.0 / self.control_hz

        derivative_x = (
            error_x - self.prev_error_x
        ) / dt

        derivative_y = (
            error_y - self.prev_error_y
        ) / dt

        self.prev_error_x = error_x
        self.prev_error_y = error_y

        if error_x == 0.0 and error_y == 0.0:
            return self.stop_command()

        control_x = (
            self.kp_x * error_x
            + self.kd_x * derivative_x
        )

        control_y = (
            self.kp_y * error_y
            + self.kd_y * derivative_y
        )

        # 与旧串口节点保持相同方向：
        #
        # error_x > 0：目标在画面右侧
        # yaw 角度应减小
        #
        # error_y > 0：目标在画面下侧
        # pitch 角度应增大
        yaw_step = int(round(-control_x))
        pitch_step = int(round(control_y))

        yaw_step = self.clamp(
            yaw_step,
            -self.max_step,
            self.max_step,
        )

        pitch_step = self.clamp(
            pitch_step,
            -self.max_step,
            self.max_step,
        )

        if (
            yaw_step == 0
            and error_x != 0.0
        ):
            yaw_step = (
                -1 if error_x > 0 else 1
            )

        if (
            pitch_step == 0
            and error_y != 0.0
        ):
            pitch_step = (
                1 if error_y > 0 else -1
            )

        head_yaw_rate_x10 = self.clamp(
            int(
                round(
                    yaw_step
                    * self.control_hz
                    * 10.0
                )
            ),
            -0x8000,
            0x7FFF,
        )
        head_pitch_rate_x10 = self.clamp(
            int(
                round(
                    pitch_step
                    * self.control_hz
                    * 10.0
                )
            ),
            -0x8000,
            0x7FFF,
        )

        payload = MotionMovePayload(
            control_epoch=self.control_epoch,
            valid_ms=self.motion_valid_ms,
            linear_mm_s=0,
            angular_mrad_s=0,
            head_yaw_rate_x10=head_yaw_rate_x10,
            head_pitch_rate_x10=head_pitch_rate_x10,
        ).encode()

        return (
            MotionOpcode.MOVE,
            payload,
        )

    def reset_pd_state(self):
        """Reset derivative history when target tracking stops."""
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0

    def queue_latest_command(self, command):
        """Keep only the latest pending real-time motion command."""
        try:
            self.command_queue.put_nowait(
                command
            )
            return
        except queue.Full:
            pass

        try:
            self.command_queue.get_nowait()
        except queue.Empty:
            pass

        try:
            self.command_queue.put_nowait(
                command
            )
        except queue.Full:
            pass

    def websocket_loop(self):
        """Maintain the connection and send the latest binary command."""
        connection = None
        last_sent_command = None

        while not self.stop_event.is_set():
            if connection is None:
                try:
                    connection = (
                        websocket.create_connection(
                            self.websocket_url,
                            timeout=2.0,
                            enable_multithread=True,
                        )
                    )

                    stop_command = self.stop_command()
                    stop_seq = self.send_command(
                        connection,
                        stop_command,
                    )

                    last_sent_command = stop_command

                    self.get_logger().info(
                        f'WebSocket connected: '
                        f'{self.websocket_url}; '
                        f'initial STOP seq={stop_seq}'
                    )

                except Exception as error:
                    self.get_logger().warning(
                        f'WebSocket connect failed: '
                        f'{error}'
                    )

                    self.close_connection(
                        connection
                    )
                    connection = None

                    self.stop_event.wait(
                        self.reconnect_delay_sec
                    )
                    continue

            try:
                command = (
                    self.command_queue.get(
                        timeout=0.1
                    )
                )
            except queue.Empty:
                continue

            try:
                seq = self.send_command(
                    connection,
                    command,
                )

                if command != last_sent_command:
                    self.get_logger().info(
                        f'WS V1 TX: '
                        f'{self.describe_command(command)}, '
                        f'seq={seq}'
                    )

                last_sent_command = command

            except Exception as error:
                self.get_logger().warning(
                    f'WebSocket send failed: '
                    f'{error}'
                )

                self.close_connection(
                    connection
                )
                connection = None
                last_sent_command = None

                # 发送失败时保留最新命令，等待重连。
                self.queue_latest_command(
                    command
                )

        if connection is not None:
            try:
                self.send_command(
                    connection,
                    self.stop_command(),
                )
            except Exception:
                pass

            self.close_connection(connection)

    def send_command(self, connection, command):
        """Encode and send one binary V1 motion message."""
        opcode, payload = command

        flags = MessageFlag.REALTIME

        if opcode != MotionOpcode.MOVE:
            flags = MessageFlag.ACK_REQUIRED

        seq = self.sequence.next_seq()

        message = ApplicationMessage(
            flags=int(flags),
            src=int(NodeId.LINUX),
            dst=int(NodeId.STM32),
            service=int(ServiceId.MOTION),
            opcode=int(opcode),
            seq=seq,
            payload=payload,
        )

        connection.send_binary(
            message.encode()
        )

        return seq

    @staticmethod
    def stop_command():
        """Return the canonical normal-stop command."""
        return (
            MotionOpcode.STOP,
            b'',
        )

    @staticmethod
    def describe_command(command):
        """Return a concise description for logs."""
        opcode, payload = command

        if opcode == MotionOpcode.STOP:
            return 'STOP'

        if opcode != MotionOpcode.MOVE:
            return f'opcode=0x{int(opcode):02X}'

        move = MotionMovePayload.decode(payload)

        return (
            'MOVE '
            f'linear={move.linear_mm_s}mm/s, '
            f'angular={move.angular_mrad_s}mrad/s, '
            f'head_yaw={move.head_yaw_rate_x10 / 10.0:.1f}deg/s, '
            f'head_pitch={move.head_pitch_rate_x10 / 10.0:.1f}deg/s'
        )

    @staticmethod
    def close_connection(connection):
        """Close a WebSocket connection without masking shutdown."""
        if connection is None:
            return

        try:
            connection.close()
        except Exception:
            pass

    @staticmethod
    def clamp(value, minimum, maximum):
        """Clamp a numeric value to an inclusive range."""
        return max(
            minimum,
            min(value, maximum),
        )

    def destroy_node(self):
        """Request a final stop and terminate the socket worker."""
        self.queue_latest_command(
            self.stop_command()
        )

        time.sleep(0.05)

        self.stop_event.set()

        if self.websocket_thread.is_alive():
            self.websocket_thread.join(
                timeout=3.0
            )

        return super().destroy_node()


def main(args=None):
    """Run the ROS 2 V1 WebSocket controller node."""
    rclpy.init(args=args)

    node = GimbalPDWebSocketNode()

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
