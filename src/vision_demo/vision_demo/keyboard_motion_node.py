#!/usr/bin/env python3
"""Publish keyboard motion commands to the shared RobotLink."""

import os
import random
import select
import sys
import termios
import time
import tty
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt8MultiArray

from vision_demo.protocol_v1 import (
    ApplicationMessage,
    MessageFlag,
    MotionMovePayload,
    MotionOpcode,
    NodeId,
    SequenceGenerator,
    ServiceId,
)


DIRECTION_UP = 'up'
DIRECTION_DOWN = 'down'
DIRECTION_LEFT = 'left'
DIRECTION_RIGHT = 'right'
DIRECTIONS = (
    DIRECTION_UP,
    DIRECTION_DOWN,
    DIRECTION_LEFT,
    DIRECTION_RIGHT,
)


class KeySequenceDecoder:
    """Decode terminal arrow escape sequences and single-character keys."""

    _ARROW_KEYS = {
        '\x1b[A': DIRECTION_UP,
        '\x1b[B': DIRECTION_DOWN,
        '\x1b[C': DIRECTION_RIGHT,
        '\x1b[D': DIRECTION_LEFT,
    }

    def __init__(self):
        """Create an empty incremental decoder."""
        self._buffer = ''

    def feed(self, text: str) -> List[str]:
        """Consume terminal text and return complete logical keys."""
        self._buffer += text
        keys = []

        while self._buffer:
            if self._buffer[0] != '\x1b':
                keys.append(self._buffer[0].lower())
                self._buffer = self._buffer[1:]
                continue

            if len(self._buffer) < 3:
                break

            sequence = self._buffer[:3]
            direction = self._ARROW_KEYS.get(sequence)

            if direction is not None:
                keys.append(direction)
                self._buffer = self._buffer[3:]
            else:
                self._buffer = self._buffer[1:]

        return keys


def direction_to_head_rates(
    direction: str,
    rate_x10: int,
) -> Tuple[int, int]:
    """Map one arrow direction to yaw and pitch rates."""
    if direction == DIRECTION_UP:
        return 0, -rate_x10

    if direction == DIRECTION_DOWN:
        return 0, rate_x10

    if direction == DIRECTION_LEFT:
        return rate_x10, 0

    if direction == DIRECTION_RIGHT:
        return -rate_x10, 0

    raise ValueError(f'unknown direction: {direction}')


def encode_motion_command(
    opcode: MotionOpcode,
    sequence: int,
    payload: bytes = b'',
) -> bytes:
    """Encode a Linux-to-STM32 command for routing through ESP32."""
    flags = MessageFlag.ACK_REQUIRED

    if opcode == MotionOpcode.MOVE:
        flags = MessageFlag.REALTIME

    return ApplicationMessage(
        flags=int(flags),
        src=int(NodeId.LINUX),
        dst=int(NodeId.STM32),
        service=int(ServiceId.MOTION),
        opcode=int(opcode),
        seq=sequence,
        payload=payload,
    ).encode()


class KeyboardMotionNode(Node):
    """Publish keyboard and random head motion through RobotLink."""

    def __init__(self):
        """Initialize parameters, terminal input, and the control timer."""
        super().__init__('keyboard_motion_node')

        self.declare_parameter('command_topic', '/robot_link/tx')
        self.declare_parameter('command_hz', 10.0)
        self.declare_parameter('head_rate_deg_s', 30.0)
        self.declare_parameter('move_valid_ms', 300)
        self.declare_parameter('manual_hold_sec', 0.35)
        self.declare_parameter('auto_change_sec', 0.7)
        self.declare_parameter('control_epoch', 0)

        self.command_topic = str(
            self.get_parameter('command_topic').value
        )
        self.command_hz = float(
            self.get_parameter('command_hz').value
        )
        head_rate_deg_s = float(
            self.get_parameter('head_rate_deg_s').value
        )
        self.move_valid_ms = int(
            self.get_parameter('move_valid_ms').value
        )
        self.manual_hold_sec = float(
            self.get_parameter('manual_hold_sec').value
        )
        self.auto_change_sec = float(
            self.get_parameter('auto_change_sec').value
        )
        configured_epoch = int(
            self.get_parameter('control_epoch').value
        )

        if self.command_hz <= 0.0:
            raise ValueError('command_hz must be greater than zero')

        if not 0.1 <= head_rate_deg_s <= 90.0:
            raise ValueError(
                'head_rate_deg_s must be in [0.1, 90.0]'
            )

        if not 1 <= self.move_valid_ms <= 1000:
            raise ValueError(
                'move_valid_ms must be in [1, 1000]'
            )

        if self.manual_hold_sec <= 0.0:
            raise ValueError(
                'manual_hold_sec must be greater than zero'
            )

        if self.auto_change_sec <= 0.0:
            raise ValueError(
                'auto_change_sec must be greater than zero'
            )

        if not 0 <= configured_epoch <= 0xFFFF:
            raise ValueError(
                'control_epoch must be in [0, 65535]'
            )

        self.head_rate_x10 = int(
            round(head_rate_deg_s * 10.0)
        )
        self.control_epoch = configured_epoch or (
            (time.time_ns() & 0xFFFF) or 1
        )

        self.sequence = SequenceGenerator()

        self.key_decoder = KeySequenceDecoder()
        self.stdin_fd = sys.stdin.fileno()

        if not os.isatty(self.stdin_fd):
            raise RuntimeError(
                'keyboard_motion must run in an interactive terminal'
            )

        self.saved_terminal_settings = termios.tcgetattr(
            self.stdin_fd
        )
        self.terminal_restored = False
        tty.setcbreak(self.stdin_fd)

        self.manual_direction: Optional[str] = None
        self.manual_until = 0.0
        self.auto_enabled = False
        self.auto_direction: Optional[str] = None
        self.next_auto_change_at = 0.0
        self.last_direction: Optional[str] = None
        self.last_send_at = 0.0
        self.quit_requested = False

        self.timer = self.create_timer(
            1.0 / max(self.command_hz, 20.0),
            self.control_tick,
        )
        self.command_publisher = self.create_publisher(
            UInt8MultiArray,
            self.command_topic,
            10,
        )

        self.get_logger().info(
            'Keyboard control ready: arrows=move, '
            'b=toggle random auto movement, q=stop/center/quit'
        )
        self.get_logger().info(
            f'control_epoch={self.control_epoch}, '
            f'head_rate={head_rate_deg_s:.1f} deg/s, '
            f'topic={self.command_topic}'
        )

    def _send_opcode(
        self,
        opcode: MotionOpcode,
        payload: bytes = b'',
    ) -> bool:
        sequence = self.sequence.next_seq()
        packet = encode_motion_command(
            opcode,
            sequence,
            payload,
        )

        message = UInt8MultiArray()
        message.data = list(packet)
        self.command_publisher.publish(message)
        return True

    def _send_direction(self, direction: str) -> bool:
        yaw_rate, pitch_rate = direction_to_head_rates(
            direction,
            self.head_rate_x10,
        )
        payload = MotionMovePayload(
            control_epoch=self.control_epoch,
            valid_ms=self.move_valid_ms,
            linear_mm_s=0,
            angular_mrad_s=0,
            head_yaw_rate_x10=yaw_rate,
            head_pitch_rate_x10=pitch_rate,
        ).encode()

        return self._send_opcode(
            MotionOpcode.MOVE,
            payload,
        )

    def _read_keys(self) -> List[str]:
        keys = []

        while select.select(
            [self.stdin_fd],
            [],
            [],
            0.0,
        )[0]:
            data = os.read(self.stdin_fd, 64)

            if not data:
                break

            keys.extend(
                self.key_decoder.feed(
                    data.decode('latin-1')
                )
            )

        return keys

    def _handle_key(self, key: str, now: float) -> None:
        if key in DIRECTIONS:
            self.auto_enabled = False
            self.auto_direction = None
            self.manual_direction = key
            self.manual_until = now + self.manual_hold_sec
            self.last_send_at = 0.0
            return

        if key == 'b':
            self.auto_enabled = not self.auto_enabled
            self.manual_direction = None

            if self.auto_enabled:
                self.auto_direction = random.choice(DIRECTIONS)
                self.next_auto_change_at = (
                    now + self.auto_change_sec
                )
                self.last_send_at = 0.0
                self.get_logger().info(
                    'Random automatic movement enabled'
                )
            else:
                self.auto_direction = None
                self._send_opcode(MotionOpcode.STOP)
                self.last_direction = None
                self.get_logger().info(
                    'Random automatic movement disabled'
                )
            return

        if key == 'q':
            self.quit_requested = True
            self.stop_center_and_close()
            self.restore_terminal()

            if rclpy.ok():
                rclpy.shutdown()

    def _current_direction(self, now: float) -> Optional[str]:
        if self.auto_enabled:
            if now >= self.next_auto_change_at:
                choices = [
                    direction
                    for direction in DIRECTIONS
                    if direction != self.auto_direction
                ]
                self.auto_direction = random.choice(choices)
                self.next_auto_change_at = (
                    now + self.auto_change_sec
                )

            return self.auto_direction

        if (
            self.manual_direction is not None
            and now < self.manual_until
        ):
            return self.manual_direction

        self.manual_direction = None
        return None

    def control_tick(self) -> None:
        """Poll keyboard input and refresh the current safe motion command."""
        now = time.monotonic()

        for key in self._read_keys():
            self._handle_key(key, now)

            if self.quit_requested:
                return

        direction = self._current_direction(now)

        if direction is None:
            if self.last_direction is not None:
                self._send_opcode(MotionOpcode.STOP)
                self.last_direction = None
            return

        send_period = 1.0 / self.command_hz

        if (
            direction != self.last_direction
            or now - self.last_send_at >= send_period
        ):
            if self._send_direction(direction):
                if direction != self.last_direction:
                    self.get_logger().info(
                        f'Head direction: {direction}'
                    )

                self.last_direction = direction
                self.last_send_at = now

    def stop_center_and_close(self) -> None:
        """Publish normal STOP then CENTER and clear local motion state."""
        self._send_opcode(MotionOpcode.STOP)
        self._send_opcode(MotionOpcode.CENTER)

        self.last_direction = None
        self.auto_enabled = False
        self.manual_direction = None

    def restore_terminal(self) -> None:
        """Restore terminal settings exactly once."""
        if self.terminal_restored:
            return

        termios.tcsetattr(
            self.stdin_fd,
            termios.TCSADRAIN,
            self.saved_terminal_settings,
        )
        self.terminal_restored = True

    def destroy_node(self):
        """Stop, center, restore the terminal, and release resources."""
        if not self.quit_requested:
            self.stop_center_and_close()

        self.restore_terminal()
        return super().destroy_node()


def main(args=None):
    """Run the interactive RobotLink keyboard motion producer."""
    rclpy.init(args=args)
    node = None

    try:
        node = KeyboardMotionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
