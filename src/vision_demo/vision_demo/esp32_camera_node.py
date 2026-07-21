#!/usr/bin/env python3

import threading
import time

import cv2
import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image


class Esp32CameraNode(Node):
    """
    从 ESP32-CAM 的 HTTP MJPEG 地址读取视频，
    转换为 ROS 2 sensor_msgs/Image，并发布到 /image_raw。
    """

    def __init__(self):
        super().__init__('esp32_camera_node')

        self.declare_parameter(
            'stream_url',
            'http://192.168.1.100:81/stream',
        )
        self.declare_parameter(
            'image_topic',
            '/image_raw',
        )
        self.declare_parameter(
            'frame_id',
            'esp32_camera',
        )
        self.declare_parameter(
            'reconnect_delay_sec',
            2.0,
        )
        self.declare_parameter(
            'opencv_buffer_size',
            1,
        )

        self.stream_url = str(
            self.get_parameter('stream_url').value
        )
        self.image_topic = str(
            self.get_parameter('image_topic').value
        )
        self.frame_id = str(
            self.get_parameter('frame_id').value
        )
        self.reconnect_delay_sec = float(
            self.get_parameter(
                'reconnect_delay_sec'
            ).value
        )
        self.opencv_buffer_size = int(
            self.get_parameter(
                'opencv_buffer_size'
            ).value
        )

        
         #队列深度设为 1：
         #下游节点处理不过来时，不积压大量旧图像。
        
        image_qos = QoSProfile(depth=1)

        self.image_publisher = self.create_publisher(
            Image,
            self.image_topic,
            image_qos,
        )

        self.bridge = CvBridge()
        self.stop_event = threading.Event()

        self.capture_thread = threading.Thread(
            target=self.capture_loop,
            name='esp32_mjpeg_capture',
            daemon=True,
        )

        self.capture_thread.start()

        self.get_logger().info(
            f'ESP32 camera node started: '
            f'{self.stream_url} -> {self.image_topic}'
        )

    def capture_loop(self):
        """
        独立线程持续读取 MJPEG。

        不在 ROS 回调线程里执行 cap.read()，
        防止网络阻塞影响其他 ROS 回调。
        """

        while not self.stop_event.is_set():
            capture = cv2.VideoCapture(
                self.stream_url
            )

            if self.opencv_buffer_size > 0:
                capture.set(
                    cv2.CAP_PROP_BUFFERSIZE,
                    self.opencv_buffer_size,
                )

            if not capture.isOpened():
                self.get_logger().warning(
                    f'Cannot open ESP32 stream: '
                    f'{self.stream_url}'
                )

                capture.release()

                self.stop_event.wait(
                    self.reconnect_delay_sec
                )
                continue

            self.get_logger().info(
                'ESP32 MJPEG stream connected'
            )

            frame_count = 0
            report_start = time.monotonic()

            while not self.stop_event.is_set():
                ok, frame = capture.read()

                if not ok or frame is None:
                    self.get_logger().warning(
                        'ESP32 stream read failed, '
                        'reconnecting'
                    )
                    break

                image_message = (
                    self.bridge.cv2_to_imgmsg(
                        frame,
                        encoding='bgr8',
                    )
                )

                image_message.header.stamp = (
                    self.get_clock().now().to_msg()
                )
                image_message.header.frame_id = (
                    self.frame_id
                )

                self.image_publisher.publish(
                    image_message
                )

                frame_count += 1

                now = time.monotonic()
                elapsed = now - report_start

                if elapsed >= 2.0:
                    height, width = frame.shape[:2]
                    fps = frame_count / elapsed

                    self.get_logger().info(
                        f'ESP32 stream: '
                        f'{width}x{height}, '
                        f'{fps:.2f} FPS'
                    )

                    frame_count = 0
                    report_start = now

            capture.release()

            if not self.stop_event.is_set():
                self.stop_event.wait(
                    self.reconnect_delay_sec
                )

    def destroy_node(self):
        self.stop_event.set()

        if self.capture_thread.is_alive():
            self.capture_thread.join(
                timeout=3.0
            )

        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = Esp32CameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()