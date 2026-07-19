#!/usr/bin/env python3

import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImageViewerNode(Node):
    def __init__(self):
        super().__init__('image_viewer_node')

        # 允许通过参数修改订阅的话题名
        self.declare_parameter('image_topic', '/image_raw')
        self.image_topic = self.get_parameter(
            'image_topic'
        ).get_parameter_value().string_value

        self.bridge = CvBridge()

        self.frame_count = 0
        self.last_time = time.time()
        self.fps = 0.0

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.get_logger().info(
            f'Image viewer node started, subscribing: {self.image_topic}'
        )

    def image_callback(self, msg: Image):
        try:
            # ROS Image 消息转 OpenCV Mat
            # bgr8 是 OpenCV 常用格式
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception as e:
            self.get_logger().error(
                f'Failed to convert image: {e}'
            )
            return

        height, width = frame.shape[:2]

        # 计算画面中心
        center_x = width // 2
        center_y = height // 2

        # 计算 FPS
        self.frame_count += 1
        now = time.time()
        elapsed = now - self.last_time

        if elapsed >= 1.0:
            self.fps = self.frame_count / elapsed
            self.frame_count = 0
            self.last_time = now

            self.get_logger().info(
                f'Image size: {width}x{height}, FPS: {self.fps:.2f}'
            )

        # 画中心十字线
        line_length = 40

        cv2.line(
            frame,
            (center_x - line_length, center_y),
            (center_x + line_length, center_y),
            (0, 255, 0),
            2
        )

        cv2.line(
            frame,
            (center_x, center_y - line_length),
            (center_x, center_y + line_length),
            (0, 255, 0),
            2
        )

        # 画中心点
        cv2.circle(
            frame,
            (center_x, center_y),
            5,
            (0, 0, 255),
            -1
        )

        # 在图像上显示信息
        text = f'{width}x{height}  FPS: {self.fps:.2f}'

        cv2.putText(
            frame,
            text,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2
        )

        # 显示图像
        cv2.imshow('ROS2 USB Camera Viewer', frame)

        # 必须调用 waitKey，否则窗口可能不刷新
        key = cv2.waitKey(1)

        # 按 q 退出显示窗口
        if key == ord('q'):
            self.get_logger().info('q pressed, closing window.')
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)

    node = ImageViewerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()