#!/usr/bin/env python3

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point


class ColorTrackerNode(Node):
    def __init__(self):
        super().__init__('color_tracker_node')

        self.declare_parameter('image_topic', '/image_raw')

        self.image_topic = self.get_parameter(
            'image_topic'
        ).get_parameter_value().string_value

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )
        self.target_pub = self.create_publisher(
            Point,
            '/target_position',
            10
        )

        self.frame_count = 0
        self.last_time = time.time()
        self.fps = 0.0

        # 绿色 HSV 阈值，后面可以按实际物体调整
        self.lower_green = np.array([35, 80, 80])
        self.upper_green = np.array([85, 255, 255])

        # 面积太小的轮廓认为是噪声
        self.min_area = 500

        self.get_logger().info(
            f'Color tracker started, subscribing: {self.image_topic}'
        )

    def image_callback(self, msg: Image):
        try:
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

        image_center_x = width // 2
        image_center_y = height // 2

        self.update_fps()

        # BGR 转 HSV，颜色检测通常在 HSV 空间更稳定
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 根据绿色阈值生成二值 mask
        mask = cv2.inRange(
            hsv,
            self.lower_green,
            self.upper_green
        )

        # 形态学操作：去掉小噪点，让目标区域更完整
        kernel = np.ones((5, 5), np.uint8)

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            kernel
        )

        # 查找轮廓
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        target_found = False
        target_x = 0
        target_y = 0
        error_x = 0
        error_y = 0
        target_area = 0

        if contours:
            # 找面积最大的轮廓，认为它是目标
            largest_contour = max(
                contours,
                key=cv2.contourArea
            )

            target_area = cv2.contourArea(largest_contour)

            if target_area >= self.min_area:
                target_found = True

                x, y, w, h = cv2.boundingRect(largest_contour)

                target_x = x + w // 2
                target_y = y + h // 2

                error_x = target_x - image_center_x
                error_y = target_y - image_center_y

                # 画目标框
                cv2.rectangle(
                    frame,
                    (x, y),
                    (x + w, y + h),
                    (0, 255, 255),
                    2
                )

                # 画目标中心点
                cv2.circle(
                    frame,
                    (target_x, target_y),
                    6,
                    (0, 0, 255),
                    -1
                )

                # 从画面中心连线到目标中心
                cv2.line(
                    frame,
                    (image_center_x, image_center_y),
                    (target_x, target_y),
                    (255, 0, 0),
                    2
                )

                cv2.putText(
                    frame,
                    f'target: ({target_x}, {target_y})',
                    (x, max(y - 10, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2
                )

        # 画画面中心十字线
        self.draw_center_cross(
            frame,
            image_center_x,
            image_center_y
        )

        # 显示状态文字
        status = 'FOUND' if target_found else 'LOST'

        cv2.putText(
            frame,
            f'{width}x{height} FPS:{self.fps:.2f}',
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )

        cv2.putText(
            frame,
            f'status:{status} area:{int(target_area)}',
            (20, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0) if target_found else (0, 0, 255),
            2
        )

        cv2.putText(
            frame,
            f'error_x:{error_x} error_y:{error_y}',
            (20, 105),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 0),
            2
        )

        target_msg = Point()

        if target_found:
            target_msg.x = float(error_x)
            target_msg.y = float(error_y)
            target_msg.z = float(target_area)
        else:
            target_msg.x = 0.0
            target_msg.y = 0.0
            target_msg.z = 0.0

        self.target_pub.publish(target_msg)

        # 每秒打印一次
        if self.frame_count == 0:
            self.get_logger().info(
                f'size={width}x{height}, '
                f'fps={self.fps:.2f}, '
                f'found={target_found}, '
                f'target=({target_x},{target_y}), '
                f'error=({error_x},{error_y}), '
                f'area={int(target_area)}'
            )

        cv2.imshow('Color Tracker View', frame)
        cv2.imshow('Color Mask', mask)

        key = cv2.waitKey(1)

        if key == ord('q'):
            cv2.destroyAllWindows()

    def update_fps(self):
        self.frame_count += 1
        now = time.time()
        elapsed = now - self.last_time

        if elapsed >= 1.0:
            self.fps = self.frame_count / elapsed
            self.frame_count = 0
            self.last_time = now

    def draw_center_cross(self, frame, center_x, center_y):
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

        cv2.circle(
            frame,
            (center_x, center_y),
            5,
            (0, 0, 255),
            -1
        )


def main(args=None):
    rclpy.init(args=args)

    node = ColorTrackerNode()

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