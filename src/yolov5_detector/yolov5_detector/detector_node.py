#!/usr/bin/env python3
"""YOLOv5n detector that feeds the existing gimbal controller."""

import importlib.util
import os
import sys
import threading
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image

from yolov5_detector.target_selection import select_target


def _ensure_yolov5_runtime():
    """Restart the ROS entry point with the isolated YOLOv5 Python."""
    if importlib.util.find_spec('torch') is not None:
        return

    runtime_python = os.path.abspath(
        os.path.expanduser(
            os.environ.get(
                'YOLOV5_PYTHON',
                '~/.venvs/robot-yolov5/bin/python',
            )
        )
    )
    if not os.path.isfile(runtime_python):
        raise RuntimeError(
            'PyTorch is unavailable and YOLOv5 runtime Python was not '
            f'found at {runtime_python}'
        )
    # A venv interpreter is commonly a symlink to /usr/bin/python3, so
    # comparing real paths would incorrectly treat both runtimes as equal.
    if os.path.abspath(sys.executable) == runtime_python:
        raise RuntimeError(
            f'PyTorch is unavailable in YOLOv5 runtime {runtime_python}'
        )

    os.execv(
        runtime_python,
        [runtime_python, sys.argv[0], *sys.argv[1:]],
    )


class Yolov5DetectorNode(Node):
    """Run YOLOv5n on the newest camera frame and publish image error."""

    def __init__(self):
        super().__init__('yolov5_detector_node')

        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('target_topic', '/target_position')
        self.declare_parameter('annotated_topic', '/yolov5/annotated')
        self.declare_parameter(
            'repo_path',
            '~/robot_ws/src/yolov5_detector/vendor/yolov5',
        )
        self.declare_parameter(
            'model_path',
            '~/robot_ws/src/yolov5_detector/weights/yolov5n.pt',
        )
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('target_class', 'person')
        self.declare_parameter('selection_strategy', 'largest')
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('image_size', 320)
        self.declare_parameter('max_detections', 20)
        self.declare_parameter('torch_num_threads', 2)
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('log_period_sec', 2.0)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.target_topic = str(self.get_parameter('target_topic').value)
        self.annotated_topic = str(
            self.get_parameter('annotated_topic').value
        )
        self.repo_path = os.path.abspath(
            os.path.expanduser(str(self.get_parameter('repo_path').value))
        )
        self.model_path = os.path.abspath(
            os.path.expanduser(str(self.get_parameter('model_path').value))
        )
        self.device = str(self.get_parameter('device').value)
        self.target_class_name = str(
            self.get_parameter('target_class').value
        ).strip()
        self.selection_strategy = str(
            self.get_parameter('selection_strategy').value
        )
        self.confidence_threshold = float(
            self.get_parameter('confidence_threshold').value
        )
        self.iou_threshold = float(
            self.get_parameter('iou_threshold').value
        )
        self.image_size = int(self.get_parameter('image_size').value)
        self.max_detections = int(
            self.get_parameter('max_detections').value
        )
        self.torch_num_threads = int(
            self.get_parameter('torch_num_threads').value
        )
        self.publish_annotated = bool(
            self.get_parameter('publish_annotated').value
        )
        self.log_period_sec = float(
            self.get_parameter('log_period_sec').value
        )

        if self.selection_strategy not in ('largest', 'nearest_center'):
            raise ValueError(
                'selection_strategy must be largest or nearest_center'
            )
        if self.image_size <= 0:
            raise ValueError('image_size must be positive')

        self.bridge = CvBridge()
        self.model, self.torch = self._load_model()
        self.target_class_id = self._resolve_target_class()

        image_qos = QoSProfile(depth=1)
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self._image_callback,
            image_qos,
        )
        self.target_publisher = self.create_publisher(
            Point,
            self.target_topic,
            10,
        )
        self.annotated_publisher = None
        if self.publish_annotated:
            self.annotated_publisher = self.create_publisher(
                Image,
                self.annotated_topic,
                image_qos,
            )

        self._frame_lock = threading.Lock()
        self._frame_event = threading.Event()
        self._stop_event = threading.Event()
        self._latest_frame = None
        self._latest_header = None
        self._received_frames = 0
        self._overwritten_frames = 0
        self._processed_frames = 0
        self._inference_ms_total = 0.0
        self._report_started = time.monotonic()

        self._worker = threading.Thread(
            target=self._inference_loop,
            name='yolov5_inference',
            daemon=True,
        )
        self._worker.start()

        selected = (
            'all classes'
            if self.target_class_id is None
            else f'{self.target_class_name} ({self.target_class_id})'
        )
        self.get_logger().info(
            'YOLOv5 detector ready: '
            f'model={self.model_path}, device={self.device}, '
            f'target={selected}, image_size={self.image_size}'
        )

    def _load_model(self):
        if not os.path.isdir(self.repo_path):
            raise FileNotFoundError(
                f'YOLOv5 repository not found: {self.repo_path}'
            )
        if not os.path.isfile(self.model_path):
            raise FileNotFoundError(
                f'YOLOv5 weights not found: {self.model_path}'
            )

        # YOLOv5 v7 checkpoints contain model objects. PyTorch 2.6+
        # otherwise defaults torch.load() to weights_only=True.
        os.environ.setdefault('TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD', '1')

        import torch

        if self.torch_num_threads > 0:
            torch.set_num_threads(self.torch_num_threads)

        model = torch.hub.load(
            self.repo_path,
            'custom',
            path=self.model_path,
            source='local',
            force_reload=False,
            verbose=False,
        )
        model.to(self.device)
        model.eval()
        model.conf = self.confidence_threshold
        model.iou = self.iou_threshold
        model.max_det = self.max_detections
        return model, torch

    def _resolve_target_class(self):
        value = self.target_class_name.lower()
        if value in ('', 'all', '*'):
            return None
        if value.isdigit():
            return int(value)

        names = self.model.names
        if isinstance(names, dict):
            items = names.items()
        else:
            items = enumerate(names)

        for class_id, class_name in items:
            if str(class_name).lower() == value:
                return int(class_id)

        available = ', '.join(str(name) for name in list(names)[:20])
        raise ValueError(
            f'Unknown target_class={self.target_class_name!r}; '
            f'available classes include: {available}'
        )

    def _image_callback(self, message):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding='bgr8',
            )
        except Exception as error:
            self.get_logger().error(f'Image conversion failed: {error}')
            return

        with self._frame_lock:
            if self._latest_frame is not None:
                self._overwritten_frames += 1
            self._latest_frame = frame
            self._latest_header = message.header
            self._received_frames += 1
        self._frame_event.set()

    def _take_latest_frame(self):
        with self._frame_lock:
            frame = self._latest_frame
            header = self._latest_header
            self._latest_frame = None
            self._latest_header = None
            self._frame_event.clear()
        return frame, header

    def _inference_loop(self):
        while not self._stop_event.is_set():
            if not self._frame_event.wait(timeout=0.1):
                continue

            frame, header = self._take_latest_frame()
            if frame is None:
                continue

            started = time.perf_counter()
            try:
                self._process_frame(frame, header)
            except Exception as error:
                self.get_logger().error(f'YOLOv5 inference failed: {error}')
            inference_ms = (time.perf_counter() - started) * 1000.0

            self._processed_frames += 1
            self._inference_ms_total += inference_ms
            self._maybe_report_stats()

    def _process_frame(self, frame, header):
        height, width = frame.shape[:2]
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        with self.torch.inference_mode():
            results = self.model(rgb_frame, size=self.image_size)

        detections = results.xyxy[0].detach().cpu().numpy()
        selected = select_target(
            detections,
            self.target_class_id,
            self.selection_strategy,
            width,
            height,
        )

        target_message = Point()
        if selected is None:
            target_message.x = 0.0
            target_message.y = 0.0
            target_message.z = 0.0
        else:
            x1, y1, x2, y2, confidence, class_id = selected
            center_x = (float(x1) + float(x2)) * 0.5
            center_y = (float(y1) + float(y2)) * 0.5
            target_message.x = center_x - width * 0.5
            target_message.y = center_y - height * 0.5
            target_message.z = max(0.0, float(x2 - x1)) * max(
                0.0,
                float(y2 - y1),
            )
            self._draw_selected(
                frame,
                selected,
                center_x,
                center_y,
                confidence,
                int(class_id),
            )

        self.target_publisher.publish(target_message)

        if self.annotated_publisher is not None:
            cv2.drawMarker(
                frame,
                (width // 2, height // 2),
                (0, 255, 0),
                markerType=cv2.MARKER_CROSS,
                markerSize=30,
                thickness=2,
            )
            annotated = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            annotated.header = header
            self.annotated_publisher.publish(annotated)

    def _draw_selected(
        self,
        frame,
        detection,
        center_x,
        center_y,
        confidence,
        class_id,
    ):
        x1, y1, x2, y2 = [int(value) for value in detection[:4]]
        names = self.model.names
        class_name = names[class_id]
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
        cv2.circle(
            frame,
            (int(center_x), int(center_y)),
            5,
            (0, 0, 255),
            -1,
        )
        cv2.putText(
            frame,
            f'{class_name} {float(confidence):.2f}',
            (x1, max(20, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
        )

    def _maybe_report_stats(self):
        now = time.monotonic()
        elapsed = now - self._report_started
        if elapsed < self.log_period_sec:
            return

        average_ms = (
            self._inference_ms_total / self._processed_frames
            if self._processed_frames
            else 0.0
        )
        fps = self._processed_frames / elapsed if elapsed > 0.0 else 0.0
        self.get_logger().info(
            f'YOLOv5 stats: processed_fps={fps:.2f}, '
            f'avg_inference={average_ms:.1f} ms, '
            f'received={self._received_frames}, '
            f'latest_overwrites={self._overwritten_frames}'
        )
        self._processed_frames = 0
        self._inference_ms_total = 0.0
        self._received_frames = 0
        self._overwritten_frames = 0
        self._report_started = now

    def destroy_node(self):
        self._stop_event.set()
        self._frame_event.set()
        if self._worker.is_alive():
            self._worker.join(timeout=3.0)
        return super().destroy_node()


def main(args=None):
    _ensure_yolov5_runtime()
    rclpy.init(args=args)
    node = None
    try:
        node = Yolov5DetectorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
