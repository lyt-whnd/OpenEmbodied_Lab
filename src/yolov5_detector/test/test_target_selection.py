"""Tests for target selection independent of ROS and PyTorch."""

from yolov5_detector.target_selection import select_target


DETECTIONS = [
    [0.0, 0.0, 100.0, 100.0, 0.8, 0.0],
    [250.0, 190.0, 350.0, 290.0, 0.9, 0.0],
    [200.0, 100.0, 450.0, 300.0, 0.7, 2.0],
]


def test_largest_filters_by_class():
    selected = select_target(DETECTIONS, 0, 'largest', 640, 480)
    assert selected == DETECTIONS[0]


def test_nearest_center():
    selected = select_target(DETECTIONS, 0, 'nearest_center', 640, 480)
    assert selected == DETECTIONS[1]


def test_all_classes():
    selected = select_target(DETECTIONS, None, 'largest', 640, 480)
    assert selected == DETECTIONS[2]


def test_no_matching_class():
    assert select_target(DETECTIONS, 7, 'largest', 640, 480) is None
