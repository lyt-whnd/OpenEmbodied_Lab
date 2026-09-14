"""Detection filtering and target selection helpers."""


def select_target(detections, class_id, strategy, width, height):
    """Return one detection or None.

    Each detection is ``[x1, y1, x2, y2, confidence, class_id]``.
    ``class_id`` may be None to accept all model classes.
    """
    candidates = []

    for detection in detections:
        if len(detection) < 6:
            continue

        detected_class = int(detection[5])
        if class_id is not None and detected_class != class_id:
            continue

        x1, y1, x2, y2 = map(float, detection[:4])
        area = max(0.0, x2 - x1) * max(0.0, y2 - y1)
        center_x = (x1 + x2) * 0.5
        center_y = (y1 + y2) * 0.5
        center_distance_sq = (
            (center_x - width * 0.5) ** 2
            + (center_y - height * 0.5) ** 2
        )
        candidates.append((detection, area, center_distance_sq))

    if not candidates:
        return None

    if strategy == 'nearest_center':
        return min(candidates, key=lambda item: item[2])[0]

    return max(candidates, key=lambda item: item[1])[0]
