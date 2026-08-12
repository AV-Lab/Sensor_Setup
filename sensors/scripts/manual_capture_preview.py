"""Build an advisory side-by-side camera/LiDAR capture preview."""

from pathlib import Path

import cv2
import numpy as np
import yaml

from scripts.calibration_common import CalibrationError
from scripts.lidar_target_detection import detect_lidar_checkerboard
from scripts.lidar_target_detection import load_lidar_detection_policy


PANEL_WIDTH = 640
PANEL_HEIGHT = 480
STATUS_HEIGHT = 64


def load_preview_contract(path):
    """Load target geometry and LiDAR screening policy from calibrate.yaml."""
    path = Path(path).expanduser().resolve()
    try:
        with path.open('r', encoding='utf-8') as config_file:
            document = yaml.safe_load(config_file)
    except (OSError, yaml.YAMLError) as error:
        raise CalibrationError(f'Cannot read preview config {path}: {error}')
    if not isinstance(document, dict):
        raise CalibrationError(f'Preview config {path} must be a mapping.')

    target = document.get('target')
    if not isinstance(target, dict) or target.get('type') != 'checkerboard':
        raise CalibrationError('target.type must be "checkerboard".')
    inner_corners = target.get('inner_corners')
    if (
        not isinstance(inner_corners, list)
        or len(inner_corners) != 2
        or any(
            isinstance(value, bool) or not isinstance(value, int)
            for value in inner_corners
        )
        or any(value < 2 for value in inner_corners)
    ):
        raise CalibrationError(
            'target.inner_corners must be [columns, rows], both at least 2.'
        )
    square_size = target.get('square_size_m')
    if (
        isinstance(square_size, bool)
        or not isinstance(square_size, (int, float))
        or not np.isfinite(square_size)
        or square_size <= 0.0
    ):
        raise CalibrationError('target.square_size_m must be positive.')

    normalized_target = {
        'type': 'checkerboard',
        'inner_corners': list(inner_corners),
        'square_size_m': float(square_size),
    }
    detector_policy = load_lidar_detection_policy(
        document.get('lidar_target_detection')
    )
    if not detector_policy['enabled']:
        raise CalibrationError(
            'Manual preview needs lidar_target_detection.enabled=true.'
        )
    return normalized_target, detector_policy


def detect_camera_checkerboard(gray, pattern_size):
    """Detect checkerboard corners using robust SB and classic fallbacks."""
    sb_flags = (
        cv2.CALIB_CB_NORMALIZE_IMAGE
        | cv2.CALIB_CB_EXHAUSTIVE
        | cv2.CALIB_CB_ACCURACY
    )
    detected, corners = cv2.findChessboardCornersSB(
        gray, pattern_size, flags=sb_flags
    )
    if detected:
        return True, corners.reshape(-1, 2), 'findChessboardCornersSB'

    classic_flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    detected, corners = cv2.findChessboardCorners(
        gray, pattern_size, flags=classic_flags
    )
    if not detected:
        return False, None, 'not_detected'
    refined = cv2.cornerSubPix(
        gray,
        corners,
        (11, 11),
        (-1, -1),
        (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.001),
    )
    return True, refined.reshape(-1, 2), 'findChessboardCorners'


def evaluate_manual_preview(
    image_bgr,
    points,
    value_field,
    target,
    detector_policy,
    image_minus_lidar_ms,
):
    """Evaluate the two target detections for one synchronized pair."""
    image_bgr = np.asarray(image_bgr)
    points = np.asarray(points)
    if image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
        raise ValueError('Preview image must be a three-channel BGR array.')
    if points.ndim != 2 or points.shape[1] != 4:
        raise ValueError('Preview points must have shape (N, 4).')

    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    pattern_size = tuple(target['inner_corners'])
    camera_detected, corners, camera_detector = (
        detect_camera_checkerboard(gray, pattern_size)
    )
    lidar = detect_lidar_checkerboard(
        points[:, :3],
        points[:, 3],
        target,
        detector_policy,
        value_field,
    )
    result = {
        'camera_detected': bool(camera_detected),
        'camera_detector': camera_detector,
        'camera_corner_count': 0 if corners is None else int(len(corners)),
        'lidar': lidar,
        'image_minus_lidar_ms': float(image_minus_lidar_ms),
        'ready': bool(
            camera_detected and lidar['status'] == 'candidate'
        ),
        'advisory_only': True,
    }
    return result, corners


def _letterbox(image, width=PANEL_WIDTH, height=PANEL_HEIGHT):
    """Resize an image without changing its aspect ratio."""
    image = np.asarray(image)
    scale = min(width / image.shape[1], height / image.shape[0])
    resized_width = max(1, round(image.shape[1] * scale))
    resized_height = max(1, round(image.shape[0] * scale))
    resized = cv2.resize(
        image,
        (resized_width, resized_height),
        interpolation=cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR,
    )
    output = np.full((height, width, 3), 24, dtype=np.uint8)
    left = (width - resized_width) // 2
    top = (height - resized_height) // 2
    output[top:top + resized_height, left:left + resized_width] = resized
    return output, scale, left, top


def _camera_panel(image_bgr, corners, result, pattern_size):
    """Draw camera checkerboard corners and status in one fixed panel."""
    panel, scale, left, top = _letterbox(image_bgr)
    if corners is not None:
        display_corners = corners.copy()
        display_corners[:, 0] = display_corners[:, 0] * scale + left
        display_corners[:, 1] = display_corners[:, 1] * scale + top
        cv2.drawChessboardCorners(
            panel,
            pattern_size,
            display_corners.reshape(-1, 1, 2).astype(np.float32),
            True,
        )
    color = (0, 220, 0) if result['camera_detected'] else (0, 0, 240)
    cv2.putText(
        panel,
        'CAMERA: DETECTED' if result['camera_detected'] else 'CAMERA: NO BOARD',
        (16, 34),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        color,
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        panel,
        f'corners={result["camera_corner_count"]}',
        (16, PANEL_HEIGHT - 18),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (230, 230, 230),
        1,
        cv2.LINE_AA,
    )
    return panel


def _scalar_colors(values):
    """Map reflectivity-like values to BGR colors robustly."""
    values = np.asarray(values, dtype=np.float64)
    lower, upper = np.percentile(values, [5.0, 95.0])
    if upper <= lower:
        normalized = np.full(values.shape, 127, dtype=np.uint8)
    else:
        normalized = np.clip(
            (values - lower) * 255.0 / (upper - lower), 0.0, 255.0
        ).astype(np.uint8)
    return cv2.applyColorMap(normalized.reshape(-1, 1), cv2.COLORMAP_TURBO)[
        :, 0, :
    ]


def _lidar_panel(points, result, detector_policy):
    """Draw the best LiDAR plane in its local coordinates."""
    panel = np.full((PANEL_HEIGHT, PANEL_WIDTH, 3), 24, dtype=np.uint8)
    detection = result['lidar']
    candidate = detection['candidate']
    detected = detection['status'] == 'candidate'
    color = (0, 220, 0) if detected else (0, 0, 240)
    title = 'LIDAR: CANDIDATE' if detected else 'LIDAR: NO CANDIDATE'
    cv2.putText(
        panel,
        title,
        (16, 34),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        color,
        2,
        cv2.LINE_AA,
    )
    if candidate is None:
        cv2.putText(
            panel,
            detection['reason'][:64],
            (16, 68),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (210, 210, 210),
            1,
            cv2.LINE_AA,
        )
        return panel

    center = np.asarray(candidate['center_sensor_m'], dtype=np.float64)
    axes = np.asarray(candidate['plane_axes_sensor'], dtype=np.float64)
    normal = np.asarray(candidate['normal_sensor'], dtype=np.float64)
    expected = np.asarray(candidate['expected_extents_m'], dtype=np.float64)
    xyz = points[:, :3].astype(np.float64, copy=False)
    offsets = xyz - center
    projected = offsets @ axes.T
    plane_distance = np.abs(offsets @ normal)
    half_window = expected * 0.75
    visible = plane_distance <= detector_policy['plane_distance_threshold_m']
    visible &= np.logical_and(
        projected >= -half_window, projected <= half_window
    ).all(axis=1)
    projected = projected[visible]
    values = points[visible, 3]

    plot_left, plot_top = 35, 65
    plot_width, plot_height = PANEL_WIDTH - 70, PANEL_HEIGHT - 145
    cv2.rectangle(
        panel,
        (plot_left, plot_top),
        (plot_left + plot_width, plot_top + plot_height),
        (90, 90, 90),
        1,
    )
    if len(projected):
        normalized = (projected + half_window) / (2.0 * half_window)
        pixels = np.column_stack((
            plot_left + normalized[:, 0] * plot_width,
            plot_top + (1.0 - normalized[:, 1]) * plot_height,
        ))
        colors = _scalar_colors(values)
        for pixel, point_color in zip(pixels.astype(int), colors):
            cv2.circle(panel, tuple(pixel), 2, tuple(map(int, point_color)), -1)

    board_lower = 1.0 / 6.0
    board_upper = 5.0 / 6.0
    cv2.rectangle(
        panel,
        (
            round(plot_left + plot_width * board_lower),
            round(plot_top + plot_height * board_lower),
        ),
        (
            round(plot_left + plot_width * board_upper),
            round(plot_top + plot_height * board_upper),
        ),
        color,
        2,
    )
    extents = candidate['estimated_extents_m']
    metrics = (
        f'points={candidate["plane_point_count"]}  '
        f'extent={extents[0]:.2f}x{extents[1]:.2f} m  '
        f'rmse={candidate["plane_rmse_m"] * 1000.0:.1f} mm'
    )
    cv2.putText(
        panel,
        metrics,
        (16, PANEL_HEIGHT - 46),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.43,
        (230, 230, 230),
        1,
        cv2.LINE_AA,
    )
    checker_metrics = (
        f'contrast={candidate["checker_contrast"]:.2f}  '
        f'pattern={candidate["checker_classification_accuracy"]:.2f}'
    )
    cv2.putText(
        panel,
        checker_metrics,
        (16, PANEL_HEIGHT - 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        color,
        1,
        cv2.LINE_AA,
    )
    return panel


def render_manual_preview(
    image_bgr,
    points,
    result,
    corners,
    target,
    detector_policy,
):
    """Render camera and LiDAR detection results into one BGR image."""
    camera = _camera_panel(
        image_bgr, corners, result, tuple(target['inner_corners'])
    )
    lidar = _lidar_panel(points, result, detector_policy)
    body = np.hstack((camera, lidar))
    output = np.full(
        (PANEL_HEIGHT + STATUS_HEIGHT, PANEL_WIDTH * 2, 3),
        18,
        dtype=np.uint8,
    )
    output[STATUS_HEIGHT:] = body
    ready = result['ready']
    color = (0, 180, 0) if ready else (0, 125, 255)
    status = 'READY: BOTH TARGETS FOUND' if ready else 'HOLD: BOTH TARGETS REQUIRED'
    cv2.putText(
        output,
        status,
        (18, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        color,
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        output,
        f'advisory preview  |  image-lidar={result["image_minus_lidar_ms"]:.2f} ms',
        (780, 39),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (220, 220, 220),
        1,
        cv2.LINE_AA,
    )
    return output
