"""Audit whether captured image/cloud pairs are useful for calibration."""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys
import uuid

import cv2
import numpy as np

from scripts.calibration_common import CalibrationError
from scripts.calibration_common import default_config_path
from scripts.calibration_common import load_calibration_bundle
from scripts.calibration_common import load_session_pairs
from scripts.calibration_common import read_pcd


def _positive_number(mapping, key, default):
    """Read one positive finite audit setting."""
    value = mapping.get(key, default)
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not np.isfinite(value)
        or value <= 0.0
    ):
        raise CalibrationError(f'quality.{key} must be a positive number.')
    return float(value)


def load_audit_policy(bundle):
    """Validate target geometry and quality-screening thresholds."""
    target = bundle.raw.get('target')
    if not isinstance(target, dict) or target.get('type') != 'checkerboard':
        raise CalibrationError('target.type must be "checkerboard".')
    inner = target.get('inner_corners')
    if (
        not isinstance(inner, list)
        or len(inner) != 2
        or any(
            isinstance(item, bool) or not isinstance(item, int)
            for item in inner
        )
        or any(item < 2 for item in inner)
    ):
        raise CalibrationError(
            'target.inner_corners must be [columns, rows], both at least 2.'
        )
    square_size = _positive_number(
        target, 'square_size_m', target.get('square_size_m')
    )
    quality = bundle.raw.get('quality', {})
    if not isinstance(quality, dict):
        raise CalibrationError('quality must be a mapping.')
    return {
        'pattern_size': tuple(inner),
        'square_size_m': square_size,
        'max_abs_sync_delta_ms': _positive_number(
            quality, 'max_abs_sync_delta_ms', 25.0
        ),
        'min_cloud_points': int(
            _positive_number(quality, 'min_cloud_points', 1000)
        ),
        'min_sharpness_variance': _positive_number(
            quality, 'min_sharpness_variance', 80.0
        ),
        'max_dark_fraction': _positive_number(
            quality, 'max_dark_fraction', 0.25
        ),
        'max_bright_fraction': _positive_number(
            quality, 'max_bright_fraction', 0.25
        ),
        'min_board_image_fraction': _positive_number(
            quality, 'min_board_image_fraction', 0.01
        ),
        'duplicate_translation_m': _positive_number(
            quality, 'duplicate_translation_m', 0.10
        ),
        'duplicate_normal_angle_deg': _positive_number(
            quality, 'duplicate_normal_angle_deg', 5.0
        ),
    }


def checkerboard_object_points(pattern_size, square_size_m):
    """Create OpenCV-ordered planar checkerboard inner-corner coordinates."""
    columns, rows = pattern_size
    grid = np.zeros((columns * rows, 3), dtype=np.float32)
    grid[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)
    grid[:, :2] *= square_size_m
    return grid


def _detect_checkerboard(gray, pattern_size):
    """Use the robust SB detector, with a classic OpenCV fallback."""
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


def _checkerboard_crop(gray, corners):
    """Return a bounded image crop around a detected checkerboard."""
    if corners is None:
        return gray
    minimum = np.floor(corners.min(axis=0)).astype(int)
    maximum = np.ceil(corners.max(axis=0)).astype(int)
    padding = max(10, int(0.05 * max(maximum - minimum)))
    left = max(0, minimum[0] - padding)
    top = max(0, minimum[1] - padding)
    right = min(gray.shape[1], maximum[0] + padding + 1)
    bottom = min(gray.shape[0], maximum[1] + padding + 1)
    return gray[top:bottom, left:right]


def analyze_image(image, camera, policy):
    """Detect the board and measure image-side calibration usefulness."""
    if image.shape[:2] != (camera.height, camera.width):
        raise CalibrationError(
            f'Image is {image.shape[1]}x{image.shape[0]}, expected '
            f'{camera.width}x{camera.height} from camera intrinsics.'
        )
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    detected, corners, detector = _detect_checkerboard(
        gray, policy['pattern_size']
    )
    crop = _checkerboard_crop(gray, corners)
    sharpness = float(cv2.Laplacian(crop, cv2.CV_64F).var())
    dark_fraction = float(np.mean(crop <= 5))
    bright_fraction = float(np.mean(crop >= 250))
    result = {
        'checkerboard_detected': bool(detected),
        'checkerboard_detector': detector,
        'sharpness_variance': sharpness,
        'dark_fraction': dark_fraction,
        'bright_fraction': bright_fraction,
        'board_image_fraction': None,
        'board_center_normalized': None,
        'board_distance_m': None,
        'board_translation_camera_m': None,
        'board_normal_camera': None,
        'board_reprojection_rmse_px': None,
    }
    if not detected:
        return result, corners

    hull = cv2.convexHull(corners.astype(np.float32))
    result['board_image_fraction'] = float(
        cv2.contourArea(hull) / (camera.width * camera.height)
    )
    center = corners.mean(axis=0)
    result['board_center_normalized'] = [
        float(center[0] / camera.width),
        float(center[1] / camera.height),
    ]

    object_points = checkerboard_object_points(
        policy['pattern_size'], policy['square_size_m']
    )
    solved, rotation_vector, translation = cv2.solvePnP(
        object_points,
        corners.astype(np.float32),
        camera.matrix,
        camera.distortion,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if solved:
        rotation, _ = cv2.Rodrigues(rotation_vector)
        normal = rotation[:, 2]
        projected, _ = cv2.projectPoints(
            object_points,
            rotation_vector,
            translation,
            camera.matrix,
            camera.distortion,
        )
        residual = projected.reshape(-1, 2) - corners
        result['board_translation_camera_m'] = (
            translation.reshape(3).astype(float).tolist()
        )
        result['board_distance_m'] = float(np.linalg.norm(translation))
        result['board_normal_camera'] = normal.astype(float).tolist()
        result['board_reprojection_rmse_px'] = float(
            np.sqrt(np.mean(np.square(residual)))
        )
    return result, corners


def analyze_cloud(path):
    """Verify PCD structure and summarize its actual valid XYZ content."""
    header, cloud = read_pcd(path)
    xyz = np.column_stack((cloud['x'], cloud['y'], cloud['z']))
    finite = np.isfinite(xyz).all(axis=1)
    nonzero = np.square(xyz).sum(axis=1) > 0.0
    valid = finite & nonzero
    return {
        'pcd_data_kind': header['DATA'][0].lower(),
        'pcd_fields': list(header.get('FIELDS') or header.get('FIELD')),
        'actual_point_count': int(len(cloud)),
        'valid_xyz_count': int(np.count_nonzero(valid)),
        'valid_xyz_fraction': float(np.mean(valid)) if len(cloud) else 0.0,
        # General cloud checks cannot prove that the calibration target is hit.
        'lidar_target_detection': 'not_verified',
    }


def _pair_reasons(pair, image_result, cloud_result, bundle, policy):
    """Return hard failures and softer screening warnings for one pair."""
    failures = []
    warnings = []
    if not image_result['checkerboard_detected']:
        failures.append('checkerboard_not_detected')
    if pair.get('lidar_frame_id') != bundle.extrinsic.source_frame:
        failures.append('unexpected_lidar_frame')
    if pair.get('image_frame_id') != bundle.extrinsic.target_frame:
        failures.append('unexpected_image_frame')
    delta_ms = abs(float(pair.get('image_minus_lidar_ns', 0))) / 1e6
    if delta_ms > policy['max_abs_sync_delta_ms']:
        failures.append('sync_delta_too_large')
    if cloud_result['valid_xyz_count'] < policy['min_cloud_points']:
        failures.append('too_few_valid_cloud_points')

    if image_result['sharpness_variance'] < policy['min_sharpness_variance']:
        warnings.append('image_or_board_region_may_be_blurred')
    if image_result['dark_fraction'] > policy['max_dark_fraction']:
        warnings.append('board_region_has_many_dark_clipped_pixels')
    if image_result['bright_fraction'] > policy['max_bright_fraction']:
        warnings.append('board_region_has_many_bright_clipped_pixels')
    fraction = image_result['board_image_fraction']
    if fraction is not None and fraction < policy['min_board_image_fraction']:
        warnings.append('checkerboard_is_small_in_image')
    if cloud_result['lidar_target_detection'] != 'verified':
        warnings.append('lidar_board_presence_not_automatically_verified')
    return failures, warnings


def _normal_angle_degrees(first, second):
    """Measure an orientation-insensitive plane-normal angle."""
    first = np.asarray(first, dtype=np.float64)
    second = np.asarray(second, dtype=np.float64)
    cosine = np.clip(abs(float(first @ second)), 0.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


def mark_duplicate_poses(results, policy):
    """Flag later board poses that add little camera-side geometry."""
    accepted = []
    duplicate_count = 0
    for result in results:
        image = result['image']
        translation = image['board_translation_camera_m']
        normal = image['board_normal_camera']
        if translation is None or normal is None:
            continue
        duplicate_of = None
        for previous in accepted:
            previous_image = previous['image']
            distance = np.linalg.norm(
                np.asarray(translation)
                - np.asarray(previous_image['board_translation_camera_m'])
            )
            angle = _normal_angle_degrees(
                normal, previous_image['board_normal_camera']
            )
            if (
                distance < policy['duplicate_translation_m']
                and angle < policy['duplicate_normal_angle_deg']
            ):
                duplicate_of = previous['index']
                break
        result['possible_duplicate_of'] = duplicate_of
        if duplicate_of is None:
            accepted.append(result)
        else:
            duplicate_count += 1
            result['warnings'].append('camera_board_pose_is_near_duplicate')
    return duplicate_count


def _draw_audit_overlay(image, corners, result):
    """Draw detected corners and concise audit status."""
    output = image.copy()
    if corners is not None:
        for corner in np.rint(corners).astype(int):
            cv2.circle(output, tuple(corner), 3, (0, 255, 0), -1)
    usable = not result['failures']
    status = 'CANDIDATE' if usable else 'REJECT'
    color = (0, 200, 0) if usable else (0, 0, 255)
    text = f'pair {result["index"]}: {status}'
    cv2.putText(
        output,
        text,
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        color,
        3,
        cv2.LINE_AA,
    )
    return output


def _write_json_atomic(path, document):
    """Atomically write one audit report."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.parent / (
        f'.{path.name}.tmp-{os.getpid()}-{uuid.uuid4().hex}'
    )
    try:
        with temporary.open('x', encoding='utf-8') as output:
            json.dump(document, output, indent=2, sort_keys=True)
            output.write('\n')
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def audit_session(bundle, session, pairs, write_overlays=False):
    """Analyze every manifest pair and return an auditable report."""
    policy = load_audit_policy(bundle)
    pair_results = []
    overlay_directory = session / 'quality_overlays'
    if write_overlays:
        overlay_directory.mkdir(exist_ok=True)

    for pair in pairs:
        image = cv2.imread(str(pair['image_path']), cv2.IMREAD_COLOR)
        if image is None:
            raise CalibrationError(f'Cannot decode {pair["image_path"]}.')
        image_result, corners = analyze_image(image, bundle.camera, policy)
        cloud_result = analyze_cloud(pair['pointcloud_path'])
        failures, warnings = _pair_reasons(
            pair, image_result, cloud_result, bundle, policy
        )
        result = {
            'index': pair['index'],
            'image_file': str(pair['image_path'].relative_to(session)),
            'pointcloud_file': str(
                pair['pointcloud_path'].relative_to(session)
            ),
            'image_minus_lidar_ms': (
                float(pair.get('image_minus_lidar_ns', 0)) / 1e6
            ),
            'candidate_usable': not failures,
            'failures': failures,
            'warnings': warnings,
            'possible_duplicate_of': None,
            'image': image_result,
            'cloud': cloud_result,
        }
        pair_results.append(result)
        if write_overlays:
            overlay = _draw_audit_overlay(image, corners, result)
            destination = overlay_directory / f'pair_{pair["index"]:04d}.jpg'
            if not cv2.imwrite(str(destination), overlay):
                raise CalibrationError(f'Cannot write {destination}.')

    duplicate_count = mark_duplicate_poses(pair_results, policy)
    detected = sum(
        item['image']['checkerboard_detected'] for item in pair_results
    )
    candidates = sum(item['candidate_usable'] for item in pair_results)
    translations = [
        item['image']['board_translation_camera_m']
        for item in pair_results
        if item['image']['board_translation_camera_m'] is not None
    ]
    centers = [
        item['image']['board_center_normalized']
        for item in pair_results
        if item['image']['board_center_normalized'] is not None
    ]
    normals = [
        item['image']['board_normal_camera']
        for item in pair_results
        if item['image']['board_normal_camera'] is not None
    ]
    sync_deltas = [
        abs(item['image_minus_lidar_ms']) for item in pair_results
    ]
    summary = {
        'pair_count': len(pair_results),
        'checkerboard_detected_count': int(detected),
        'candidate_usable_count': int(candidates),
        'possible_duplicate_pose_count': duplicate_count,
        'diverse_camera_pose_count': int(detected) - duplicate_count,
        'sync_abs_ms': {
            'minimum': float(np.min(sync_deltas)),
            'median': float(np.median(sync_deltas)),
            'p95': float(np.percentile(sync_deltas, 95)),
            'maximum': float(np.max(sync_deltas)),
        },
        'board_distance_m': None,
        'board_center_normalized_range': None,
        'board_normal_max_separation_deg': None,
        'important_limitation': (
            'Candidate usable means image target, timing, frames, and general '
            'cloud structure passed. It does not prove the checkerboard was '
            'hit or isolated in the LiDAR cloud.'
        ),
    }
    if translations:
        distances = np.linalg.norm(np.asarray(translations), axis=1)
        summary['board_distance_m'] = {
            'minimum': float(distances.min()),
            'maximum': float(distances.max()),
        }
    if centers:
        center_array = np.asarray(centers)
        summary['board_center_normalized_range'] = {
            'x': [
                float(center_array[:, 0].min()),
                float(center_array[:, 0].max()),
            ],
            'y': [
                float(center_array[:, 1].min()),
                float(center_array[:, 1].max()),
            ],
        }
    if len(normals) >= 2:
        separations = [
            _normal_angle_degrees(normals[first], normals[second])
            for first in range(len(normals))
            for second in range(first + 1, len(normals))
        ]
        summary['board_normal_max_separation_deg'] = float(
            max(separations)
        )
    return {
        'schema_version': 1,
        'created_at_utc': datetime.now(timezone.utc).isoformat(),
        'capture_session': str(session),
        'calibration_config': str(bundle.path),
        'camera_config': str(bundle.camera.source_path),
        'target': bundle.raw['target'],
        'policy': policy,
        'summary': summary,
        'pairs': pair_results,
    }


def parse_arguments(arguments=None):
    """Parse command-line options for the dataset auditor."""
    parser = argparse.ArgumentParser(
        description=(
            'Audit calibration pair usefulness from manifest data.'
        )
    )
    parser.add_argument(
        '--session', required=True, help='Capture session path'
    )
    parser.add_argument('--config', help='calibrate.yaml path')
    parser.add_argument(
        '--write-overlays',
        action='store_true',
        help='Write images with detected checkerboard corners',
    )
    parser.add_argument(
        '--output', help='Report path (default: SESSION/quality_report.json)'
    )
    return parser.parse_args(arguments)


def main(args=None):
    """Run the manifest-based calibration dataset audit."""
    options = parse_arguments(args)
    try:
        config = options.config or default_config_path()
        bundle = load_calibration_bundle(config, require_valid=False)
        session, pairs = load_session_pairs(options.session)
        report = audit_session(
            bundle, session, pairs, write_overlays=options.write_overlays
        )
        destination = (
            Path(options.output).expanduser().resolve()
            if options.output
            else session / 'quality_report.json'
        )
        _write_json_atomic(destination, report)
        summary = report['summary']
        print(f'Calibration quality report: {destination}')
        print(
            f'checkerboard={summary["checkerboard_detected_count"]}/'
            f'{summary["pair_count"]}, '
            f'candidates={summary["candidate_usable_count"]}, '
            f'possible_duplicates='
            f'{summary["possible_duplicate_pose_count"]}'
        )
        print(f'LIMITATION: {summary["important_limitation"]}')
    except (CalibrationError, OSError, ValueError) as error:
        print(f'calibration_audit: {error}', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
