"""Tests for calibration configuration, review, and dataset auditing."""

import json
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

from scripts.calibration_audit import analyze_image
from scripts.calibration_audit import audit_session
from scripts.calibration_audit import load_audit_policy
from scripts.calibration_audit import verify_session_provenance
from scripts.calibration_common import CalibrationError
from scripts.calibration_common import load_calibration_bundle
from scripts.calibration_common import load_session_pairs
from scripts.calibration_common import matrix_to_quaternion_xyzw
from scripts.calibration_common import project_points
from scripts.calibration_common import read_pcd
from scripts.calibration_tf_publisher import source_parent_transform
from scripts.camera_configuration import validate_camera_config
from scripts.interactive_calibration import apply_refinement
from scripts.lidar_target_detection import detect_lidar_checkerboard
from scripts.lidar_target_detection import load_lidar_detection_policy
from scripts.manual_capture_preview import evaluate_manual_preview
from scripts.manual_capture_preview import load_preview_contract
from scripts.manual_capture_preview import render_manual_preview


def _write_yaml(path, document):
    """Write a small test YAML document."""
    path.write_text(yaml.safe_dump(document), encoding='utf-8')


def _camera_document(calibrated=True):
    """Return a compact calibrated-camera configuration."""
    return {
        'camera': {'width': 640, 'height': 480},
        'intrinsics': {
            'calibrated': calibrated,
            'distortion_model': 'plumb_bob',
            'camera_matrix_K': [
                500.0, 0.0, 320.0,
                0.0, 500.0, 240.0,
                0.0, 0.0, 1.0,
            ],
            'distortion': [0.0, 0.0, 0.0, 0.0, 0.0],
            'provenance': {
                'method': 'opencv_test' if calibrated else 'placeholder',
            },
        },
    }


def _calibration_document(valid=True):
    """Return a schema-v2 calibration contract for tests."""
    return {
        'schema_version': 2,
        'extrinsic': {
            'valid': valid,
            'method': 'test',
            'source_frame': 'os_sensor',
            'target_frame': 'camera_optical_frame',
            'convention': 'p_target = T_target_source * p_source',
            'translation_unit': 'm',
            'matrix': np.eye(4).reshape(-1).tolist(),
        },
        'camera': {'config_file': 'camera_config.yaml'},
        'target': {
            'type': 'checkerboard',
            'inner_corners': [8, 6],
            'square_size_m': 0.04,
        },
        'quality': {
            'max_abs_sync_delta_ms': 25.0,
            'min_cloud_points': 1000,
            'min_sharpness_variance': 10.0,
            'max_dark_fraction': 0.75,
            'max_bright_fraction': 0.75,
            'min_board_image_fraction': 0.01,
            'duplicate_translation_m': 0.1,
            'duplicate_normal_angle_deg': 5.0,
        },
    }


def _write_bundle(tmp_path, valid=True, calibrated=True):
    """Write linked camera and calibration files."""
    camera_path = tmp_path / 'camera_config.yaml'
    calibration_path = tmp_path / 'calibrate.yaml'
    _write_yaml(camera_path, _camera_document(calibrated))
    _write_yaml(calibration_path, _calibration_document(valid))
    return calibration_path


def _checkerboard_image():
    """Render an 8x6-inner-corner checkerboard at the test resolution."""
    image = np.full((480, 640, 3), 210, dtype=np.uint8)
    square = 50
    left = 95
    top = 65
    for row in range(7):
        for column in range(9):
            value = 25 if (row + column) % 2 == 0 else 235
            start = (left + column * square, top + row * square)
            end = (start[0] + square, start[1] + square)
            cv2.rectangle(image, start, end, (value, value, value), -1)
    return image


def _pcd_bytes(points):
    """Encode XYZ float32 points using the saver-compatible binary layout."""
    points = np.asarray(points, dtype='<f4')
    header = (
        '# .PCD v0.7\n'
        'VERSION 0.7\n'
        'FIELDS x y z\n'
        'SIZE 4 4 4\n'
        'TYPE F F F\n'
        'COUNT 1 1 1\n'
        f'WIDTH {len(points)}\n'
        'HEIGHT 1\n'
        f'POINTS {len(points)}\n'
        'DATA binary\n'
    ).encode('ascii')
    return header + points.tobytes()


def test_placeholder_transform_is_refused(tmp_path):
    """Review and TF use cannot silently consume placeholder extrinsics."""
    path = _write_bundle(tmp_path, valid=False)
    with pytest.raises(CalibrationError, match='placeholder'):
        load_calibration_bundle(path, require_valid=True)
    bundle = load_calibration_bundle(path, require_valid=False)
    assert bundle.extrinsic.valid is False


def test_uncalibrated_camera_is_refused(tmp_path):
    """All tools fail closed when camera intrinsics remain placeholders."""
    path = _write_bundle(tmp_path, calibrated=False)
    with pytest.raises(CalibrationError, match='calibrated must be true'):
        load_calibration_bundle(path)


def test_calibrated_camera_needs_real_provenance(tmp_path):
    """A validity toggle alone cannot make placeholder intrinsics trusted."""
    path = _write_bundle(tmp_path)
    camera_path = tmp_path / 'camera_config.yaml'
    camera = yaml.safe_load(camera_path.read_text(encoding='utf-8'))
    camera['intrinsics']['provenance']['method'] = 'placeholder'
    _write_yaml(camera_path, camera)
    with pytest.raises(CalibrationError, match='provenance.method'):
        load_calibration_bundle(path)


def test_repository_camera_placeholder_is_valid_but_not_calibrated():
    """The shipped camera placeholder is explicit and structurally valid."""
    config_path = Path(__file__).parents[1] / 'config' / 'camera_config.yaml'
    config = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    validate_camera_config(config)
    assert config['intrinsics']['calibrated'] is False
    assert config['intrinsics']['provenance']['method'] == 'placeholder'


def test_projection_filters_points_behind_camera(tmp_path):
    """Raw-image projection uses the camera model and positive depth only."""
    bundle = load_calibration_bundle(_write_bundle(tmp_path))
    points = np.array([
        [0.0, 0.0, 1.0],
        [0.2, 0.0, 1.0],
        [0.0, 0.0, -1.0],
    ])
    pixels, depths, forward = project_points(
        points, np.eye(4), bundle.camera
    )
    assert np.allclose(pixels, [[320.0, 240.0], [420.0, 240.0]])
    assert np.allclose(depths, [1.0, 1.0])
    assert forward.tolist() == [True, True, False]


def test_refinement_axes_are_explicit_and_symmetric():
    """Refinement commands use the same axis for each plus/minus pair."""
    identity = np.eye(4)
    translated = apply_refinement(identity, 'ty+', 0.001, 0.01)
    assert np.allclose(translated[:3, 3], [0.0, 0.001, 0.0])
    restored = apply_refinement(translated, 'ty-', 0.001, 0.01)
    assert np.allclose(restored, identity)
    rotated = apply_refinement(identity, 'pitch+', 0.001, 0.01)
    restored = apply_refinement(rotated, 'pitch-', 0.001, 0.01)
    assert np.allclose(restored, identity)


def test_quaternion_conversion_is_normalized():
    """Static-TF quaternion conversion preserves identity and normalization."""
    identity = matrix_to_quaternion_xyzw(np.eye(3))
    assert np.allclose(identity, [0.0, 0.0, 0.0, 1.0])
    rotation, _ = cv2.Rodrigues(np.array([0.2, -0.1, 0.3]))
    quaternion = matrix_to_quaternion_xyzw(rotation)
    assert np.isclose(np.linalg.norm(quaternion), 1.0)


def test_static_tf_uses_inverse_for_source_parent(tmp_path):
    """Invert the stored source-to-target map for a source-parent TF."""
    calibration = _calibration_document()
    matrix = np.eye(4)
    matrix[:3, 3] = [1.0, -2.0, 3.0]
    calibration['extrinsic']['matrix'] = matrix.reshape(-1).tolist()
    _write_yaml(tmp_path / 'camera_config.yaml', _camera_document())
    path = tmp_path / 'calibrate.yaml'
    _write_yaml(path, calibration)
    bundle = load_calibration_bundle(path)
    tf_matrix = source_parent_transform(bundle)
    assert np.allclose(tf_matrix @ matrix, np.eye(4))


def test_binary_pcd_reader_preserves_xyz(tmp_path):
    """The local PCD reader consumes the exact saver binary format."""
    path = tmp_path / 'points.pcd'
    expected = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
    path.write_bytes(_pcd_bytes(expected))
    actual = read_pcd(path, xyz_only=True)
    assert np.allclose(actual, expected)


def test_image_audit_detects_checkerboard_and_pose(tmp_path):
    """The pair audit detects the configured board and estimates its pose."""
    bundle = load_calibration_bundle(_write_bundle(tmp_path))
    policy = load_audit_policy(bundle)
    result, corners = analyze_image(
        _checkerboard_image(), bundle.camera, policy
    )
    assert result['checkerboard_detected'] is True
    assert corners.shape == (48, 2)
    assert result['board_image_fraction'] > 0.1
    assert result['board_distance_m'] > 0.0
    assert result['board_reprojection_rmse_px'] < 1.0


def _synthetic_lidar_checkerboard(uniform=False):
    """Return a dense planar 9x7-square reflectivity target."""
    random_generator = np.random.default_rng(4)
    square_size = 0.04
    points = []
    reflectivity = []
    for column in range(9):
        for row in range(7):
            for u in np.linspace(0.05, 0.95, 4):
                for v in np.linspace(0.05, 0.95, 4):
                    points.append([
                        (column + u) * square_size - 0.18,
                        (row + v) * square_size - 0.14,
                        3.0 + random_generator.normal(0.0, 0.001),
                    ])
                    value = 100.0
                    if not uniform:
                        value = 220.0 if (column + row) % 2 else 20.0
                    reflectivity.append(value)
    return np.asarray(points), np.asarray(reflectivity)


def test_lidar_detector_finds_checkerboard_like_candidate():
    """Expected plane dimensions plus alternating reflectivity are screened."""
    target = _calibration_document()['target']
    policy = load_lidar_detection_policy({'enabled': True})
    points, reflectivity = _synthetic_lidar_checkerboard()

    result = detect_lidar_checkerboard(
        points, reflectivity, target, policy, 'reflectivity'
    )

    assert result['status'] == 'candidate'
    assert result['requires_manual_confirmation'] is True
    assert result['candidate']['passes_thresholds'] is True
    assert result['candidate']['checker_classification_accuracy'] > 0.9


def test_lidar_detector_rejects_uniform_planar_patch():
    """Plane shape alone is not enough to claim a target candidate."""
    target = _calibration_document()['target']
    policy = load_lidar_detection_policy({'enabled': True})
    points, reflectivity = _synthetic_lidar_checkerboard(uniform=True)

    result = detect_lidar_checkerboard(
        points, reflectivity, target, policy, 'reflectivity'
    )

    assert result['status'] == 'not_detected'


def test_manual_preview_renders_both_detection_results():
    """Manual preview clearly reports when both synthetic targets are found."""
    target = _calibration_document()['target']
    policy = load_lidar_detection_policy({'enabled': True})
    xyz, reflectivity = _synthetic_lidar_checkerboard()
    points = np.column_stack((xyz, reflectivity)).astype(np.float32)
    image = _checkerboard_image()

    result, corners = evaluate_manual_preview(
        image,
        points,
        'reflectivity',
        target,
        policy,
        3.5,
    )
    preview = render_manual_preview(
        image, points, result, corners, target, policy
    )

    assert result['ready'] is True
    assert result['advisory_only'] is True
    assert result['camera_corner_count'] == 48
    assert result['lidar']['status'] == 'candidate'
    assert preview.shape == (544, 1280, 3)


def test_preview_contract_uses_calibration_target_source(tmp_path):
    """Preview target dimensions come from calibrate.yaml, not a duplicate."""
    document = _calibration_document()
    document['lidar_target_detection'] = {'enabled': True}
    path = tmp_path / 'calibrate.yaml'
    _write_yaml(path, document)

    target, policy = load_preview_contract(path)

    assert target == document['target']
    assert policy['enabled'] is True


def _runtime_provenance_document():
    """Return runtime metadata matching the synthetic camera bundle."""
    return {
        'schema_version': 1,
        'publisher_metadata_complete': True,
        'camera_info': {
            'frame_id': 'camera_optical_frame',
            'width': 640,
            'height': 480,
            'distortion_model': 'plumb_bob',
            'd': [0.0, 0.0, 0.0, 0.0, 0.0],
            'k': [
                500.0, 0.0, 320.0,
                0.0, 500.0, 240.0,
                0.0, 0.0, 1.0,
            ],
        },
        'camera_publisher': {
            'publisher': 'sensors.camera_node',
            'camera': {'applied_standard_controls_verified': True},
        },
        'lidar_publisher': {
            'publisher': 'sensors.ouster_node',
            'ros_policy': {'frame_id': 'os_sensor'},
        },
    }


def test_session_provenance_must_match_loaded_intrinsics(tmp_path):
    """Audit cannot silently use a camera model different from capture."""
    bundle = load_calibration_bundle(_write_bundle(tmp_path))
    session = tmp_path / 'session'
    session.mkdir()
    (session / 'session.json').write_text(
        json.dumps({'runtime_metadata_file': 'runtime_metadata.json'}),
        encoding='utf-8',
    )
    runtime_path = session / 'runtime_metadata.json'
    runtime = _runtime_provenance_document()
    runtime_path.write_text(json.dumps(runtime), encoding='utf-8')

    result = verify_session_provenance(session, bundle)

    assert result['status'] == 'verified'
    assert result['camera_controls_applied_and_verified'] is True
    runtime['camera_info']['k'][0] = 700.0
    runtime_path.write_text(json.dumps(runtime), encoding='utf-8')
    with pytest.raises(CalibrationError, match='does not match'):
        verify_session_provenance(session, bundle)


def test_session_provenance_reports_allowed_legacy_publishers(tmp_path):
    """An explicitly incomplete archive remains auditable but not verified."""
    bundle = load_calibration_bundle(_write_bundle(tmp_path))
    session = tmp_path / 'legacy_session'
    session.mkdir()
    runtime = _runtime_provenance_document()
    runtime['publisher_metadata_complete'] = False
    runtime['camera_publisher'] = None
    runtime['lidar_publisher'] = None
    (session / 'runtime_metadata.json').write_text(
        json.dumps(runtime), encoding='utf-8'
    )

    result = verify_session_provenance(session, bundle)

    assert result['status'] == 'publisher_metadata_incomplete'


def test_session_audit_uses_manifest_and_reports_limitation(tmp_path):
    """A complete synthetic session produces an honest quality report."""
    calibration_path = _write_bundle(tmp_path, valid=False)
    bundle = load_calibration_bundle(calibration_path, require_valid=False)
    session = tmp_path / 'session'
    images = session / 'images'
    clouds = session / 'pcds'
    images.mkdir(parents=True)
    clouds.mkdir()
    image_path = images / 'img_0000.png'
    cloud_path = clouds / 'pc_0000.pcd'
    assert cv2.imwrite(str(image_path), _checkerboard_image())
    points = np.column_stack((
        np.linspace(1.0, 2.0, 1200),
        np.zeros(1200),
        np.ones(1200),
    ))
    cloud_path.write_bytes(_pcd_bytes(points))
    record = {
        'index': 0,
        'image_file': 'images/img_0000.png',
        'pointcloud_file': 'pcds/pc_0000.pcd',
        'image_minus_lidar_ns': 10_000_000,
        'image_frame_id': 'camera_optical_frame',
        'lidar_frame_id': 'os_sensor',
    }
    (session / 'manifest.jsonl').write_text(
        json.dumps(record) + '\n', encoding='utf-8'
    )
    resolved_session, pairs = load_session_pairs(session)
    report = audit_session(bundle, resolved_session, pairs)
    assert report['summary']['checkerboard_detected_count'] == 1
    assert report['summary']['candidate_usable_count'] == 1
    assert report['pairs'][0]['cloud']['actual_point_count'] == 1200
    assert report['runtime_provenance']['status'] == 'legacy_metadata_missing'
    assert 'not proof' in report['summary']['important_limitation']
