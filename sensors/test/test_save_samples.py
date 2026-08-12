"""Tests for synchronized calibration capture conversion and artifacts."""

import json

import cv2
import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo, Image, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String

from scripts.save_samples import (
    CaptureSession,
    camera_info_message_to_document,
    decode_runtime_metadata,
    image_message_to_bgr,
    make_binary_pcd,
    pointcloud_message_to_array,
    validate_save_config,
)
from scripts.v4l2_camera import control_readback_matches


def _cloud(fields, points):
    """Create a float32 PointCloud2 fixture."""
    point_fields = [
        PointField(
            name=name,
            offset=index * 4,
            datatype=PointField.FLOAT32,
            count=1,
        )
        for index, name in enumerate(fields)
    ]
    return point_cloud2.create_cloud(
        Header(), point_fields, np.asarray(points, dtype=np.float32)
    )


def _valid_config(tmp_path):
    """Return a minimal valid saver configuration."""
    return {
        'ROS': {
            'image_topic_name': '/image',
            'camera_info_topic_name': '/camera_info',
            'pointcloud_topic_name': '/points',
            'camera_runtime_metadata_topic': '/camera/runtime_metadata',
            'lidar_runtime_metadata_topic': '/lidar/runtime_metadata',
            'save_service_name': '/save',
        },
        'Sync': {'threshold_sec': 0.025, 'queue_size': 10},
        'Capture': {
            'mode': 'manual',
            'total_samples': 3,
            'min_interval_sec': 1.0,
            'max_pair_age_sec': 0.25,
            'min_point_count': 1,
            'diagnostics_interval_sec': 5.0,
            'require_runtime_metadata': True,
            'require_calibrated_camera_info': True,
            'output_root': str(tmp_path),
            'session_name': 'test_session',
        },
        'ManualPreview': {
            'enabled': True,
            'display': False,
            'analysis_interval_sec': 0.5,
            'calibration_config': 'calibrate.yaml',
            'window_name': 'test preview',
        },
    }


def test_image_conversion_respects_row_padding():
    """V4L2 row padding must not become visible image pixels."""
    message = Image()
    message.height = 1
    message.width = 2
    message.encoding = 'bgr8'
    message.step = 8
    message.data = bytes([1, 2, 3, 4, 5, 6, 99, 99])

    image = image_message_to_bgr(message)

    assert image.flags['C_CONTIGUOUS']
    assert image.tolist() == [[[1, 2, 3], [4, 5, 6]]]


def test_rgb_image_is_converted_to_bgr():
    """Saved PNG colors must match an RGB ROS source."""
    message = Image()
    message.height = 1
    message.width = 1
    message.encoding = 'rgb8'
    message.step = 3
    message.data = bytes([255, 0, 0])

    assert image_message_to_bgr(message).tolist() == [[[0, 0, 255]]]


def test_pointcloud_prefers_reflectivity_and_filters_invalid_points():
    """The custom Ouster reflectivity field must survive PCD conversion."""
    message = _cloud(
        ('x', 'y', 'z', 'reflectivity'),
        [
            [1.0, 2.0, 3.0, 42.0],
            [0.0, 0.0, 0.0, 99.0],
            [float('nan'), 1.0, 2.0, 8.0],
        ],
    )

    points, value_field = pointcloud_message_to_array(message)

    assert value_field == 'reflectivity'
    np.testing.assert_array_equal(
        points, np.array([[1.0, 2.0, 3.0, 42.0]], dtype=np.float32)
    )


def test_pointcloud_accepts_official_xyzi_intensity_field():
    """The saver remains compatible with an official-driver XYZI cloud."""
    message = _cloud(
        ('x', 'y', 'z', 'intensity'),
        [[1.0, 2.0, 3.0, 7.0]],
    )

    points, value_field = pointcloud_message_to_array(message)

    assert value_field == 'intensity'
    assert points[0, 3] == 7.0


def test_binary_pcd_preserves_four_float_fields():
    """Binary PCD output must retain the scalar calibration channel."""
    points = np.array([[1.0, 2.0, 3.0, 4.0]], dtype=np.float32)

    pcd = make_binary_pcd(points, 'reflectivity')
    header, payload = pcd.split(b'DATA binary\n', maxsplit=1)

    assert b'FIELDS x y z reflectivity' in header
    np.testing.assert_array_equal(
        np.frombuffer(payload, dtype='<f4').reshape(-1, 4), points
    )


def test_camera_info_document_preserves_active_projection():
    """The session provenance retains the exact synchronized CameraInfo."""
    message = CameraInfo()
    message.header.stamp.sec = 12
    message.header.stamp.nanosec = 34
    message.header.frame_id = 'camera_optical_frame'
    message.width = 640
    message.height = 480
    message.distortion_model = 'plumb_bob'
    message.d = [0.1, -0.2, 0.0, 0.0, 0.0]
    message.k = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
    message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    message.p = [
        500.0, 0.0, 320.0, 0.0,
        0.0, 500.0, 240.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
    ]

    document = camera_info_message_to_document(message)

    assert document['stamp_ns'] == 12_000_000_034
    assert document['frame_id'] == 'camera_optical_frame'
    assert document['calibrated'] is True
    assert document['k'] == list(message.k)
    assert document['p'] == list(message.p)


def test_runtime_metadata_must_be_a_json_object():
    """Malformed durable publisher metadata must fail closed."""
    valid = String(data='{"schema_version": 1, "device": "test"}')
    assert decode_runtime_metadata(valid, 'camera')['device'] == 'test'

    with pytest.raises(ValueError, match='JSON object'):
        decode_runtime_metadata(String(data='[1, 2]'), 'camera')
    with pytest.raises(ValueError, match='invalid JSON'):
        decode_runtime_metadata(String(data='{'), 'camera')
    with pytest.raises(ValueError, match='invalid JSON'):
        decode_runtime_metadata(String(data='{"value": NaN}'), 'camera')


def test_runtime_metadata_verifies_expected_publisher():
    """A similarly named topic cannot silently provide another node's data."""
    message = String(data=json.dumps({
        'schema_version': 1,
        'publisher': 'unexpected.node',
    }))

    with pytest.raises(ValueError, match='did not come from'):
        decode_runtime_metadata(message, 'camera', 'sensors.camera_node')


def test_control_readback_comparison_allows_small_backend_quantization():
    """Control verification accepts small quantization but rejects drift."""
    assert control_readback_matches(100.0, 101.0)
    assert not control_readback_matches(100.0, 110.0)
    assert not control_readback_matches(1.0, float('nan'))


@pytest.mark.parametrize(
    ('section', 'key', 'value'),
    [
        ('Capture', 'mode', 'unknown'),
        ('Capture', 'total_samples', 0),
        ('Sync', 'threshold_sec', '0.025'),
        ('Sync', 'queue_size', True),
    ],
)
def test_configuration_rejects_unsafe_values(
    tmp_path, section, key, value
):
    """Mistyped capture limits must fail instead of being coerced."""
    config = _valid_config(tmp_path)
    config[section][key] = value

    with pytest.raises(ValueError):
        validate_save_config(config)


def test_capture_session_writes_complete_auditable_pair(tmp_path):
    """A completed pair has two files and one matching manifest record."""
    session = CaptureSession(
        tmp_path,
        'test_session',
        {'schema_version': 1, 'configuration': {}},
    )
    image = np.zeros((3, 4, 3), dtype=np.uint8)
    image[:, :, 1] = 255
    points = np.array([[1.0, 2.0, 3.0, 4.0]], dtype=np.float32)
    metadata = {
        'image_stamp_ns': 1_020,
        'lidar_stamp_ns': 1_000,
        'absolute_stamp_delta_ns': 20,
    }

    image_path, pointcloud_path = session.write_pair(
        0, image, points, 'reflectivity', metadata
    )

    assert cv2.imread(str(image_path)).shape == image.shape
    assert pointcloud_path.read_bytes().startswith(b'# .PCD v0.7')
    records = [
        json.loads(line)
        for line in session.manifest_path.read_text(
            encoding='utf-8'
        ).splitlines()
    ]
    assert records == [{
        **metadata,
        'image_file': 'images/img_0000.png',
        'index': 0,
        'pointcloud_file': 'pcds/pc_0000.pcd',
    }]

    with pytest.raises(FileExistsError):
        session.write_pair(0, image, points, 'reflectivity', metadata)


def test_capture_session_archives_immutable_runtime_provenance(tmp_path):
    """Active sensor metadata is archived once and cannot drift silently."""
    session = CaptureSession(
        tmp_path,
        'metadata_session',
        {'schema_version': 1},
    )
    provenance = {
        'camera_info': {'k': [500.0]},
        'camera_publisher': {'device': '/dev/video0'},
        'lidar_publisher': {'serial': '1234'},
    }

    session.write_runtime_provenance(provenance)
    session.write_runtime_provenance(provenance)

    archived = json.loads(
        session.runtime_metadata_path.read_text(encoding='utf-8')
    )
    assert archived == provenance
    changed = {**provenance, 'lidar_publisher': {'serial': 'different'}}
    with pytest.raises(ValueError, match='changed during the session'):
        session.write_runtime_provenance(changed)


def test_capture_session_never_reuses_an_existing_name(tmp_path):
    """A new run must not overwrite an earlier calibration session."""
    CaptureSession(tmp_path, 'fixed_name', {'schema_version': 1})

    with pytest.raises(FileExistsError):
        CaptureSession(tmp_path, 'fixed_name', {'schema_version': 1})
