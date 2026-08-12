"""Tests for synchronized calibration capture conversion and artifacts."""

import json

import cv2
import numpy as np
import pytest
from sensor_msgs.msg import Image, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from scripts.save_samples import (
    CaptureSession,
    image_message_to_bgr,
    make_binary_pcd,
    pointcloud_message_to_array,
    validate_save_config,
)


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
            'pointcloud_topic_name': '/points',
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
            'output_root': str(tmp_path),
            'session_name': 'test_session',
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


def test_capture_session_never_reuses_an_existing_name(tmp_path):
    """A new run must not overwrite an earlier calibration session."""
    CaptureSession(tmp_path, 'fixed_name', {'schema_version': 1})

    with pytest.raises(FileExistsError):
        CaptureSession(tmp_path, 'fixed_name', {'schema_version': 1})
