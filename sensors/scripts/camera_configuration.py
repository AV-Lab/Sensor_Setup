"""Configuration loading and validation for the camera publisher."""

import math
import os

from ament_index_python.packages import get_package_share_directory
import yaml


def load_camera_config(node):
    """Load camera YAML from a ROS override or the package share directory."""
    node.declare_parameter('config_file', '')
    configured_path = node.get_parameter(
        'config_file'
    ).get_parameter_value().string_value

    if configured_path:
        path = os.path.abspath(os.path.expanduser(configured_path))
    else:
        package_share = get_package_share_directory('sensors')
        path = os.path.join(package_share, 'config', 'camera_config.yaml')

    if not os.path.isfile(path):
        raise FileNotFoundError(f'Camera configuration does not exist: {path}')

    with open(path, 'r', encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file)
    if not isinstance(config, dict):
        raise ValueError('Camera configuration must be a YAML mapping.')

    validate_camera_config(config)
    node.get_logger().info(f'Using camera configuration: {path}')
    return config


def validate_camera_config(config):
    """Validate values that affect capture and calibration correctness."""
    sections = ('camera', 'capture', 'controls', 'ROS', 'qos', 'intrinsics')
    for section in sections:
        if not isinstance(config.get(section), dict):
            raise ValueError(
                f'Missing camera configuration section: {section}'
            )

    camera = config['camera']
    capture = config['capture']
    intrinsics = config['intrinsics']

    device = camera.get('device')
    if isinstance(device, bool) or not isinstance(device, (int, str)):
        raise ValueError('camera.device must be an index, path, or "auto".')
    if isinstance(device, int) and device < 0:
        raise ValueError('camera.device must not be negative.')
    if isinstance(device, str) and not device:
        raise ValueError('camera.device must not be empty.')
    if device == 'auto' and not camera.get('device_match'):
        raise ValueError(
            'camera.device_match is required when device is auto.'
        )

    for name in ('width', 'height'):
        _require_integer(camera, name, minimum=1, prefix='camera')
    _require_number(camera, 'fps', minimum=0.001, prefix='camera')

    pixel_format = camera.get('pixel_format')
    if not isinstance(pixel_format, str) or len(pixel_format) != 4:
        raise ValueError('camera.pixel_format must contain four characters.')
    camera['pixel_format'] = pixel_format.upper()

    if camera.get('backend') != 'V4L2':
        raise ValueError('camera.backend must be V4L2.')
    if not isinstance(camera.get('strict_mode'), bool):
        raise ValueError('camera.strict_mode must be true or false.')

    _require_integer(capture, 'buffer_size', minimum=1, prefix='capture')
    _require_integer(capture, 'warmup_frames', minimum=0, prefix='capture')
    _require_integer(
        capture,
        'max_consecutive_failures',
        minimum=1,
        prefix='capture',
    )
    _require_number(
        capture,
        'diagnostics_interval_sec',
        minimum=0.001,
        prefix='capture',
    )

    if not isinstance(intrinsics.get('calibrated'), bool):
        raise ValueError('intrinsics.calibrated must be true or false.')
    if intrinsics.get('distortion_model') != 'plumb_bob':
        raise ValueError('intrinsics.distortion_model must be plumb_bob.')
    lengths = {
        'camera_matrix_K': 9,
        'rectification': 9,
        'projection': 12,
    }
    for name, length in lengths.items():
        values = intrinsics.get(name)
        if not isinstance(values, list) or len(values) != length:
            raise ValueError(
                f'intrinsics.{name} must contain {length} values.'
            )
        _require_finite_values(values, f'intrinsics.{name}')
    distortion = intrinsics.get('distortion')
    if not isinstance(distortion, list) or len(distortion) not in {
        4, 5, 8, 12, 14,
    }:
        raise ValueError(
            'intrinsics.distortion needs 4, 5, 8, 12, or 14 values.'
        )
    _require_finite_values(distortion, 'intrinsics.distortion')
    if intrinsics['calibrated']:
        provenance = intrinsics.get('provenance')
        if not isinstance(provenance, dict):
            raise ValueError(
                'Calibrated intrinsics need a provenance mapping.'
            )
        method = provenance.get('method')
        if (
            not isinstance(method, str)
            or not method
            or method == 'placeholder'
        ):
            raise ValueError(
                'Calibrated intrinsics need a non-placeholder '
                'provenance.method.'
            )
        matrix = intrinsics['camera_matrix_K']
        if matrix[0] <= 0.0 or matrix[4] <= 0.0:
            raise ValueError(
                'Calibrated camera focal lengths must be positive.'
            )
        if matrix[6:9] != [0.0, 0.0, 1.0]:
            raise ValueError(
                'intrinsics.camera_matrix_K must end with [0, 0, 1].'
            )


def _require_finite_values(values, name):
    """Require a numeric finite calibration array."""
    if any(
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        for value in values
    ):
        raise ValueError(f'{name} must contain only finite numbers.')


def _require_integer(mapping, name, minimum, prefix):
    """Require one integer configuration value with a lower bound."""
    value = mapping.get(name)
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or value < minimum
    ):
        raise ValueError(f'{prefix}.{name} must be an integer >= {minimum}.')


def _require_number(mapping, name, minimum, prefix):
    """Require one numeric configuration value with a lower bound."""
    value = mapping.get(name)
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or value < minimum
    ):
        raise ValueError(f'{prefix}.{name} must be >= {minimum}.')
