"""Shared, fail-safe helpers for camera-to-LiDAR calibration tools."""

from dataclasses import dataclass
import json
from pathlib import Path

import cv2
import numpy as np
import yaml


class CalibrationError(ValueError):
    """Report an invalid calibration, camera model, or capture session."""


@dataclass(frozen=True)
class CameraModel:
    """Describe the calibrated raw-image camera model."""

    width: int
    height: int
    matrix: np.ndarray
    distortion: np.ndarray
    distortion_model: str
    source_path: Path


@dataclass(frozen=True)
class ExtrinsicCalibration:
    """Describe one explicitly directed rigid transform."""

    source_frame: str
    target_frame: str
    matrix: np.ndarray
    valid: bool
    method: str


@dataclass(frozen=True)
class CalibrationBundle:
    """Hold the validated review configuration and camera model."""

    path: Path
    raw: dict
    camera: CameraModel
    extrinsic: ExtrinsicCalibration


def default_config_path():
    """Return the installed package's default calibration configuration."""
    from ament_index_python.packages import get_package_share_directory

    share = Path(get_package_share_directory('sensors'))
    return share / 'config' / 'calibrate.yaml'


def load_yaml(path):
    """Read one YAML mapping or raise a useful configuration error."""
    path = Path(path).expanduser().resolve()
    try:
        with path.open('r', encoding='utf-8') as stream:
            document = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as error:
        raise CalibrationError(f'Cannot read {path}: {error}') from error
    if not isinstance(document, dict):
        raise CalibrationError(f'{path} must contain a YAML mapping.')
    return path, document


def _require_mapping(mapping, key, context):
    """Return one required nested mapping."""
    value = mapping.get(key)
    if not isinstance(value, dict):
        raise CalibrationError(f'{context}.{key} must be a mapping.')
    return value


def _require_string(mapping, key, context):
    """Return one required non-empty string."""
    value = mapping.get(key)
    if not isinstance(value, str) or not value.strip():
        raise CalibrationError(f'{context}.{key} must be a non-empty string.')
    return value.strip()


def _require_positive_integer(mapping, key, context):
    """Return one required positive integer."""
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise CalibrationError(f'{context}.{key} must be a positive integer.')
    return value


def _finite_array(value, shape, name):
    """Convert and validate a finite numeric array."""
    try:
        array = np.asarray(value, dtype=np.float64).reshape(shape)
    except (TypeError, ValueError) as error:
        raise CalibrationError(f'{name} must have shape {shape}.') from error
    if not np.isfinite(array).all():
        raise CalibrationError(f'{name} contains a non-finite value.')
    return array


def validate_rigid_transform(matrix, name='extrinsic.matrix'):
    """Validate and return a homogeneous, right-handed rigid transform."""
    transform = _finite_array(matrix, (4, 4), name)
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        raise CalibrationError(f'{name} must end with [0, 0, 0, 1].')
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-3):
        raise CalibrationError(f'{name} rotation is not orthonormal.')
    determinant = float(np.linalg.det(rotation))
    if not np.isclose(determinant, 1.0, atol=1e-3):
        raise CalibrationError(
            f'{name} rotation determinant is {determinant:.6f}, not +1.'
        )
    return transform


def load_camera_model(camera_path):
    """Load a calibrated raw-image camera model from camera_config.yaml."""
    camera_path, document = load_yaml(camera_path)
    camera = _require_mapping(document, 'camera', 'camera_config')
    intrinsics = _require_mapping(document, 'intrinsics', 'camera_config')
    calibrated = intrinsics.get('calibrated')
    if calibrated is not True:
        raise CalibrationError(
            f'{camera_path}: intrinsics.calibrated must be true before '
            'calibration review.'
        )
    provenance = _require_mapping(
        intrinsics, 'provenance', 'camera_config.intrinsics'
    )
    method = _require_string(
        provenance, 'method', 'camera_config.intrinsics.provenance'
    )
    if method == 'placeholder':
        raise CalibrationError(
            'Camera intrinsics are marked calibrated but provenance.method '
            'is still "placeholder".'
        )

    width = _require_positive_integer(camera, 'width', 'camera_config.camera')
    height = _require_positive_integer(
        camera, 'height', 'camera_config.camera'
    )
    matrix = _finite_array(
        intrinsics.get('camera_matrix_K'),
        (3, 3),
        'intrinsics.camera_matrix_K',
    )
    if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
        raise CalibrationError('Camera focal lengths must be positive.')
    if not np.allclose(matrix[2], [0.0, 0.0, 1.0], atol=1e-9):
        raise CalibrationError('Camera matrix must end with [0, 0, 1].')

    distortion = np.asarray(
        intrinsics.get('distortion'), dtype=np.float64
    ).reshape(-1)
    valid_distortion_lengths = {4, 5, 8, 12, 14}
    if (
        distortion.size not in valid_distortion_lengths
        or not np.isfinite(distortion).all()
    ):
        raise CalibrationError(
            'intrinsics.distortion must contain 4, 5, 8, 12, or 14 '
            'finite coefficients.'
        )
    distortion_model = intrinsics.get('distortion_model', 'plumb_bob')
    if distortion_model != 'plumb_bob':
        raise CalibrationError(
            'Only the OpenCV plumb_bob camera model is supported.'
        )
    return CameraModel(
        width=width,
        height=height,
        matrix=matrix,
        distortion=distortion,
        distortion_model=distortion_model,
        source_path=camera_path,
    )


def load_calibration_bundle(path=None, require_valid=True):
    """Load the frame contract, transform, and referenced camera model."""
    path = default_config_path() if path is None else path
    path, document = load_yaml(path)
    if document.get('schema_version') != 2:
        raise CalibrationError(
            f'{path}: schema_version must be 2; legacy calibration files '
            'are intentionally rejected.'
        )

    extrinsic = _require_mapping(document, 'extrinsic', 'calibration')
    valid = extrinsic.get('valid')
    if not isinstance(valid, bool):
        raise CalibrationError('extrinsic.valid must be true or false.')
    if require_valid and not valid:
        raise CalibrationError(
            'The LiDAR-camera transform is marked as a placeholder. Install '
            'a solved transform and set extrinsic.valid=true first.'
        )
    source_frame = _require_string(
        extrinsic, 'source_frame', 'extrinsic'
    )
    target_frame = _require_string(
        extrinsic, 'target_frame', 'extrinsic'
    )
    if source_frame == target_frame:
        raise CalibrationError(
            'Extrinsic source and target frames must differ.'
        )
    convention = _require_string(extrinsic, 'convention', 'extrinsic')
    expected = 'p_target = T_target_source * p_source'
    if convention != expected:
        raise CalibrationError(
            f'extrinsic.convention must be exactly {expected!r}.'
        )
    if extrinsic.get('translation_unit') != 'm':
        raise CalibrationError('extrinsic.translation_unit must be "m".')
    matrix = validate_rigid_transform(extrinsic.get('matrix'))
    method = _require_string(extrinsic, 'method', 'extrinsic')
    if valid and method == 'placeholder':
        raise CalibrationError(
            'Extrinsic is marked valid but method is still "placeholder".'
        )

    camera_reference = _require_mapping(document, 'camera', 'calibration')
    camera_file = _require_string(
        camera_reference, 'config_file', 'calibration.camera'
    )
    camera_path = Path(camera_file).expanduser()
    if not camera_path.is_absolute():
        camera_path = path.parent / camera_path
    camera_model = load_camera_model(camera_path)

    return CalibrationBundle(
        path=path,
        raw=document,
        camera=camera_model,
        extrinsic=ExtrinsicCalibration(
            source_frame=source_frame,
            target_frame=target_frame,
            matrix=matrix,
            valid=valid,
            method=method,
        ),
    )


def load_session_pairs(session_path):
    """Read and strictly validate all pairs in a capture manifest."""
    session = Path(session_path).expanduser().resolve()
    manifest_path = session / 'manifest.jsonl'
    if not manifest_path.is_file():
        raise CalibrationError(f'Missing capture manifest: {manifest_path}')

    pairs = []
    seen_indices = set()
    try:
        with manifest_path.open('r', encoding='utf-8') as manifest:
            for line_number, line in enumerate(manifest, start=1):
                if not line.strip():
                    continue
                record = json.loads(line)
                if not isinstance(record, dict):
                    raise CalibrationError(
                        f'{manifest_path}:{line_number} is not a JSON object.'
                    )
                index = record.get('index')
                if isinstance(index, bool) or not isinstance(index, int):
                    raise CalibrationError(
                        f'{manifest_path}:{line_number} has no integer index.'
                    )
                if index in seen_indices:
                    raise CalibrationError(
                        f'{manifest_path} repeats pair index {index}.'
                    )
                seen_indices.add(index)
                image = session / str(record.get('image_file', ''))
                cloud = session / str(record.get('pointcloud_file', ''))
                if not image.is_file() or not cloud.is_file():
                    raise CalibrationError(
                        f'Pair {index} references a missing image or cloud.'
                    )
                copy = dict(record)
                copy['image_path'] = image.resolve()
                copy['pointcloud_path'] = cloud.resolve()
                pairs.append(copy)
    except (OSError, json.JSONDecodeError) as error:
        raise CalibrationError(
            f'Cannot parse capture manifest {manifest_path}: {error}'
        ) from error

    if not pairs:
        raise CalibrationError(f'Capture manifest is empty: {manifest_path}')
    pairs.sort(key=lambda item: item['index'])
    return session, pairs


def _pcd_numpy_type(type_name, size):
    """Map one PCD scalar type to a little-endian NumPy type."""
    types = {
        ('F', 4): '<f4',
        ('F', 8): '<f8',
        ('I', 1): '<i1',
        ('I', 2): '<i2',
        ('I', 4): '<i4',
        ('I', 8): '<i8',
        ('U', 1): '<u1',
        ('U', 2): '<u2',
        ('U', 4): '<u4',
        ('U', 8): '<u8',
    }
    try:
        return types[(type_name.upper(), size)]
    except KeyError as error:
        raise CalibrationError(
            f'Unsupported PCD scalar type {type_name}{size}.'
        ) from error


def read_pcd(path, xyz_only=False):
    """Read uncompressed binary or ASCII PCD data without Open3D."""
    path = Path(path)
    header = {}
    try:
        with path.open('rb') as stream:
            while True:
                line = stream.readline()
                if not line:
                    raise CalibrationError(f'{path} has no PCD DATA header.')
                decoded = line.decode('ascii').strip()
                if not decoded or decoded.startswith('#'):
                    continue
                key, *values = decoded.split()
                header[key.upper()] = values
                if key.upper() == 'DATA':
                    payload = stream.read()
                    break
    except (OSError, UnicodeDecodeError) as error:
        raise CalibrationError(f'Cannot read PCD {path}: {error}') from error

    fields = header.get('FIELDS') or header.get('FIELD')
    if not fields or any(name not in fields for name in ('x', 'y', 'z')):
        raise CalibrationError(f'{path} has no x/y/z PCD fields.')
    sizes = [int(value) for value in header.get('SIZE', [])]
    types = header.get('TYPE', [])
    counts = [int(value) for value in header.get('COUNT', ['1'] * len(fields))]
    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise CalibrationError(f'{path} has inconsistent PCD field metadata.')
    point_count_values = header.get('POINTS')
    if not point_count_values:
        raise CalibrationError(f'{path} has no PCD POINTS value.')
    point_count = int(point_count_values[0])

    dtype_fields = []
    for name, size, type_name, count in zip(fields, sizes, types, counts):
        scalar_type = _pcd_numpy_type(type_name, size)
        dtype_fields.append(
            (name, scalar_type) if count == 1 else (name, scalar_type, count)
        )
    dtype = np.dtype(dtype_fields)
    data_kind = header['DATA'][0].lower()
    if data_kind == 'binary':
        expected = point_count * dtype.itemsize
        if len(payload) < expected:
            raise CalibrationError(
                f'{path} has {len(payload)} data bytes; expected {expected}.'
            )
        points = np.frombuffer(payload[:expected], dtype=dtype).copy()
    elif data_kind == 'ascii':
        flat = np.fromstring(payload.decode('ascii'), sep=' ')
        scalar_count = sum(counts)
        if flat.size != point_count * scalar_count:
            raise CalibrationError(f'{path} has malformed ASCII PCD data.')
        rows = flat.reshape(point_count, scalar_count)
        points = np.empty(point_count, dtype=dtype)
        offset = 0
        for name, count in zip(fields, counts):
            values = rows[:, offset:offset + count]
            points[name] = values[:, 0] if count == 1 else values
            offset += count
    else:
        raise CalibrationError(
            f'{path}: PCD DATA {data_kind!r} is not supported.'
        )

    if not xyz_only:
        return header, points
    xyz = np.column_stack((points['x'], points['y'], points['z']))
    xyz = np.asarray(xyz, dtype=np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]


def project_points(points, transform, camera):
    """Project source-frame XYZ onto a raw distorted camera image."""
    xyz = np.asarray(points, dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise CalibrationError('Point array must have shape (N, 3).')
    transform = validate_rigid_transform(transform)
    camera_xyz = xyz @ transform[:3, :3].T + transform[:3, 3]
    forward = camera_xyz[:, 2] > 1e-6
    camera_xyz = camera_xyz[forward]
    if not camera_xyz.size:
        return np.empty((0, 2)), np.empty(0), forward
    pixels, _ = cv2.projectPoints(
        camera_xyz,
        np.zeros(3),
        np.zeros(3),
        camera.matrix,
        camera.distortion,
    )
    return pixels.reshape(-1, 2), camera_xyz[:, 2], forward


def invert_transform(transform):
    """Invert one validated rigid transform."""
    transform = validate_rigid_transform(transform)
    inverse = np.eye(4)
    inverse[:3, :3] = transform[:3, :3].T
    inverse[:3, 3] = -inverse[:3, :3] @ transform[:3, 3]
    return inverse


def rotation_angle_degrees(rotation):
    """Return the shortest rotation angle represented by a matrix."""
    trace_value = float(np.trace(rotation))
    cosine = np.clip((trace_value - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


def transform_delta(original, candidate):
    """Measure global translation and rotation change between transforms."""
    delta = validate_rigid_transform(candidate) @ invert_transform(original)
    return {
        'translation_m': float(np.linalg.norm(delta[:3, 3])),
        'rotation_deg': rotation_angle_degrees(delta[:3, :3]),
    }


def matrix_to_quaternion_xyzw(rotation):
    """Convert a valid rotation matrix to a normalized XYZW quaternion."""
    rotation = _finite_array(rotation, (3, 3), 'rotation')
    vector, _ = cv2.Rodrigues(rotation)
    angle = float(np.linalg.norm(vector))
    if angle < 1e-15:
        return np.array([0.0, 0.0, 0.0, 1.0])
    axis = vector.reshape(3) / angle
    half = angle / 2.0
    quaternion = np.concatenate((axis * np.sin(half), [np.cos(half)]))
    return quaternion / np.linalg.norm(quaternion)
