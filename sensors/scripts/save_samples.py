"""Save auditable synchronized image and LiDAR pairs for calibration."""

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import time
import uuid

from ament_index_python.packages import get_package_share_directory
import cv2
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger
import yaml


SUPPORTED_CAPTURE_MODES = {'manual', 'interval'}
SUPPORTED_IMAGE_ENCODINGS = {
    'bgr8': 3,
    'rgb8': 3,
    'mono8': 1,
    'bgra8': 4,
    'rgba8': 4,
}
POINT_VALUE_FIELDS = ('reflectivity', 'intensity', 'signal')
SESSION_NAME_PATTERN = re.compile(r'[A-Za-z0-9][A-Za-z0-9_.-]*')


def stamp_to_ns(stamp):
    """Convert a ROS time message to integer nanoseconds."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def image_message_to_bgr(message):
    """Convert one supported ROS Image into contiguous BGR pixels."""
    encoding = message.encoding.lower()
    if encoding not in SUPPORTED_IMAGE_ENCODINGS:
        supported = ', '.join(sorted(SUPPORTED_IMAGE_ENCODINGS))
        raise ValueError(
            f'Unsupported image encoding {message.encoding!r}; '
            f'supported encodings: {supported}.'
        )

    height = int(message.height)
    width = int(message.width)
    channels = SUPPORTED_IMAGE_ENCODINGS[encoding]
    minimum_step = width * channels
    step = int(message.step)
    if height <= 0 or width <= 0:
        raise ValueError('Image width and height must be positive.')
    if step < minimum_step:
        raise ValueError(
            f'Image step {step} is smaller than {minimum_step} bytes.'
        )

    raw = np.frombuffer(message.data, dtype=np.uint8)
    required_size = height * step
    if raw.size < required_size:
        raise ValueError(
            f'Image has {raw.size} bytes but its dimensions require '
            f'{required_size}.'
        )
    pixels = raw[:required_size].reshape(height, step)
    pixels = pixels[:, :minimum_step].reshape(height, width, channels)

    if encoding == 'bgr8':
        bgr = pixels
    elif encoding == 'rgb8':
        bgr = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
    elif encoding == 'mono8':
        bgr = cv2.cvtColor(pixels, cv2.COLOR_GRAY2BGR)
    elif encoding == 'bgra8':
        bgr = cv2.cvtColor(pixels, cv2.COLOR_BGRA2BGR)
    else:
        bgr = cv2.cvtColor(pixels, cv2.COLOR_RGBA2BGR)
    return np.ascontiguousarray(bgr)


def pointcloud_message_to_array(message):
    """Extract finite XYZ and reflectivity-like values without Python loops."""
    fields = {field.name: field for field in message.fields}
    missing_xyz = [name for name in ('x', 'y', 'z') if name not in fields]
    if missing_xyz:
        raise ValueError(
            'PointCloud2 is missing required field(s): '
            + ', '.join(missing_xyz)
        )

    value_field = next(
        (name for name in POINT_VALUE_FIELDS if name in fields),
        None,
    )
    if value_field is None:
        supported = ', '.join(POINT_VALUE_FIELDS)
        raise ValueError(
            f'PointCloud2 needs one value field: {supported}.'
        )

    selected_fields = ['x', 'y', 'z', value_field]
    structured = point_cloud2.read_points(
        message,
        field_names=selected_fields,
        skip_nans=False,
    )
    columns = [
        np.asarray(structured[name], dtype=np.float32)
        for name in selected_fields
    ]
    if any(column.ndim != 1 for column in columns):
        raise ValueError(
            'Calibration point fields must each contain one value.'
        )

    points = np.column_stack(columns)
    valid = np.isfinite(points).all(axis=1)
    valid &= np.square(points[:, :3]).sum(axis=1) > 0.0
    return np.ascontiguousarray(points[valid]), value_field


def make_binary_pcd(points, value_field):
    """Encode float32 XYZ plus one scalar channel as binary PCD 0.7."""
    points = np.asarray(points)
    if points.ndim != 2 or points.shape[1] != 4:
        raise ValueError('PCD input must have shape (N, 4).')
    if value_field not in POINT_VALUE_FIELDS:
        raise ValueError(f'Unsupported PCD scalar field {value_field!r}.')

    little_endian_points = np.ascontiguousarray(points, dtype='<f4')
    point_count = little_endian_points.shape[0]
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\n'
        'VERSION 0.7\n'
        f'FIELDS x y z {value_field}\n'
        'SIZE 4 4 4 4\n'
        'TYPE F F F F\n'
        'COUNT 1 1 1 1\n'
        f'WIDTH {point_count}\n'
        'HEIGHT 1\n'
        'VIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {point_count}\n'
        'DATA binary\n'
    ).encode('ascii')
    return header + little_endian_points.tobytes(order='C')


def validate_save_config(config):
    """Validate the saver configuration without silently coercing values."""
    for section_name in ('ROS', 'Sync', 'Capture'):
        if not isinstance(config.get(section_name), dict):
            raise ValueError(
                f'Missing save configuration section {section_name!r}.'
            )

    ros = config['ROS']
    for key in (
        'image_topic_name',
        'pointcloud_topic_name',
        'save_service_name',
    ):
        if not isinstance(ros.get(key), str) or not ros[key]:
            raise ValueError(f'ROS.{key} must be a non-empty string.')

    sync = config['Sync']
    _require_number(sync, 'threshold_sec', minimum=0.000001, maximum=1.0)
    _require_integer(sync, 'queue_size', minimum=2)

    capture = config['Capture']
    mode = capture.get('mode')
    if mode not in SUPPORTED_CAPTURE_MODES:
        supported = ', '.join(sorted(SUPPORTED_CAPTURE_MODES))
        raise ValueError(
            f'Capture.mode must be one of: {supported}.'
        )
    _require_integer(capture, 'total_samples', minimum=1)
    _require_number(capture, 'min_interval_sec', minimum=0.0)
    _require_number(capture, 'max_pair_age_sec', minimum=0.001)
    _require_integer(capture, 'min_point_count', minimum=1)
    _require_number(capture, 'diagnostics_interval_sec', minimum=0.1)

    output_root = capture.get('output_root')
    if not isinstance(output_root, str) or not output_root:
        raise ValueError('Capture.output_root must be a non-empty path.')
    session_name = capture.get('session_name')
    if session_name != 'auto' and (
        not isinstance(session_name, str)
        or SESSION_NAME_PATTERN.fullmatch(session_name) is None
    ):
        raise ValueError(
            'Capture.session_name must be "auto" or a simple file name.'
        )
    return config


def _require_number(mapping, key, minimum=None, maximum=None):
    """Require one finite integer or floating-point configuration value."""
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{key} must be a finite number.')
    value = float(value)
    if not np.isfinite(value):
        raise ValueError(f'{key} must be a finite number.')
    if minimum is not None and value < minimum:
        raise ValueError(f'{key} must be at least {minimum}.')
    if maximum is not None and value > maximum:
        raise ValueError(f'{key} must be at most {maximum}.')


def _require_integer(mapping, key, minimum=None):
    """Require one non-boolean integer configuration value."""
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f'{key} must be an integer.')
    if minimum is not None and value < minimum:
        raise ValueError(f'{key} must be at least {minimum}.')


class CaptureSession:
    """Own one non-overwriting calibration dataset directory."""

    def __init__(self, output_root, session_name, session_metadata):
        """Create a unique session and write its immutable metadata."""
        if session_name == 'auto':
            session_name = datetime.now(timezone.utc).strftime(
                'session_%Y%m%dT%H%M%S.%fZ'
            )

        root = Path(output_root).expanduser()
        if not root.is_absolute():
            root = Path.cwd() / root
        self.root = root.resolve()
        self.session_name = session_name
        self.directory = self.root / session_name
        self.images_directory = self.directory / 'images'
        self.pointclouds_directory = self.directory / 'pcds'
        self.manifest_path = self.directory / 'manifest.jsonl'
        self.summary_path = self.directory / 'summary.json'

        self.directory.mkdir(parents=True, exist_ok=False)
        self.images_directory.mkdir()
        self.pointclouds_directory.mkdir()
        self._write_json_atomic(
            self.directory / 'session.json', session_metadata
        )

    def write_pair(self, index, image_bgr, points, value_field, metadata):
        """Atomically write one image/cloud pair, then append its manifest."""
        image_name = f'img_{index:04d}.png'
        pointcloud_name = f'pc_{index:04d}.pcd'
        image_path = self.images_directory / image_name
        pointcloud_path = self.pointclouds_directory / pointcloud_name
        if image_path.exists() or pointcloud_path.exists():
            raise FileExistsError(
                f'Capture pair {index} already exists in {self.directory}.'
            )

        encoded, png = cv2.imencode(
            '.png', image_bgr, [cv2.IMWRITE_PNG_COMPRESSION, 3]
        )
        if not encoded:
            raise RuntimeError(
                'OpenCV failed to encode the calibration image.'
            )
        pcd = make_binary_pcd(points, value_field)

        temporary_paths = []
        finalized = []
        try:
            image_temporary = self._write_temporary(
                self.images_directory, image_name, png.tobytes()
            )
            temporary_paths.append(image_temporary)
            pointcloud_temporary = self._write_temporary(
                self.pointclouds_directory, pointcloud_name, pcd
            )
            temporary_paths.append(pointcloud_temporary)
            os.replace(image_temporary, image_path)
            finalized.append(image_path)
            os.replace(pointcloud_temporary, pointcloud_path)
            finalized.append(pointcloud_path)
            record = dict(metadata)
            record.update({
                'index': index,
                'image_file': str(image_path.relative_to(self.directory)),
                'pointcloud_file': str(
                    pointcloud_path.relative_to(self.directory)
                ),
            })
            self._append_manifest(record)
        except Exception:
            for path in temporary_paths:
                Path(path).unlink(missing_ok=True)
            for path in finalized:
                path.unlink(missing_ok=True)
            raise
        return image_path, pointcloud_path

    def write_summary(self, statistics):
        """Write the latest capture statistics atomically."""
        summary = dict(statistics)
        summary['updated_at_utc'] = datetime.now(timezone.utc).isoformat()
        self._write_json_atomic(self.summary_path, summary)

    def _append_manifest(self, record):
        """Durably append one completed pair record."""
        line = json.dumps(record, sort_keys=True) + '\n'
        with self.manifest_path.open('a', encoding='utf-8') as manifest:
            manifest.write(line)
            manifest.flush()
            os.fsync(manifest.fileno())

    def _write_json_atomic(self, path, value):
        """Atomically replace one JSON document."""
        content = (
            json.dumps(value, indent=2, sort_keys=True) + '\n'
        ).encode('utf-8')
        temporary = self._write_temporary(path.parent, path.name, content)
        try:
            os.replace(temporary, path)
        finally:
            Path(temporary).unlink(missing_ok=True)

    @staticmethod
    def _write_temporary(directory, final_name, content):
        """Write and fsync unique temporary bytes beside their destination."""
        temporary = directory / (
            f'.{final_name}.tmp-{os.getpid()}-{uuid.uuid4().hex}'
        )
        with temporary.open('xb') as output:
            output.write(content)
            output.flush()
            os.fsync(output.fileno())
        return temporary


class SensorSyncSaverNode(Node):
    """Synchronize live messages and save selected calibration pairs."""

    def __init__(self):
        """Load configuration, create the session, and subscribe to sensors."""
        super().__init__('sensor_sync_saver')
        self.declare_parameter('config_file', '')
        self.config_path = self._resolve_config_path()
        self.config = self._load_config(self.config_path)
        self.ros_config = self.config['ROS']
        self.sync_config = self.config['Sync']
        self.capture_config = self.config['Capture']
        self.capture_mode = self.capture_config['mode']
        self.total_samples = self.capture_config['total_samples']
        self.threshold_ns = round(
            self.sync_config['threshold_sec'] * 1_000_000_000
        )
        self.min_interval_ns = round(
            self.capture_config['min_interval_sec'] * 1_000_000_000
        )
        self.max_pair_age_ns = round(
            self.capture_config['max_pair_age_sec'] * 1_000_000_000
        )

        self.save_count = 0
        self.matched_count = 0
        self.rejected_count = 0
        self._latest_pair = None
        self._latest_delta_ns = None
        self._last_saved_measurement_ns = None
        self._last_saved_stamp_pair = None
        self._expected_image_frame = None
        self._expected_lidar_frame = None
        self._complete_logged = False
        self._shutdown_started = False

        use_sim_time = self.get_parameter(
            'use_sim_time'
        ).get_parameter_value().bool_value
        session_metadata = {
            'schema_version': 1,
            'created_at_utc': datetime.now(timezone.utc).isoformat(),
            'config_file': str(self.config_path),
            'configuration': self.config,
            'requested_sensor_configurations': (
                self._load_requested_sensor_configurations()
            ),
            'use_sim_time': use_sim_time,
        }
        self.session = CaptureSession(
            self.capture_config['output_root'],
            self.capture_config['session_name'],
            session_metadata,
        )

        qos = QoSProfile(
            depth=self.sync_config['queue_size'],
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._lidar_subscriber = Subscriber(
            self,
            PointCloud2,
            self.ros_config['pointcloud_topic_name'],
            qos_profile=qos,
        )
        self._image_subscriber = Subscriber(
            self,
            Image,
            self.ros_config['image_topic_name'],
            qos_profile=qos,
        )
        self._synchronizer = ApproximateTimeSynchronizer(
            [self._lidar_subscriber, self._image_subscriber],
            queue_size=self.sync_config['queue_size'],
            slop=self.sync_config['threshold_sec'],
            allow_headerless=False,
        )
        self._synchronizer.registerCallback(self._synchronized_callback)

        self._save_service = self.create_service(
            Trigger,
            self.ros_config['save_service_name'],
            self._save_service_callback,
        )
        self._diagnostics_timer = self.create_timer(
            self.capture_config['diagnostics_interval_sec'],
            self._log_diagnostics,
        )
        self._log_setup(use_sim_time)

    def _resolve_config_path(self):
        """Resolve an optional ROS override or the installed default YAML."""
        override = self.get_parameter(
            'config_file'
        ).get_parameter_value().string_value
        if override:
            return Path(override).expanduser().resolve()
        package_directory = get_package_share_directory('sensors')
        return Path(package_directory) / 'config' / 'save_sample.yaml'

    @staticmethod
    def _load_config(path):
        """Load and validate one calibration capture YAML document."""
        with path.open('r', encoding='utf-8') as config_file:
            config = yaml.safe_load(config_file)
        if not isinstance(config, dict):
            raise ValueError('Save configuration must be a YAML mapping.')
        return validate_save_config(config)

    def _load_requested_sensor_configurations(self):
        """Snapshot requested publisher YAML without claiming active state."""
        config_directory = self.config_path.parent
        snapshots = {}
        for file_name in ('camera_config.yaml', 'ouster_config.yaml'):
            path = config_directory / file_name
            try:
                with path.open('r', encoding='utf-8') as config_file:
                    snapshots[file_name] = yaml.safe_load(config_file)
            except (OSError, yaml.YAMLError) as error:
                snapshots[file_name] = {'snapshot_error': str(error)}
        return snapshots

    def _log_setup(self, use_sim_time):
        """Describe the exact capture policy and output location."""
        self.get_logger().info(
            f'Calibration capture session: {self.session.directory}'
        )
        self.get_logger().info(
            f'mode={self.capture_mode}, total={self.total_samples}, '
            f'slop={self.sync_config["threshold_sec"]:.6f} s, '
            f'min_interval={self.capture_config["min_interval_sec"]:.3f} s, '
            f'use_sim_time={use_sim_time}'
        )
        self.get_logger().info(
            f'image={self.ros_config["image_topic_name"]}, '
            f'cloud={self.ros_config["pointcloud_topic_name"]}'
        )
        if self.capture_mode == 'manual':
            self.get_logger().info(
                'Hold the calibration target still, then call: ros2 service '
                f'call {self.ros_config["save_service_name"]} '
                'std_srvs/srv/Trigger "{}"'
            )
        else:
            self.get_logger().warning(
                'Interval capture is enabled. Use it only when the target '
                'pose schedule is controlled; manual mode is safer for '
                'calibration pose diversity.'
            )

    def _synchronized_callback(self, lidar_message, image_message):
        """Cache or save one approximately synchronized message pair."""
        lidar_stamp_ns = stamp_to_ns(lidar_message.header.stamp)
        image_stamp_ns = stamp_to_ns(image_message.header.stamp)
        if lidar_stamp_ns <= 0 or image_stamp_ns <= 0:
            self.rejected_count += 1
            self.get_logger().warning(
                'Rejected synchronized pair with a zero or negative stamp.',
                throttle_duration_sec=5.0,
            )
            return

        delta_ns = image_stamp_ns - lidar_stamp_ns
        if abs(delta_ns) > self.threshold_ns:
            self.rejected_count += 1
            self.get_logger().warning(
                f'Rejected pair outside configured slop: '
                f'{delta_ns / 1e6:.3f} ms.',
                throttle_duration_sec=5.0,
            )
            return

        self.matched_count += 1
        self._latest_delta_ns = delta_ns
        self._latest_pair = (
            lidar_message,
            image_message,
            time.monotonic_ns(),
        )
        if self.capture_mode == 'interval':
            self._try_save_pair(lidar_message, image_message)

    def _save_service_callback(self, request, response):
        """Save the latest fresh synchronized pair after a manual request."""
        del request
        if self.capture_mode != 'manual':
            response.success = False
            response.message = 'Capture.mode is not manual.'
            return response
        if self.save_count >= self.total_samples:
            response.success = False
            response.message = (
                'Requested calibration sample count is complete.'
            )
            return response
        if self._latest_pair is None:
            response.success = False
            response.message = 'No synchronized pair has arrived yet.'
            return response

        lidar_message, image_message, received_ns = self._latest_pair
        age_ns = time.monotonic_ns() - received_ns
        if age_ns > self.max_pair_age_ns:
            response.success = False
            response.message = (
                f'Latest synchronized pair is {age_ns / 1e3 / 1e3:.1f} ms '
                'old; check both sensor topics.'
            )
            return response

        success, message = self._try_save_pair(
            lidar_message, image_message
        )
        response.success = success
        response.message = message
        return response

    def _try_save_pair(self, lidar_message, image_message):
        """Apply capture gates and persist one complete calibration pair."""
        if self.save_count >= self.total_samples:
            self._log_complete_once()
            return False, 'Requested calibration sample count is complete.'

        lidar_stamp_ns = stamp_to_ns(lidar_message.header.stamp)
        image_stamp_ns = stamp_to_ns(image_message.header.stamp)
        stamp_pair = (lidar_stamp_ns, image_stamp_ns)
        if stamp_pair == self._last_saved_stamp_pair:
            return False, 'This synchronized pair was already saved.'

        measurement_ns = max(lidar_stamp_ns, image_stamp_ns)
        if (
            self._last_saved_measurement_ns is not None
            and measurement_ns - self._last_saved_measurement_ns
            < self.min_interval_ns
        ):
            remaining_ns = (
                self.min_interval_ns
                - (measurement_ns - self._last_saved_measurement_ns)
            )
            return False, (
                f'Wait {remaining_ns / 1e9:.3f} s for a newer pair.'
            )

        image_frame = image_message.header.frame_id
        lidar_frame = lidar_message.header.frame_id
        if (
            self._expected_image_frame is not None
            and image_frame != self._expected_image_frame
        ):
            self.rejected_count += 1
            return False, (
                f'Image frame changed from {self._expected_image_frame!r} '
                f'to {image_frame!r}.'
            )
        if (
            self._expected_lidar_frame is not None
            and lidar_frame != self._expected_lidar_frame
        ):
            self.rejected_count += 1
            return False, (
                f'LiDAR frame changed from {self._expected_lidar_frame!r} '
                f'to {lidar_frame!r}.'
            )

        try:
            image_bgr = image_message_to_bgr(image_message)
            points, value_field = pointcloud_message_to_array(lidar_message)
            if points.shape[0] < self.capture_config['min_point_count']:
                self.rejected_count += 1
                return False, (
                    f'Cloud contains {points.shape[0]} valid points; '
                    f'{self.capture_config["min_point_count"]} required.'
                )

            delta_ns = image_stamp_ns - lidar_stamp_ns
            metadata = {
                'saved_at_utc': datetime.now(timezone.utc).isoformat(),
                'image_stamp_ns': image_stamp_ns,
                'lidar_stamp_ns': lidar_stamp_ns,
                'image_minus_lidar_ns': delta_ns,
                'absolute_stamp_delta_ns': abs(delta_ns),
                'image_frame_id': image_frame,
                'lidar_frame_id': lidar_frame,
                'image_encoding': image_message.encoding,
                'image_width': int(image_message.width),
                'image_height': int(image_message.height),
                'point_count': int(points.shape[0]),
                'point_value_field': value_field,
                'point_step': int(lidar_message.point_step),
                'pointcloud_is_bigendian': bool(
                    lidar_message.is_bigendian
                ),
                'pointcloud_is_dense': bool(lidar_message.is_dense),
                'pointcloud_fields': [
                    {
                        'name': field.name,
                        'offset': int(field.offset),
                        'datatype': int(field.datatype),
                        'count': int(field.count),
                    }
                    for field in lidar_message.fields
                ],
            }
            image_path, pointcloud_path = self.session.write_pair(
                self.save_count,
                image_bgr,
                points,
                value_field,
                metadata,
            )
        except Exception as error:
            self.rejected_count += 1
            self.get_logger().error(
                f'Failed to save calibration pair: {error}'
            )
            return False, f'Failed to save calibration pair: {error}'

        self._expected_image_frame = image_frame
        self._expected_lidar_frame = lidar_frame
        self._last_saved_measurement_ns = measurement_ns
        self._last_saved_stamp_pair = stamp_pair
        self.save_count += 1
        message = (
            f'Saved pair {self.save_count}/{self.total_samples}: '
            f'{image_path.name}, {pointcloud_path.name}; '
            f'delta={delta_ns / 1e6:.3f} ms, points={points.shape[0]}.'
        )
        self.get_logger().info(message)
        self._write_summary()
        if self.save_count >= self.total_samples:
            self._log_complete_once()
        return True, message

    def _log_diagnostics(self):
        """Periodically report matching and capture progress."""
        delta_text = (
            'none'
            if self._latest_delta_ns is None
            else f'{self._latest_delta_ns / 1e6:.3f} ms'
        )
        self.get_logger().info(
            f'Calibration capture: matched={self.matched_count}, '
            f'saved={self.save_count}/{self.total_samples}, '
            f'rejected={self.rejected_count}, latest_delta={delta_text}'
        )

    def _log_complete_once(self):
        """Announce completion without shutting ROS down from a callback."""
        if self._complete_logged:
            return
        self._complete_logged = True
        self.get_logger().info(
            f'Calibration capture complete: {self.session.directory}. '
            'Stop the node with Ctrl-C after checking the manifest.'
        )

    def _write_summary(self):
        """Persist current counters without changing pair data."""
        self.session.write_summary({
            'matched_pairs': self.matched_count,
            'saved_pairs': self.save_count,
            'rejected_pairs': self.rejected_count,
            'requested_pairs': self.total_samples,
            'complete': self.save_count >= self.total_samples,
        })

    def shutdown(self):
        """Write final capture statistics exactly once."""
        if self._shutdown_started:
            return
        self._shutdown_started = True
        try:
            self._write_summary()
        except Exception as error:
            self.get_logger().error(
                f'Failed to write final calibration summary: {error}'
            )


def main(args=None):
    """Run the synchronized calibration capture node."""
    rclpy.init(args=args)
    node = None
    try:
        node = SensorSyncSaverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
