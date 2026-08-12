"""Publish aligned Ouster XYZ and reflectivity as ROS PointCloud2 data."""

import json
import math
from numbers import Real
import os
import threading
from urllib.request import urlopen

from ament_index_python.packages import get_package_share_directory
import numpy as np
from ouster.sdk import core, sensor
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
import yaml


LIDAR_MODES = {
    '512x10': core.LidarMode.MODE_512x10,
    '512x20': core.LidarMode.MODE_512x20,
    '1024x10': core.LidarMode.MODE_1024x10,
    '1024x20': core.LidarMode.MODE_1024x20,
    '2048x10': core.LidarMode.MODE_2048x10,
}

TIMESTAMP_MODES = {
    'TIME_FROM_INTERNAL_OSC': core.TimestampMode.TIME_FROM_INTERNAL_OSC,
    'TIME_FROM_PTP_1588': core.TimestampMode.TIME_FROM_PTP_1588,
    'TIME_FROM_SYNC_PULSE_IN': core.TimestampMode.TIME_FROM_SYNC_PULSE_IN,
}

MULTIPURPOSE_IO_MODES = {
    'OFF': core.MultipurposeIOMode.MULTIPURPOSE_OFF,
    'INPUT_NMEA_UART': (
        core.MultipurposeIOMode.MULTIPURPOSE_INPUT_NMEA_UART
    ),
    'OUTPUT_FROM_INTERNAL_OSC': (
        core.MultipurposeIOMode.MULTIPURPOSE_OUTPUT_FROM_INTERNAL_OSC
    ),
    'OUTPUT_FROM_PTP_1588': (
        core.MultipurposeIOMode.MULTIPURPOSE_OUTPUT_FROM_PTP_1588
    ),
    'OUTPUT_FROM_SYNC_PULSE_IN': (
        core.MultipurposeIOMode.MULTIPURPOSE_OUTPUT_FROM_SYNC_PULSE_IN
    ),
}

ROS_STAMP_SOURCES = {'host_receipt', 'sensor_scan_midpoint'}

PTP_STATUS_PATH = '/api/v1/time/ptp'
PTP_HTTP_TIMEOUT_SEC = 2.0
PTP_POLL_INTERVAL_SEC = 2.0
PTP_MAX_MASTER_OFFSET_NS = 250_000

XYZI_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(
        name='reflectivity',
        offset=12,
        datatype=PointField.FLOAT32,
        count=1,
    ),
]


def _configured_enum(mapping, configured_value, setting_name):
    """Resolve one named setting and reject silent configuration fallbacks."""
    try:
        return mapping[configured_value]
    except KeyError as error:
        supported = ', '.join(sorted(mapping))
        raise ValueError(
            f'Unsupported {setting_name} {configured_value!r}; '
            f'supported values: {supported}.'
        ) from error


def _configured_bool(mapping, key, default, setting_name):
    """Read a YAML boolean without treating non-empty strings as true."""
    value = mapping.get(key, default)
    if not isinstance(value, bool):
        raise ValueError(f'{setting_name} must be true or false.')
    return value


def _configured_float(
    mapping,
    key,
    default,
    setting_name,
    minimum=None,
    maximum=None,
):
    """Read one finite numeric YAML value and enforce optional bounds."""
    value = mapping.get(key, default)
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f'{setting_name} must be a finite number.')
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f'{setting_name} must be a finite number.')
    if minimum is not None and value < minimum:
        raise ValueError(f'{setting_name} must be at least {minimum}.')
    if maximum is not None and value > maximum:
        raise ValueError(f'{setting_name} must be at most {maximum}.')
    return value


def evaluate_ptp_lock(status, max_master_offset_ns):
    """Return whether an Ouster PTP status response describes a good lock."""
    try:
        port_state = status['port_data_set']['port_state']
        gm_present = status['time_status_np']['gm_present']
        master_offset = status['time_status_np'].get('master_offset')
        if master_offset is None:
            master_offset = status['current_data_set']['offset_from_master']
        master_offset = float(master_offset)
    except (KeyError, TypeError, ValueError) as error:
        return False, f'incomplete PTP status: {error}'

    if port_state != 'SLAVE':
        return False, f'port_state={port_state!r}, expected \'SLAVE\''
    if gm_present is not True:
        return False, 'no PTP grandmaster is present'
    if abs(master_offset) > max_master_offset_ns:
        return False, (
            f'master offset {master_offset:.0f} ns exceeds '
            f'{max_master_offset_ns} ns'
        )
    return True, f'locked with master offset {master_offset:.0f} ns'


def evaluate_ptp_timebase(status, configured_offset_sec):
    """Validate the UTC/TAI conversion advertised by the grandmaster."""
    properties = status.get('time_properties_data_set')
    if not isinstance(properties, dict):
        return True, (
            'PTP time properties are unavailable; the converted timestamp '
            'will still be checked against ROS time'
        )

    ptp_timescale_value = properties.get('ptp_timescale')
    if ptp_timescale_value in (True, 1):
        ptp_timescale = True
    elif ptp_timescale_value in (False, 0):
        ptp_timescale = False
    else:
        return False, (
            'invalid or missing PTP time_properties_data_set.ptp_timescale'
        )

    if not ptp_timescale:
        if not math.isclose(configured_offset_sec, 0.0, abs_tol=1e-9):
            return False, (
                'grandmaster does not advertise the PTP/TAI timescale, so '
                'ptp_utc_tai_offset must be 0.0'
            )
        return True, 'grandmaster supplies a non-TAI timebase; no offset used'

    offset_valid_value = properties.get('current_utc_offset_valid')
    offset_valid = offset_valid_value in (True, 1)
    if not offset_valid:
        return True, (
            'grandmaster advertises PTP/TAI but its current UTC offset is '
            'not marked valid; the ROS-time plausibility check remains active'
        )

    try:
        current_utc_offset = float(properties['current_utc_offset'])
    except (KeyError, TypeError, ValueError):
        return False, 'grandmaster reports an invalid current UTC offset'
    if not math.isfinite(current_utc_offset):
        return False, 'grandmaster reports a non-finite current UTC offset'

    expected_offset = -current_utc_offset
    if not math.isclose(
        configured_offset_sec, expected_offset, abs_tol=1e-9
    ):
        return False, (
            f'grandmaster announces TAI-UTC={current_utc_offset:g} s; '
            f'ptp_utc_tai_offset must be {expected_offset:g} s, not '
            f'{configured_offset_sec:g} s'
        )
    return True, (
        f'grandmaster announces valid TAI-UTC={current_utc_offset:g} s; '
        f'applying {configured_offset_sec:g} s for ROS/UTC'
    )


def fetch_ptp_status(hostname):
    """Read the sensor PTP status through its documented HTTP endpoint."""
    url = f'http://{hostname}{PTP_STATUS_PATH}'
    with urlopen(url, timeout=PTP_HTTP_TIMEOUT_SEC) as response:
        status = json.load(response)
    if not isinstance(status, dict):
        raise ValueError('Ouster PTP status response is not a JSON object.')
    return status


def scan_midpoint_ns(column_timestamps, valid_columns):
    """Return the midpoint of the valid scan-column interval, or ``None``."""
    timestamps = np.asarray(column_timestamps)
    columns = np.asarray(valid_columns, dtype=bool)
    if timestamps.ndim != 1 or columns.shape != timestamps.shape:
        raise ValueError(
            'Column timestamps and the valid-column mask must be 1-D arrays '
            'with the same shape.'
        )

    usable_timestamps = timestamps[columns & (timestamps > 0)]
    if usable_timestamps.size == 0:
        return None

    first_timestamp = int(usable_timestamps.min())
    last_timestamp = int(usable_timestamps.max())
    return first_timestamp + (last_timestamp - first_timestamp) // 2


def valid_column_ratio(valid_columns):
    """Return the fraction of columns marked valid by the sensor."""
    columns = np.asarray(valid_columns, dtype=bool)
    if columns.ndim != 1 or columns.size == 0:
        raise ValueError(
            'The valid-column mask must be a non-empty 1-D array.'
        )
    return float(np.count_nonzero(columns)) / columns.size


def ptp_timestamp_to_ros_ns(
    ptp_timestamp_ns,
    utc_tai_offset_ns,
    receipt_ros_ns,
    max_difference_ns,
):
    """Convert PTP/TAI nanoseconds and reject an implausible ROS epoch."""
    converted_ns = int(ptp_timestamp_ns) + int(utc_tai_offset_ns)
    if converted_ns < 0:
        return None, 'UTC/TAI conversion produced a negative timestamp'

    difference_ns = abs(converted_ns - int(receipt_ros_ns))
    if difference_ns > int(max_difference_ns):
        return None, (
            f'converted PTP time differs from ROS time by '
            f'{difference_ns / 1e9:.6f} s'
        )
    return converted_ns, None


class OusterLidarPublisher(Node):
    """Configure an Ouster and publish filtered, aligned XYZI clouds."""

    def __init__(self):
        """Load configuration and construct a non-blocking ROS node."""
        super().__init__('ouster_lidar_publisher')

        self._stop_event = threading.Event()
        self._ptp_lock_event = threading.Event()
        self._stream_thread = None
        self._ptp_monitor_thread = None
        self._stream = None
        self._stream_lock = threading.Lock()
        self._fatal_error = None
        self._shutdown_started = False

        self.config_file = self._load_config()
        self.sensor_config = self.config_file['sensor']
        self.lidar_config = self.config_file['lidar']
        self.hostname = self.sensor_config['host_name']

        self._warn_if_using_simulated_time()
        self._configure_timestamp_policy()
        self._configure_scan_policy()
        self.config = self._make_sensor_config()
        self.persist_config = _configured_bool(
            self.sensor_config,
            'persist_config',
            False,
            'sensor.persist_config',
        )
        self._configure_sensor()

        self.publisher = self.create_publisher(
            PointCloud2,
            self.config_file['ROS']['topic_name'],
            self._make_qos_profile(),
        )
        self._health_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._health_timer = self.create_timer(
            0.2,
            self._check_stream_health,
            clock=self._health_clock,
        )

    def start_streaming(self):
        """Start sensor I/O workers after node construction has completed."""
        if self._stream_thread is not None:
            raise RuntimeError('Ouster streaming has already been started.')
        if self._stop_event.is_set():
            raise RuntimeError('Cannot start Ouster streaming after shutdown.')

        if self._ptp_lock_required:
            self._ptp_monitor_thread = threading.Thread(
                target=self._ptp_monitor_loop,
                name='ouster_ptp_monitor',
                daemon=True,
            )
            self._ptp_monitor_thread.start()
        else:
            self._ptp_lock_event.set()

        self._stream_thread = threading.Thread(
            target=self._stream_worker,
            name='ouster_stream',
            daemon=True,
        )
        self._stream_thread.start()

    @staticmethod
    def _load_config():
        """Load the installed Ouster YAML configuration."""
        package_directory = get_package_share_directory('sensors')
        config_path = os.path.join(
            package_directory, 'config', 'ouster_config.yaml'
        )
        with open(config_path, 'r', encoding='utf-8') as config_file:
            return yaml.safe_load(config_file)

    def _warn_if_using_simulated_time(self):
        """Warn if a live sensor is using playback/simulation time."""
        use_sim_time = self.get_parameter(
            'use_sim_time'
        ).get_parameter_value().bool_value
        self.get_logger().info(f'use_sim_time is set to: {use_sim_time}')
        if use_sim_time:
            self.get_logger().warning(
                'A live Ouster is using simulated time. host_receipt stamps '
                'will follow /clock and cannot synchronize live sensors.'
            )

    def _configure_timestamp_policy(self):
        """Validate the distinct sensor-clock and ROS-stamp settings."""
        self.timestamp_mode_name = self.lidar_config['timestamp_mode']
        self.timestamp_mode = _configured_enum(
            TIMESTAMP_MODES,
            self.timestamp_mode_name,
            'lidar.timestamp_mode',
        )

        self.ros_stamp_source = self.lidar_config['ros_stamp_source']
        if self.ros_stamp_source not in ROS_STAMP_SOURCES:
            supported = ', '.join(sorted(ROS_STAMP_SOURCES))
            raise ValueError(
                f'Unsupported lidar.ros_stamp_source '
                f'{self.ros_stamp_source!r}; supported values: {supported}.'
            )

        self.require_ptp_lock = _configured_bool(
            self.lidar_config,
            'require_ptp_lock',
            True,
            'lidar.require_ptp_lock',
        )
        self.ptp_utc_tai_offset_sec = _configured_float(
            self.lidar_config,
            'ptp_utc_tai_offset',
            -37.0,
            'lidar.ptp_utc_tai_offset',
            minimum=-100.0,
            maximum=100.0,
        )
        self._ptp_utc_tai_offset_ns = round(
            self.ptp_utc_tai_offset_sec * 1_000_000_000
        )
        self.max_ptp_ros_time_difference_sec = _configured_float(
            self.lidar_config,
            'max_ptp_ros_time_difference_sec',
            1.0,
            'lidar.max_ptp_ros_time_difference_sec',
            minimum=0.001,
            maximum=60.0,
        )
        self._max_ptp_ros_time_difference_ns = round(
            self.max_ptp_ros_time_difference_sec * 1_000_000_000
        )
        self._ptp_lock_required = (
            self.require_ptp_lock
            and self.timestamp_mode
            == core.TimestampMode.TIME_FROM_PTP_1588
            and self.ros_stamp_source == 'sensor_scan_midpoint'
        )

        if self.timestamp_mode == core.TimestampMode.TIME_FROM_INTERNAL_OSC:
            self.get_logger().warning(
                'INTERNAL_OSC timestamps are relative to sensor power-on, '
                'not epoch. Cross-sensor sync will not work with those '
                'sensor timestamps.'
            )
        if self.ros_stamp_source == 'host_receipt':
            self.get_logger().warning(
                'ros_stamp_source=host_receipt samples ROS time as soon as a '
                'scan is delivered, before XYZ conversion and cloud packing. '
                'It is not the LiDAR measurement time.'
            )
        elif self.timestamp_mode != core.TimestampMode.TIME_FROM_PTP_1588:
            self.get_logger().warning(
                'sensor_scan_midpoint is not in a shared epoch. Do not '
                'approximately synchronize it with epoch-stamped cameras.'
            )

        if (
            self.timestamp_mode == core.TimestampMode.TIME_FROM_PTP_1588
            and self.ros_stamp_source == 'sensor_scan_midpoint'
        ):
            self.get_logger().info(
                f'PTP sensor timestamps will receive '
                f'{self.ptp_utc_tai_offset_sec:g} s UTC/TAI conversion and '
                f'must be within {self.max_ptp_ros_time_difference_sec:g} s '
                f'of ROS time.'
            )

        phase_lock_enabled = _configured_bool(
            self.lidar_config,
            'phase_lock_enable',
            False,
            'lidar.phase_lock_enable',
        )
        if (
            phase_lock_enabled
            and self.timestamp_mode
            == core.TimestampMode.TIME_FROM_INTERNAL_OSC
        ):
            self.get_logger().warning(
                'Phase lock is referenced to the free-running internal '
                'oscillator, not a clock shared with the camera.'
            )
        if (
            not self.require_ptp_lock
            and self.timestamp_mode == core.TimestampMode.TIME_FROM_PTP_1588
            and self.ros_stamp_source == 'sensor_scan_midpoint'
        ):
            self.get_logger().warning(
                'PTP lock enforcement is disabled. Sensor timestamps may '
                'not share an epoch with the camera.'
            )

    def _configure_scan_policy(self):
        """Validate scan-integrity limits used before cloud conversion."""
        self.min_scan_valid_columns_ratio = _configured_float(
            self.lidar_config,
            'min_scan_valid_columns_ratio',
            0.0,
            'lidar.min_scan_valid_columns_ratio',
            minimum=0.0,
            maximum=1.0,
        )

    def _make_sensor_config(self):
        """Translate validated YAML values into an Ouster SensorConfig."""
        config = core.SensorConfig()
        config.udp_port_lidar = int(self.sensor_config['udp_port_lidar'])
        config.udp_port_imu = int(self.sensor_config['udp_port_imu'])
        config.operating_mode = core.OperatingMode.OPERATING_NORMAL
        config.lidar_mode = _configured_enum(
            LIDAR_MODES,
            self.lidar_config['mode'],
            'lidar.mode',
        )
        config.timestamp_mode = self.timestamp_mode
        config.multipurpose_io_mode = _configured_enum(
            MULTIPURPOSE_IO_MODES,
            self.lidar_config.get('multipurpose_io_mode', 'OFF'),
            'lidar.multipurpose_io_mode',
        )
        azimuth_window = tuple(
            int(value) for value in self.lidar_config.get(
                'azimuth_window', [0, 360000]
            )
        )
        if (
            len(azimuth_window) != 2
            or any(value < 0 or value > 360000 for value in azimuth_window)
        ):
            raise ValueError(
                'lidar.azimuth_window must contain two values from 0 to '
                '360000 millidegrees.'
            )
        config.azimuth_window = azimuth_window
        config.phase_lock_enable = _configured_bool(
            self.lidar_config,
            'phase_lock_enable',
            False,
            'lidar.phase_lock_enable',
        )
        phase_lock_offset = int(
            self.lidar_config.get('phase_lock_offset', 0)
        )
        if phase_lock_offset < 0 or phase_lock_offset > 360000:
            raise ValueError(
                'lidar.phase_lock_offset must be from 0 to 360000 '
                'millidegrees.'
            )
        config.phase_lock_offset = phase_lock_offset
        return config

    def _configure_sensor(self):
        """Apply the requested sensor configuration and verify key values."""
        sensor.set_config(
            self.hostname,
            self.config,
            persist=self.persist_config,
            udp_dest_auto=True,
        )
        active_config = sensor.get_config(self.hostname, active=True)
        expected = {
            'udp_port_lidar': self.config.udp_port_lidar,
            'udp_port_imu': self.config.udp_port_imu,
            'operating_mode': self.config.operating_mode,
            'lidar_mode': self.config.lidar_mode,
            'timestamp_mode': self.config.timestamp_mode,
            'multipurpose_io_mode': self.config.multipurpose_io_mode,
            'azimuth_window': self.config.azimuth_window,
            'phase_lock_enable': self.config.phase_lock_enable,
        }
        if self.config.phase_lock_enable:
            expected['phase_lock_offset'] = self.config.phase_lock_offset
        mismatches = []
        for name, requested_value in expected.items():
            active_value = getattr(active_config, name)
            if name == 'azimuth_window' and active_value is not None:
                active_value = tuple(active_value)
                requested_value = tuple(requested_value)
            if active_value != requested_value:
                mismatches.append(
                    f'{name}: requested {requested_value}, active '
                    f'{active_value}'
                )
        if mismatches:
            raise RuntimeError(
                'Ouster rejected part of its configuration: '
                + '; '.join(mismatches)
            )
        self.active_config = active_config

    def _make_qos_profile(self):
        """Build the configured ROS QoS profile."""
        qos = self.config_file['qos']
        try:
            reliability = getattr(ReliabilityPolicy, qos['reliability'])
            history = getattr(HistoryPolicy, qos['history'])
            depth = int(qos['depth'])
        except (AttributeError, KeyError, TypeError, ValueError) as error:
            raise ValueError(f'Invalid Ouster QoS configuration: {error}')
        if depth <= 0:
            raise ValueError('qos.depth must be positive.')
        return QoSProfile(
            reliability=reliability,
            history=history,
            depth=depth,
        )

    def _log_setup(self, metadata):
        """Log the configured mode and optional sensor identity details."""
        mode = self.lidar_config.get('info_mode', 'minimal')
        self.get_logger().info(
            f'Ouster {self.hostname}: mode={self.config.lidar_mode}, '
            f'timestamp_mode={self.timestamp_mode_name}, '
            f'ros_stamp_source={self.ros_stamp_source}, '
            f'azimuth_window={self.config.azimuth_window}, '
            f'phase_lock={self.config.phase_lock_enable}, '
            f'min_valid_columns={self.min_scan_valid_columns_ratio:.3f}, '
            f'persist_config={self.persist_config}'
        )
        if mode == 'detailed':
            self.get_logger().info(
                f'Model={metadata.prod_line}, serial={metadata.sn}, '
                f'firmware={metadata.fw_rev}, '
                f'udp_dest={self.active_config.udp_dest}'
            )
        elif mode != 'minimal':
            self.get_logger().warning(
                f'Unknown lidar.info_mode {mode!r}; using minimal output.'
            )

    def _set_cloud_stamp(self, header, scan, valid_columns, receipt_stamp):
        """Set the cloud header stamp according to the configured policy."""
        if self.ros_stamp_source == 'host_receipt':
            header.stamp = receipt_stamp
            return True

        timestamp_ns = scan_midpoint_ns(scan.timestamp, valid_columns)
        if timestamp_ns is None:
            return False
        if self.timestamp_mode == core.TimestampMode.TIME_FROM_PTP_1588:
            receipt_ros_ns = (
                int(receipt_stamp.sec) * 1_000_000_000
                + int(receipt_stamp.nanosec)
            )
            timestamp_ns, error = ptp_timestamp_to_ros_ns(
                timestamp_ns,
                self._ptp_utc_tai_offset_ns,
                receipt_ros_ns,
                self._max_ptp_ros_time_difference_ns,
            )
            if error is not None:
                self.get_logger().warning(
                    f'Rejecting Ouster PTP timestamp: {error}. Check the '
                    f'grandmaster, host clock, and ptp_utc_tai_offset.',
                    throttle_duration_sec=5.0,
                )
                return False
        header.stamp.sec = timestamp_ns // 1_000_000_000
        header.stamp.nanosec = timestamp_ns % 1_000_000_000
        return True

    def _ptp_monitor_loop(self):
        """Continuously permit sensor stamps only while PTP is locked."""
        previous_state = None
        while not self._stop_event.is_set():
            timebase_unverified = False
            try:
                status = fetch_ptp_status(self.hostname)
                locked, reason = evaluate_ptp_lock(
                    status, PTP_MAX_MASTER_OFFSET_NS
                )
                if locked:
                    timebase_ready, timebase_reason = evaluate_ptp_timebase(
                        status, self.ptp_utc_tai_offset_sec
                    )
                    time_properties = status.get('time_properties_data_set')
                    timebase_unverified = (
                        not isinstance(time_properties, dict)
                        or (
                            time_properties.get('ptp_timescale') in (True, 1)
                            and time_properties.get(
                                'current_utc_offset_valid'
                            ) not in (True, 1)
                        )
                    )
                    locked = timebase_ready
                    reason = f'{reason}; {timebase_reason}'
            except Exception as error:
                locked = False
                reason = f'PTP status request failed: {error}'

            if locked:
                self._ptp_lock_event.set()
                if previous_state is not True:
                    if timebase_unverified:
                        self.get_logger().warning(f'Ouster PTP {reason}.')
                    else:
                        self.get_logger().info(f'Ouster PTP {reason}.')
            else:
                self._ptp_lock_event.clear()
                self.get_logger().warning(
                    f'Ouster PTP is not ready: {reason}. LiDAR clouds using '
                    'sensor_scan_midpoint will not be published.',
                    throttle_duration_sec=5.0,
                )
            previous_state = locked
            self._stop_event.wait(PTP_POLL_INTERVAL_SEC)

    def _stream_worker(self):
        """Run blocking sensor I/O outside the ROS executor thread."""
        try:
            self.publish_pointcloud()
            if (
                not self._stop_event.is_set()
                and rclpy.ok(context=self.context)
            ):
                raise RuntimeError('Ouster stream ended unexpectedly.')
        except Exception as error:
            if not self._stop_event.is_set():
                self._fatal_error = RuntimeError(
                    f'Ouster streaming thread failed: {error}'
                )
                self.get_logger().error(str(self._fatal_error))
        finally:
            self._stop_event.set()

    def _check_stream_health(self):
        """Raise streaming failures from the ROS executor thread."""
        if self._fatal_error is not None:
            error = self._fatal_error
            self._fatal_error = None
            raise error

    def _close_stream(self, expected_stream=None):
        """Detach and close the live stream exactly once across threads."""
        with self._stream_lock:
            stream = self._stream
            if expected_stream is not None and stream is not expected_stream:
                return
            self._stream = None
        if stream is not None:
            stream.close()

    def publish_pointcloud(self):
        """Read and publish aligned valid points until ROS shuts down."""
        stream = sensor.SensorScanSource(
            self.hostname,
            lidar_port=self.config.udp_port_lidar,
            imu_port=self.config.udp_port_imu,
            do_not_reinitialize=True,
        )
        with self._stream_lock:
            self._stream = stream
        try:
            metadata = stream.sensor_info[0]
            xyz_lut = core.XYZLut(metadata)
            self._log_setup(metadata)
            warned_about_empty_scan = False
            warned_about_timestamp = False
            last_dropped_scans = int(stream.dropped_scans())

            for scans in stream:
                if (
                    self._stop_event.is_set()
                    or not rclpy.ok(context=self.context)
                ):
                    break

                dropped_scans = int(stream.dropped_scans())
                if dropped_scans > last_dropped_scans:
                    self.get_logger().warning(
                        f'Ouster receive queue dropped '
                        f'{dropped_scans - last_dropped_scans} scan(s); '
                        f'total dropped={dropped_scans}.',
                        throttle_duration_sec=5.0,
                    )
                last_dropped_scans = dropped_scans

                scan = scans[0]
                if scan is None:
                    continue
                if not self._ptp_lock_event.is_set():
                    continue

                receipt_stamp = self.get_clock().now().to_msg()
                valid_columns = (scan.status & 0x01) != 0
                column_ratio = valid_column_ratio(valid_columns)
                if column_ratio < self.min_scan_valid_columns_ratio:
                    self.get_logger().warning(
                        f'Skipping incomplete Ouster scan: '
                        f'{column_ratio:.1%} valid columns is below the '
                        f'configured '
                        f'{self.min_scan_valid_columns_ratio:.1%}.',
                        throttle_duration_sec=5.0,
                    )
                    continue

                header = Header()
                header.frame_id = self.lidar_config['frame_id']
                if not self._set_cloud_stamp(
                    header, scan, valid_columns, receipt_stamp
                ):
                    if not warned_about_timestamp:
                        self.get_logger().warning(
                            'Skipping LiDAR scan with an invalid or '
                            'implausible sensor timestamp.'
                        )
                        warned_about_timestamp = True
                    continue
                warned_about_timestamp = False

                ranges = scan.field(core.ChanField.RANGE)
                reflectivity = scan.field(core.ChanField.REFLECTIVITY)
                xyz = xyz_lut(scan)

                # All arrays remain in native staggered order. A single mask
                # preserves the XYZ/reflectivity pixel correspondence.
                valid_points = (
                    (ranges > 0)
                    & valid_columns[np.newaxis, :]
                    & np.isfinite(xyz).all(axis=-1)
                )

                if not np.any(valid_points):
                    if not warned_about_empty_scan:
                        self.get_logger().warning(
                            'Skipping LiDAR scan with no valid points.'
                        )
                        warned_about_empty_scan = True
                    continue

                warned_about_empty_scan = False
                xyz_points = xyz[valid_points].astype(np.float32, copy=False)
                reflectivity = reflectivity[valid_points].astype(
                    np.float32, copy=False
                )
                points = np.column_stack((xyz_points, reflectivity))

                pointcloud = pc2.create_cloud(header, XYZI_FIELDS, points)
                # The common validity mask removed zero-range and non-finite
                # points, so downstream consumers can trust the dense flag.
                pointcloud.is_dense = True
                self.publisher.publish(pointcloud)
        finally:
            self._close_stream(expected_stream=stream)

    def shutdown(self):
        """Stop sensor workers and close the live stream."""
        if self._shutdown_started:
            return
        self._shutdown_started = True
        self._stop_event.set()

        try:
            self._close_stream()
        except Exception as error:
            self.get_logger().warning(
                f'Error while closing the Ouster stream: {error}'
            )

        current_thread = threading.current_thread()
        for worker, name in (
            (self._stream_thread, 'stream'),
            (self._ptp_monitor_thread, 'PTP monitor'),
        ):
            if (
                worker is not None
                and worker.is_alive()
                and worker is not current_thread
            ):
                worker.join(timeout=3.0)
                if worker.is_alive():
                    self.get_logger().warning(
                        f'Ouster {name} thread did not stop within three '
                        'seconds.'
                    )


def main(args=None):
    """Run the Ouster publisher until shutdown."""
    rclpy.init(args=args)
    publisher = None
    try:
        publisher = OusterLidarPublisher()
        publisher.start_streaming()
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if publisher is not None:
            publisher.shutdown()
            publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
