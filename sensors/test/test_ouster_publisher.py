"""Focused tests for Ouster publisher configuration and scan timing."""

import threading
from types import SimpleNamespace

import numpy as np
from ouster.sdk import core
import pytest

from scripts import ouster_publisher
from scripts.ouster_publisher import (
    OusterLidarPublisher,
    _configured_bool,
    _configured_enum,
    _configured_float,
    evaluate_ptp_lock,
    evaluate_ptp_timebase,
    ptp_timestamp_to_ros_ns,
    scan_midpoint_ns,
    valid_column_ratio,
)


def test_scan_midpoint_uses_only_valid_positive_columns():
    """Invalid columns and zero sentinel timestamps must be ignored."""
    timestamps = np.array([0, 100, 200, 900, 1000], dtype=np.uint64)
    valid_columns = np.array([True, True, True, False, True])

    assert scan_midpoint_ns(timestamps, valid_columns) == 550


def test_scan_midpoint_returns_none_without_usable_timestamp():
    """A scan without a positive valid column timestamp has no midpoint."""
    timestamps = np.array([0, 50], dtype=np.uint64)
    valid_columns = np.array([True, False])

    assert scan_midpoint_ns(timestamps, valid_columns) is None


def test_scan_midpoint_rejects_mismatched_shapes():
    """Column timestamps and validity must describe the same scan."""
    with pytest.raises(ValueError, match='same shape'):
        scan_midpoint_ns(np.array([1, 2]), np.array([True]))


def test_configured_enum_rejects_unknown_value():
    """Configuration typos must not silently select another sensor mode."""
    with pytest.raises(ValueError, match='supported values: known'):
        _configured_enum({'known': 1}, 'typo', 'setting')


def test_configured_bool_rejects_yaml_strings():
    """A quoted false value must not silently become true in Python."""
    with pytest.raises(ValueError, match='must be true or false'):
        _configured_bool({'enabled': 'false'}, 'enabled', False, 'enabled')


@pytest.mark.parametrize('value', ['-37', float('nan'), True, -101, 101])
def test_configured_float_rejects_wrong_type_or_range(value):
    """Clock offsets must be finite numeric YAML values in a sane range."""
    with pytest.raises(ValueError):
        _configured_float(
            {'offset': value},
            'offset',
            -37.0,
            'offset',
            minimum=-100.0,
            maximum=100.0,
        )


def test_evaluate_ptp_timebase_accepts_announced_tai_utc_offset():
    """A valid currentUtcOffset determines the required conversion sign."""
    status = {
        'time_properties_data_set': {
            'ptp_timescale': 1,
            'current_utc_offset_valid': 1,
            'current_utc_offset': 37,
        }
    }

    ready, reason = evaluate_ptp_timebase(status, -37.0)

    assert ready
    assert 'TAI-UTC=37' in reason


def test_evaluate_ptp_timebase_rejects_missing_tai_to_utc_conversion():
    """TAI seconds must not be published directly as ROS/UTC seconds."""
    status = {
        'time_properties_data_set': {
            'ptp_timescale': True,
            'current_utc_offset_valid': True,
            'current_utc_offset': 37,
        }
    }

    ready, reason = evaluate_ptp_timebase(status, 0.0)

    assert not ready
    assert 'must be -37 s' in reason


def test_evaluate_ptp_timebase_requires_zero_for_non_tai_clock():
    """A UTC-like grandmaster must not receive a second UTC conversion."""
    status = {'time_properties_data_set': {'ptp_timescale': 0}}

    ready, reason = evaluate_ptp_timebase(status, -37.0)

    assert not ready
    assert 'must be 0.0' in reason


def test_ptp_timestamp_conversion_applies_negative_37_seconds():
    """The PTP/TAI epoch is converted before it enters a ROS header."""
    converted, error = ptp_timestamp_to_ros_ns(
        1_737_000_037_025_000_000,
        -37_000_000_000,
        1_737_000_000_030_000_000,
        1_000_000_000,
    )

    assert error is None
    assert converted == 1_737_000_000_025_000_000


def test_ptp_timestamp_conversion_rejects_wrong_epoch():
    """The plausibility guard catches an omitted UTC/TAI conversion."""
    converted, error = ptp_timestamp_to_ros_ns(
        1_737_000_037_025_000_000,
        0,
        1_737_000_000_030_000_000,
        1_000_000_000,
    )

    assert converted is None
    assert '36.995000 s' in error


def test_valid_column_ratio_counts_sensor_status_bits():
    """Partial scans are measured before expensive XYZ conversion."""
    assert valid_column_ratio([True, True, False, True]) == 0.75


def test_evaluate_ptp_lock_accepts_a_disciplined_slave():
    """PTP publication is allowed only for a present, close grandmaster."""
    status = {
        'port_data_set': {'port_state': 'SLAVE'},
        'time_status_np': {
            'gm_present': True,
            'master_offset': -1250,
        },
        'current_data_set': {'offset_from_master': -1250},
    }

    locked, reason = evaluate_ptp_lock(status, 250_000)

    assert locked
    assert '1250 ns' in reason


@pytest.mark.parametrize(
    ('port_state', 'gm_present', 'master_offset', 'reason_fragment'),
    [
        ('MASTER', True, 0, 'port_state'),
        ('SLAVE', False, 0, 'no PTP grandmaster'),
        ('SLAVE', True, 250_001, 'exceeds'),
    ],
)
def test_evaluate_ptp_lock_rejects_unsynchronized_states(
    port_state, gm_present, master_offset, reason_fragment
):
    """Each required PTP health condition is independently enforced."""
    status = {
        'port_data_set': {'port_state': port_state},
        'time_status_np': {
            'gm_present': gm_present,
            'master_offset': master_offset,
        },
        'current_data_set': {'offset_from_master': master_offset},
    }

    locked, reason = evaluate_ptp_lock(status, 250_000)

    assert not locked
    assert reason_fragment in reason


def test_start_streaming_returns_while_worker_is_running():
    """The blocking stream must not prevent the ROS executor from starting."""
    publisher = object.__new__(OusterLidarPublisher)
    release_worker = threading.Event()
    publisher._stream_thread = None
    publisher._ptp_monitor_thread = None
    publisher._stop_event = threading.Event()
    publisher._ptp_lock_event = threading.Event()
    publisher._ptp_lock_required = False
    publisher._stream_worker = release_worker.wait

    publisher.start_streaming()

    assert publisher._stream_thread.is_alive()
    assert publisher._ptp_lock_event.is_set()
    release_worker.set()
    publisher._stream_thread.join(timeout=1.0)
    assert not publisher._stream_thread.is_alive()


def test_sensor_configuration_is_active_but_not_persisted(monkeypatch):
    """Normal launches must not rewrite persistent sensor configuration."""
    publisher = object.__new__(OusterLidarPublisher)
    publisher.hostname = 'sensor.example'
    publisher.persist_config = False
    publisher.config = core.SensorConfig()
    publisher.config.udp_port_lidar = 7502
    publisher.config.udp_port_imu = 7503
    publisher.config.operating_mode = (
        core.OperatingMode.OPERATING_NORMAL
    )
    publisher.config.lidar_mode = core.LidarMode.MODE_1024x20
    publisher.config.timestamp_mode = (
        core.TimestampMode.TIME_FROM_INTERNAL_OSC
    )
    publisher.config.multipurpose_io_mode = (
        core.MultipurposeIOMode.MULTIPURPOSE_OFF
    )
    publisher.config.azimuth_window = (0, 360000)
    publisher.config.phase_lock_enable = False

    active_config = SimpleNamespace(
        udp_port_lidar=7502,
        udp_port_imu=7503,
        operating_mode=publisher.config.operating_mode,
        lidar_mode=publisher.config.lidar_mode,
        timestamp_mode=publisher.config.timestamp_mode,
        multipurpose_io_mode=publisher.config.multipurpose_io_mode,
        azimuth_window=[0, 360000],
        phase_lock_enable=False,
    )
    set_calls = []

    def fake_set_config(hostname, config, **options):
        """Record the SDK configuration call."""
        set_calls.append((hostname, config, options))

    monkeypatch.setattr(
        ouster_publisher.sensor, 'set_config', fake_set_config
    )
    monkeypatch.setattr(
        ouster_publisher.sensor,
        'get_config',
        lambda hostname, active: active_config,
    )

    publisher._configure_sensor()

    assert set_calls[0][0] == 'sensor.example'
    assert set_calls[0][2] == {
        'persist': False,
        'udp_dest_auto': True,
    }


def test_stream_is_closed_only_once_across_shutdown_paths():
    """Worker and main-thread cleanup must not double-close SDK I/O."""
    publisher = object.__new__(OusterLidarPublisher)
    publisher._stream_lock = threading.Lock()
    stream = SimpleNamespace(close_calls=0)

    def close_stream():
        """Record one simulated SDK close call."""
        stream.close_calls += 1

    stream.close = close_stream
    publisher._stream = stream

    publisher._close_stream()
    publisher._close_stream(expected_stream=stream)

    assert stream.close_calls == 1
