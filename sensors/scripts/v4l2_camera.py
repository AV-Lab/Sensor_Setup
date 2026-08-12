"""V4L2 device discovery and configuration for the See3CAM publisher."""

import math
import os
from pathlib import Path

import cv2


DEFAULT_BY_ID_DIRECTORY = '/dev/v4l/by-id'
STANDARD_CONTROL_PROPERTIES = {
    'auto_exposure': cv2.CAP_PROP_AUTO_EXPOSURE,
    'auto_white_balance': cv2.CAP_PROP_AUTO_WB,
    'exposure': cv2.CAP_PROP_EXPOSURE,
    'gain': cv2.CAP_PROP_GAIN,
    'brightness': cv2.CAP_PROP_BRIGHTNESS,
    'contrast': cv2.CAP_PROP_CONTRAST,
    'saturation': cv2.CAP_PROP_SATURATION,
    'sharpness': cv2.CAP_PROP_SHARPNESS,
}


def control_readback_matches(requested, actual):
    """Return whether a finite control readback matches within two percent."""
    requested = float(requested)
    actual = float(actual)
    tolerance = max(0.01, abs(requested) * 0.02)
    return math.isfinite(actual) and abs(actual - requested) <= tolerance


def _finite_readback(capture, property_id):
    """Return a finite OpenCV control readback or ``None`` if unavailable."""
    value = float(capture.get(property_id))
    return value if math.isfinite(value) else None


def discover_camera_device(model, by_id_directory=DEFAULT_BY_ID_DIRECTORY):
    """Return the unique video-index0 by-id path matching a camera model."""
    model_key = _normalize_name(model)
    directory = Path(by_id_directory)
    candidates = [
        path
        for path in directory.glob('*')
        if model_key in _normalize_name(path.name)
        and 'videoindex0' in _normalize_name(path.name)
        and path.exists()
    ]

    if not candidates:
        raise RuntimeError(
            f'No {model!r} video-index0 device found under {directory}. '
            'Connect the camera and check its /dev/v4l/by-id entry.'
        )
    if len(candidates) > 1:
        paths = ', '.join(str(path) for path in sorted(candidates))
        raise RuntimeError(
            f'Multiple {model!r} cameras were found: {paths}. '
            'Automatic selection requires exactly one.'
        )
    return str(candidates[0])


def _normalize_name(value):
    """Normalize names for punctuation-insensitive matching."""
    return ''.join(
        character.lower()
        for character in str(value)
        if character.isalnum()
    )


class V4L2Camera:
    """Own and configure one OpenCV V4L2 capture device."""

    def __init__(self, camera_config, capture_config, controls, logger):
        """Resolve, open, and configure the requested camera."""
        self._camera_config = camera_config
        self._capture_config = capture_config
        self._controls = controls
        self._logger = logger
        self._capture = None
        self.active_mode = None
        self.active_controls = {}

        self.device = self._resolve_device()
        self._open()
        try:
            self._configure_mode()
            self._apply_standard_controls()
            self._discard_warmup_frames()
        except Exception:
            self.release()
            raise

    def _resolve_device(self):
        """Resolve an automatic model match or preserve an explicit device."""
        configured_device = self._camera_config['device']
        if configured_device != 'auto':
            if isinstance(configured_device, int):
                self._logger.warning(
                    f'Opening index {configured_device}. Its number can '
                    'change after a reboot or USB reconnect.'
                )
            return configured_device

        device = discover_camera_device(self._camera_config['device_match'])
        target = os.path.realpath(device)
        self._logger.info(
            f'Automatically selected camera {device} -> {target}'
        )
        return device

    def _open(self):
        """Open the resolved device through the Linux V4L2 backend."""
        self._capture = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not self._capture.isOpened():
            raise RuntimeError(
                f'Failed to open camera device {self.device!r}.'
            )

    def _configure_mode(self):
        """Request and verify format, image size, frame rate, and buffering."""
        config = self._camera_config
        pixel_format = config['pixel_format']
        properties = (
            (cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*pixel_format)),
            (cv2.CAP_PROP_FRAME_WIDTH, config['width']),
            (cv2.CAP_PROP_FRAME_HEIGHT, config['height']),
            (cv2.CAP_PROP_FPS, config['fps']),
        )
        for property_id, value in properties:
            self._capture.set(property_id, value)

        buffer_size = self._capture_config['buffer_size']
        if not self._capture.set(cv2.CAP_PROP_BUFFERSIZE, buffer_size):
            self._logger.warning(
                'V4L2 did not confirm CAP_PROP_BUFFERSIZE; verify buffering '
                'on the deployed camera.'
            )

        actual = {
            'width': int(round(self._capture.get(cv2.CAP_PROP_FRAME_WIDTH))),
            'height': int(
                round(self._capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
            ),
            'fps': float(self._capture.get(cv2.CAP_PROP_FPS)),
            'format': self._decode_fourcc(
                self._capture.get(cv2.CAP_PROP_FOURCC)
            ),
        }
        self.active_mode = actual
        self._logger.info(
            f'Camera mode: {actual["width"]}x{actual["height"]} at '
            f'{actual["fps"]:.3f} Hz, format={actual["format"]}, backend=V4L2'
        )

        mismatches = self._mode_mismatches(actual)
        if mismatches:
            message = (
                'Camera rejected requested mode: ' + '; '.join(mismatches)
            )
            if config['strict_mode']:
                raise RuntimeError(message)
            self._logger.warning(message)

    def _mode_mismatches(self, actual):
        """Compare requested and effective camera modes."""
        config = self._camera_config
        mismatches = []
        for name in ('width', 'height'):
            if actual[name] != config[name]:
                mismatches.append(
                    f'{name} requested={config[name]}, actual={actual[name]}'
                )
        requested_fps = float(config['fps'])
        if abs(actual['fps'] - requested_fps) > max(0.1, requested_fps * 0.01):
            mismatches.append(
                f'fps requested={requested_fps:.3f}, '
                f'actual={actual["fps"]:.3f}'
            )
        if actual['format'] != config['pixel_format']:
            mismatches.append(
                f'format requested={config["pixel_format"]}, '
                f'actual={actual["format"]}'
            )
        return mismatches

    @staticmethod
    def _decode_fourcc(value):
        """Convert an OpenCV numeric FOURCC value into four characters."""
        fourcc = int(round(value))
        return ''.join(
            chr((fourcc >> (8 * index)) & 0xFF)
            for index in range(4)
        )

    def _apply_standard_controls(self):
        """Apply optional controls that OpenCV exposes through V4L2."""
        if not self._controls.get('apply_standard_controls', False):
            self._record_control_readback(set_call_accepted=False)
            self._logger.warning(
                'Camera controls are disabled until the See3CAM controls are '
                'verified with v4l2-ctl.'
            )
            return

        auto_exposure = bool(self._controls.get('auto_exposure', True))
        self._set_control(
            'auto_exposure',
            cv2.CAP_PROP_AUTO_EXPOSURE,
            0.75 if auto_exposure else 0.25,
        )
        self._set_control(
            'auto_white_balance',
            cv2.CAP_PROP_AUTO_WB,
            1.0 if self._controls.get('auto_white_balance', True) else 0.0,
        )
        if not auto_exposure and self._controls.get('exposure') is not None:
            self._set_control(
                'exposure', cv2.CAP_PROP_EXPOSURE, self._controls['exposure']
            )

        optional = (
            ('gain', cv2.CAP_PROP_GAIN),
            ('brightness', cv2.CAP_PROP_BRIGHTNESS),
            ('contrast', cv2.CAP_PROP_CONTRAST),
            ('saturation', cv2.CAP_PROP_SATURATION),
            ('sharpness', cv2.CAP_PROP_SHARPNESS),
        )
        for name, property_id in optional:
            if self._controls.get(name) is not None:
                self._set_control(name, property_id, self._controls[name])

        # Archive readback for controls that were not explicitly written too.
        for name, property_id in STANDARD_CONTROL_PROPERTIES.items():
            if name not in self.active_controls:
                self.active_controls[name] = {
                    'requested': self._controls.get(name),
                    'actual': _finite_readback(self._capture, property_id),
                    'set_call_accepted': False,
                    'readback_matches_request': None,
                }

    def _set_control(self, name, property_id, value):
        """Set one standard control and report its effective value."""
        if not self._capture.set(property_id, float(value)):
            raise RuntimeError(
                f'Camera rejected {name}={value!r}; it may require an e-con '
                'extension control.'
            )
        actual = self._capture.get(property_id)
        matches = control_readback_matches(value, actual)
        self.active_controls[name] = {
            'requested': value,
            'actual': float(actual),
            'set_call_accepted': True,
            'readback_matches_request': matches,
        }
        if not matches:
            raise RuntimeError(
                f'Camera {name} readback {actual!r} does not match the '
                f'requested value {value!r}.'
            )
        self._logger.info(
            f'Camera control {name}: requested={value}, actual={actual}'
        )

    def _record_control_readback(self, set_call_accepted):
        """Read standard V4L2 controls without claiming vendor verification."""
        self.active_controls = {
            name: {
                'requested': self._controls.get(name),
                'actual': _finite_readback(self._capture, property_id),
                'set_call_accepted': set_call_accepted,
                'readback_matches_request': None,
            }
            for name, property_id in STANDARD_CONTROL_PROPERTIES.items()
        }

    def _refresh_control_readback(self):
        """Refresh all values after warm-up and recheck applied controls."""
        for name, property_id in STANDARD_CONTROL_PROPERTIES.items():
            entry = self.active_controls[name]
            actual = _finite_readback(self._capture, property_id)
            entry['actual'] = actual
            if entry['set_call_accepted']:
                entry['readback_matches_request'] = (
                    actual is not None
                    and control_readback_matches(entry['requested'], actual)
                )
                if not entry['readback_matches_request']:
                    raise RuntimeError(
                        f'Camera {name} changed after warm-up: requested '
                        f'{entry["requested"]!r}, active {actual!r}.'
                    )

    def runtime_metadata(self):
        """Return the resolved device, active mode, and control readback."""
        self._refresh_control_readback()
        return {
            'backend': 'V4L2',
            'configured_device': self._camera_config['device'],
            'resolved_device': str(self.device),
            'resolved_target': os.path.realpath(str(self.device)),
            'active_mode': dict(self.active_mode or {}),
            'controls_requested_for_application': bool(
                self._controls.get('apply_standard_controls', False)
            ),
            'active_standard_controls': dict(self.active_controls),
            'applied_standard_controls_verified': bool(
                self._controls.get('apply_standard_controls', False)
                and all(
                    entry['readback_matches_request'] is True
                    for entry in self.active_controls.values()
                    if entry['set_call_accepted']
                )
            ),
            'control_readback_limitation': (
                'OpenCV exposes standard V4L2 properties only; e-con extension '
                'controls require separate vendor/V4L2 verification.'
            ),
        }

    def _discard_warmup_frames(self):
        """Discard configured free-running frames before publication."""
        frame_count = self._capture_config['warmup_frames']
        if frame_count:
            self._logger.info(f'Discarding {frame_count} warm-up frames.')
        for frame_index in range(frame_count):
            if not self.grab():
                raise RuntimeError(
                    f'Failed to grab warm-up frame {frame_index + 1}/'
                    f'{frame_count}.'
                )

    def grab(self):
        """Grab the next frame without decoding it."""
        return self._capture.grab()

    def retrieve(self):
        """Retrieve and decode the most recently grabbed frame."""
        return self._capture.retrieve()

    def is_opened(self):
        """Return whether the underlying capture device is open."""
        return self._capture is not None and self._capture.isOpened()

    def release(self):
        """Release the V4L2 camera if it is open."""
        if self.is_opened():
            self._capture.release()
