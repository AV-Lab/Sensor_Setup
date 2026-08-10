"""V4L2 device discovery and configuration for the See3CAM publisher."""

import os
from pathlib import Path

import cv2


DEFAULT_BY_ID_DIRECTORY = '/dev/v4l/by-id'


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

    def _set_control(self, name, property_id, value):
        """Set one standard control and report its effective value."""
        if not self._capture.set(property_id, float(value)):
            raise RuntimeError(
                f'Camera rejected {name}={value!r}; it may require an e-con '
                'extension control.'
            )
        actual = self._capture.get(property_id)
        self._logger.info(
            f'Camera control {name}: requested={value}, actual={actual}'
        )

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
