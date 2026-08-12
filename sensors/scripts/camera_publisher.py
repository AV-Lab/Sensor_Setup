"""Publish unrectified images from the configured V4L2 camera."""

import json
import threading

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header, String

from .camera_configuration import load_camera_config
from .camera_diagnostics import CameraDiagnostics
from .v4l2_camera import V4L2Camera


SUPPORTED_TIMESTAMP_SOURCES = {'host_receipt'}


class CameraPublisher(Node):
    """Capture and publish raw camera frames without rectification."""

    def __init__(self):
        """Configure ROS, the V4L2 camera, and the capture thread."""
        super().__init__('camera_publisher')

        self._stop_event = threading.Event()
        self._capture_thread = None
        self._camera = None
        self._fatal_error = None
        self._shutdown_started = False

        self.config = load_camera_config(self)
        self._camera_config = self.config['camera']
        self._capture_config = self.config['capture']
        self._timestamp_source = self._capture_config['timestamp_source']
        self._validate_timestamp_source()
        self._warn_if_using_simulated_time()

        self._diagnostics = CameraDiagnostics(
            logger=self.get_logger(),
            requested_fps=self._camera_config['fps'],
            timestamp_source=self._timestamp_source,
            interval_sec=self._capture_config['diagnostics_interval_sec'],
            max_consecutive_failures=self._capture_config[
                'max_consecutive_failures'
            ],
        )

        qos_profile = self._make_qos_profile()
        self.image_publisher = self.create_publisher(
            Image,
            self.config['ROS']['topic_name'],
            qos_profile,
        )
        self.camera_info_publisher = self.create_publisher(
            CameraInfo,
            self.config['ROS']['camera_info_topic'],
            qos_profile,
        )
        metadata_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.runtime_metadata_publisher = self.create_publisher(
            String,
            self.config['ROS']['runtime_metadata_topic'],
            metadata_qos,
        )

        self._camera = V4L2Camera(
            camera_config=self._camera_config,
            capture_config=self._capture_config,
            controls=self.config['controls'],
            logger=self.get_logger(),
        )
        self.camera_info = self._make_camera_info()
        self._publish_runtime_metadata()
        self._warn_about_host_receipt_timing()

        self._health_timer = self.create_timer(0.2, self._check_capture_health)
        self._capture_thread = threading.Thread(
            target=self._capture_loop,
            name='camera_capture',
            daemon=True,
        )
        self._capture_thread.start()

    def _validate_timestamp_source(self):
        """Reject timestamp sources that are not implemented."""
        if self._timestamp_source not in SUPPORTED_TIMESTAMP_SOURCES:
            supported = ', '.join(sorted(SUPPORTED_TIMESTAMP_SOURCES))
            raise ValueError(
                f'Unsupported timestamp source {self._timestamp_source!r}; '
                f'currently supported: {supported}.'
            )

    def _warn_if_using_simulated_time(self):
        """Warn when a live camera would be stamped from a simulated clock."""
        use_sim_time = self.get_parameter(
            'use_sim_time'
        ).get_parameter_value().bool_value
        self.get_logger().info(f'use_sim_time is set to: {use_sim_time}')
        if use_sim_time:
            self.get_logger().warning(
                'A live camera is using simulated time. Host receipt times '
                'will follow /clock and cannot synchronize live sensors.'
            )

    def _warn_about_host_receipt_timing(self):
        """Describe the limits of the current timestamp implementation."""
        self.get_logger().warning(
            'timestamp_source=host_receipt samples ROS time after grab(), '
            'before retrieve/decode and publication. It is still later than '
            'exposure and USB transfer.'
        )
        self.get_logger().warning(
            'OpenCV does not expose the V4L2 hardware frame sequence here. '
            'Explicit capture failures are detected, but silent device-side '
            'frame drops cannot yet be proven.'
        )

    def _make_qos_profile(self):
        """Build the configured ROS publisher QoS profile."""
        qos = self.config['qos']
        try:
            reliability = getattr(ReliabilityPolicy, qos['reliability'])
            history = getattr(HistoryPolicy, qos['history'])
            depth = int(qos['depth'])
        except (AttributeError, KeyError, TypeError, ValueError) as error:
            raise ValueError(f'Invalid camera QoS configuration: {error}')
        if depth <= 0:
            raise ValueError('qos.depth must be positive.')
        return QoSProfile(
            reliability=reliability,
            history=history,
            depth=depth,
        )

    def _make_camera_info(self):
        """Construct CameraInfo for the unrectified published image."""
        camera_info = CameraInfo()
        camera_info.header.frame_id = self.config['ROS']['frame_id']
        camera_info.height = self._camera_config['height']
        camera_info.width = self._camera_config['width']
        intrinsics = self.config['intrinsics']
        camera_info.distortion_model = intrinsics['distortion_model']
        if not intrinsics['calibrated']:
            self.get_logger().warning(
                'Camera intrinsics are uncalibrated. CameraInfo K[0] remains '
                'zero until the See3CAM calibration is installed.'
            )
            return camera_info

        camera_info.k = [
            float(value) for value in intrinsics['camera_matrix_K']
        ]
        camera_info.d = [float(value) for value in intrinsics['distortion']]
        camera_info.r = [float(value) for value in intrinsics['rectification']]
        camera_info.p = [float(value) for value in intrinsics['projection']]
        return camera_info

    def _publish_runtime_metadata(self):
        """Publish durable active camera configuration and control readback."""
        document = {
            'schema_version': 1,
            'publisher': 'sensors.camera_node',
            'frame_id': self.config['ROS']['frame_id'],
            'timestamp_source': self._timestamp_source,
            'camera': self._camera.runtime_metadata(),
            'intrinsics_provenance': self.config['intrinsics'].get(
                'provenance', {}
            ),
        }
        message = String()
        message.data = json.dumps(document, sort_keys=True, allow_nan=False)
        self.runtime_metadata_publisher.publish(message)
        self.get_logger().info(
            'Published durable camera runtime metadata on '
            f'{self.config["ROS"]["runtime_metadata_topic"]}.'
        )

    def _capture_loop(self):
        """Block on camera frames and publish each frame as it arrives."""
        while not self._stop_event.is_set() and rclpy.ok(context=self.context):
            try:
                if not self._camera.grab():
                    self._record_capture_failure('grab() returned false')
                    continue

                receipt_time = self.get_clock().now()
                retrieved, frame = self._camera.retrieve()
                if not retrieved or frame is None:
                    self._record_capture_failure(
                        'retrieve() returned no frame'
                    )
                    continue

                self._diagnostics.record_capture_success()
                frame_bgr = self._normalize_to_bgr(frame)
                header = Header()
                header.stamp = receipt_time.to_msg()
                header.frame_id = self.config['ROS']['frame_id']

                image_message = self._make_image_message(frame_bgr, header)
                self.camera_info.header.stamp = header.stamp
                self.camera_info.header.frame_id = header.frame_id
                self.image_publisher.publish(image_message)
                self.camera_info_publisher.publish(self.camera_info)
                self._diagnostics.record_published_frame(
                    receipt_time.nanoseconds
                )
            except Exception as error:
                self._fatal_error = RuntimeError(
                    f'Camera capture thread failed: {error}'
                )
                self._stop_event.set()

    @staticmethod
    def _normalize_to_bgr(frame):
        """Return an OpenCV frame as three-channel BGR."""
        if frame.ndim == 2:
            return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        if frame.ndim != 3:
            raise ValueError(f'Unsupported camera frame shape: {frame.shape}')
        if frame.shape[2] == 3:
            return frame
        if frame.shape[2] == 4:
            return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        raise ValueError(f'Unsupported camera frame shape: {frame.shape}')

    @staticmethod
    def _make_image_message(frame_bgr, header):
        """Pack one contiguous BGR frame directly into a ROS Image."""
        if not frame_bgr.flags['C_CONTIGUOUS']:
            frame_bgr = frame_bgr.copy(order='C')
        height, width, channels = frame_bgr.shape
        if channels != 3:
            raise ValueError(
                f'Expected a three-channel BGR frame, got {frame_bgr.shape}.'
            )

        message = Image()
        message.header = header
        message.height = height
        message.width = width
        message.encoding = 'bgr8'
        message.is_bigendian = 0
        message.step = width * channels
        message.data = frame_bgr.tobytes()
        return message

    def _record_capture_failure(self, reason):
        """Stop capture after the configured number of consecutive failures."""
        if self._diagnostics.record_capture_failure(reason):
            raise RuntimeError(
                'Camera reached the consecutive capture-failure limit.'
            )

    def _check_capture_health(self):
        """Raise capture-thread failures in the ROS executor thread."""
        if self._fatal_error is not None:
            error = self._fatal_error
            self._fatal_error = None
            raise error

    def shutdown(self):
        """Stop capture and release the camera device."""
        if self._shutdown_started:
            return
        self._shutdown_started = True
        self._stop_event.set()

        if self._camera is not None:
            self._camera.release()
        if (
            self._capture_thread is not None
            and self._capture_thread.is_alive()
            and threading.current_thread() is not self._capture_thread
        ):
            self._capture_thread.join(timeout=2.0)
            if self._capture_thread.is_alive():
                self.get_logger().warning(
                    'Camera capture thread did not stop within two seconds.'
                )


def main(args=None):
    """Run the camera publisher node."""
    rclpy.init(args=args)
    camera_publisher = None
    try:
        camera_publisher = CameraPublisher()
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if camera_publisher is not None:
            camera_publisher.shutdown()
            camera_publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
