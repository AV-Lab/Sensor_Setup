"""Publish unrectified images from the legacy optional ZED camera path."""

import os

from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
import pyzed.sl as sl
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
import yaml


ZED_TIMESTAMP_SOURCES = {
    'IMAGE': sl.TIME_REFERENCE.IMAGE,
    'CURRENT': sl.TIME_REFERENCE.CURRENT,
    'System_Time': None,
}


class ZEDCameraPublisher(Node):
    """Publish the ZED left image using the legacy monocular configuration."""

    def __init__(self):
        """Load configuration, open the ZED, and create ROS publishers."""
        super().__init__('zed_camera_publisher')
        use_sim_time = self.get_parameter(
            'use_sim_time'
        ).get_parameter_value().bool_value
        self.get_logger().info(f'use_sim_time is set to: {use_sim_time}')

        self.config = self._load_config()
        timestamp_name = self.config['camera']['timestamp_mode']
        if timestamp_name not in ZED_TIMESTAMP_SOURCES:
            supported = ', '.join(ZED_TIMESTAMP_SOURCES)
            raise ValueError(
                f'Unsupported ZED timestamp_mode {timestamp_name!r}; '
                f'supported values: {supported}.'
            )
        self._timestamp_source = ZED_TIMESTAMP_SOURCES[timestamp_name]
        self._zed = sl.Camera()

        init_params = sl.InitParameters()
        init_params.set_from_camera_id(int(self.config['camera']['id']))
        init_params.camera_fps = int(self.config['camera']['fps'])
        init_params.camera_resolution = getattr(
            sl.RESOLUTION, self.config['camera']['resolution']
        )
        init_params.depth_mode = getattr(
            sl.DEPTH_MODE, self.config['camera']['depth_mode']
        )
        status = self._zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f'Error opening ZED: {status}')

        qos = self._make_qos_profile()
        self._image_publisher = self.create_publisher(
            Image, self.config['ROS']['topic_name'], qos
        )
        self._camera_info_publisher = self.create_publisher(
            CameraInfo, self.config['ROS']['camera_info_topic'], qos
        )
        self._camera_info = self._make_camera_info()
        self._image = sl.Mat()
        self._timer = self.create_timer(
            1.0 / float(self.config['camera']['fps']), self._publish_image
        )

    @staticmethod
    def _config_path():
        """Return the installed legacy ZED configuration path."""
        share = get_package_share_directory('sensors')
        return os.path.join(share, 'config', 'zed_config.yaml')

    def _load_config(self):
        """Read the legacy ZED YAML configuration."""
        path = self._config_path()
        with open(path, 'r', encoding='utf-8') as config_file:
            config = yaml.safe_load(config_file)
        if not isinstance(config, dict):
            raise ValueError(f'ZED configuration must be a mapping: {path}')
        return config

    def _make_qos_profile(self):
        """Construct the configured image QoS profile."""
        qos = self.config['qos']
        return QoSProfile(
            reliability=getattr(ReliabilityPolicy, qos['reliability']),
            history=getattr(HistoryPolicy, qos['history']),
            depth=int(qos['depth']),
        )

    def _make_camera_info(self):
        """Build CameraInfo for the published raw, distorted left image."""
        camera_info = CameraInfo()
        camera_info.header.frame_id = self.config['ROS']['frame_id']
        camera_info.height = int(self.config['camera']['height'])
        camera_info.width = int(self.config['camera']['width'])
        intrinsics = self.config['intrinsics']
        camera_info.k = [float(value) for value in intrinsics['camera_matrix_K']]
        camera_info.d = [float(value) for value in intrinsics['distortion']]
        camera_info.r = [float(value) for value in intrinsics['rectification']]
        camera_info.p = [float(value) for value in intrinsics['projection']]
        camera_info.distortion_model = 'plumb_bob'
        return camera_info

    def _capture_stamp(self):
        """Return the configured ZED or ROS timestamp as a ROS message."""
        if self._timestamp_source is None:
            return self.get_clock().now().to_msg()
        timestamp_ns = int(
            self._zed.get_timestamp(self._timestamp_source).get_nanoseconds()
        )
        return Time(nanoseconds=timestamp_ns).to_msg()

    def _publish_image(self):
        """Grab and publish one unrectified ZED left image."""
        if self._zed.grab() != sl.ERROR_CODE.SUCCESS:
            self.get_logger().warning(
                'ZED grab failed.', throttle_duration_sec=5.0
            )
            return
        self._zed.retrieve_image(self._image, sl.VIEW.LEFT)
        frame = np.asarray(self._image.get_data())
        if frame.ndim != 3:
            raise RuntimeError(f'Unexpected ZED frame shape: {frame.shape}')
        if frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        elif frame.shape[2] != 3:
            raise RuntimeError(f'Unexpected ZED frame shape: {frame.shape}')
        frame = np.ascontiguousarray(frame)

        header = Header()
        header.frame_id = self.config['ROS']['frame_id']
        header.stamp = self._capture_stamp()
        message = Image()
        message.header = header
        message.height, message.width, channels = frame.shape
        message.encoding = 'bgr8'
        message.is_bigendian = 0
        message.step = message.width * channels
        message.data = frame.tobytes()

        self._camera_info.header = header
        self._image_publisher.publish(message)
        self._camera_info_publisher.publish(self._camera_info)

    def shutdown(self):
        """Close the ZED device."""
        self._zed.close()


def main(args=None):
    """Run the legacy ZED publisher."""
    rclpy.init(args=args)
    node = None
    try:
        node = ZEDCameraPublisher()
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
