"""Publish an accepted LiDAR-camera calibration as a static ROS transform."""

from pathlib import Path
import sys

import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

from scripts.calibration_common import CalibrationError
from scripts.calibration_common import invert_transform
from scripts.calibration_common import load_calibration_bundle
from scripts.calibration_common import matrix_to_quaternion_xyzw


def source_parent_transform(bundle):
    """Return T_source_target for a source-parent, target-child TF edge."""
    return invert_transform(bundle.extrinsic.matrix)


def make_transform_message(bundle, stamp):
    """Convert the directed matrix contract into one TransformStamped."""
    matrix = source_parent_transform(bundle)
    quaternion = matrix_to_quaternion_xyzw(matrix[:3, :3])
    message = TransformStamped()
    message.header.stamp = stamp
    message.header.frame_id = bundle.extrinsic.source_frame
    message.child_frame_id = bundle.extrinsic.target_frame
    message.transform.translation.x = float(matrix[0, 3])
    message.transform.translation.y = float(matrix[1, 3])
    message.transform.translation.z = float(matrix[2, 3])
    message.transform.rotation.x = float(quaternion[0])
    message.transform.rotation.y = float(quaternion[1])
    message.transform.rotation.z = float(quaternion[2])
    message.transform.rotation.w = float(quaternion[3])
    return message


class CalibrationTransformPublisher(Node):
    """Validate and broadcast one accepted static calibration."""

    def __init__(self):
        """Load the calibration file and publish exactly one static edge."""
        super().__init__('calibration_transform_publisher')
        self.declare_parameter('config_file', '')
        override = self.get_parameter(
            'config_file'
        ).get_parameter_value().string_value
        bundle = load_calibration_bundle(
            Path(override).expanduser().resolve() if override else None,
            require_valid=True,
        )
        self._broadcaster = StaticTransformBroadcaster(self)
        message = make_transform_message(
            bundle, self.get_clock().now().to_msg()
        )
        self._broadcaster.sendTransform(message)
        self.get_logger().info(
            f'Published static TF parent={message.header.frame_id!r}, '
            f'child={message.child_frame_id!r}. Source matrix convention: '
            'p_target = T_target_source * p_source; TF publishes its inverse '
            'because the source frame is the tree parent.'
        )


def main(args=None):
    """Run the accepted-calibration static transform publisher."""
    rclpy.init(args=args)
    node = None
    try:
        node = CalibrationTransformPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except CalibrationError as error:
        print(f'calibration_tf: {error}', file=sys.stderr)
        return 2
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
