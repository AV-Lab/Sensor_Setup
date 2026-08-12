"""Publish a wall-clock-derived ROS clock for explicit test environments."""

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


class ClockPublisher(Node):
    """Publish system-backed ROS time on ``/clock`` at 100 Hz."""

    def __init__(self):
        """Create the clock publisher and reject recursive simulated time."""
        super().__init__('clock_publisher')
        use_sim_time = self.get_parameter(
            'use_sim_time'
        ).get_parameter_value().bool_value
        if use_sim_time:
            raise ValueError(
                'clock_node must use system time; set use_sim_time=false.'
            )
        self.get_logger().warning(
            'clock_node is a test utility. Do not run it with live sensor nodes '
            'that already use system time.'
        )
        self.publisher = self.create_publisher(Clock, '/clock', 10)
        self.timer = self.create_timer(0.01, self._publish_clock)

    def _publish_clock(self):
        """Publish the current system-backed ROS clock value."""
        message = Clock()
        message.clock = self.get_clock().now().to_msg()
        self.publisher.publish(message)


def main(args=None):
    """Run the explicit test clock publisher."""
    rclpy.init(args=args)
    node = None
    try:
        node = ClockPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
