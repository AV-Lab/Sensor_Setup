import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

class ClockPublisher(Node):
    def __init__(self):
        super().__init__('clock_publisher')
        self.publisher_ = self.create_publisher(Clock, '/clock', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100 Hz

    def timer_callback(self):
        msg = Clock()
        msg.clock = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    clock_publisher = ClockPublisher()
    rclpy.spin(clock_publisher)
    clock_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()