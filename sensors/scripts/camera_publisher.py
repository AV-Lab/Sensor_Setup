import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from std_msgs.msg import Header
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import yaml
import os
from ament_index_python.packages import get_package_share_directory
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSLivelinessPolicy

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # Get use_sim_time parameter
        use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        self.get_logger().info(f'use_sim_time is set to: {use_sim_time}')
        
        # Load configuration
        config_path = self.load_yaml_file()
        with open(config_path, 'r') as config_file:
            self.config = yaml.safe_load(config_file)
        
        self.timestamp = self.config['camera']['timestamp_mode']
        
        self.camera_info = CameraInfo()
        self.get_camera_info()
        self.camera_k = np.array(self.config['intrinsics']['camera_matrix_K']).reshape(-1,3)
        self.camera_distortion = np.array(self.config['intrinsics']['distortion'])

        self.cap = cv2.VideoCapture(self.config['camera']['id'])
        self.bridge = CvBridge()

        # Initialize camera settings
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config['camera']['width'])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config['camera']['height'])
        self.cap.set(cv2.CAP_PROP_FPS, self.config['camera']['fps'])

        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera")
            exit(1)

        # QoS profile
        qos_profile = QoSProfile(
            reliability=getattr(ReliabilityPolicy, self.config['qos']['reliability']),
            history=getattr(HistoryPolicy, self.config['qos']['history']),
            depth=self.config['qos']['depth']
        )

        # Publisher for ROS2 image topic
        self.image_publisher = self.create_publisher(
            Image, 
            self.config['ROS']['topic_name'], 
            qos_profile
        )
        self.camera_info_publisher = self.create_publisher(
            CameraInfo, 
            self.config['ROS']['camera_info_topic'], 
            2
        )
        
        self.create_timer(1.0 / self.config['camera']['fps'], self.publish_image)
    
    def load_yaml_file(self):
        # Get the directory of the package's shared files
        package_share_directory = get_package_share_directory('sensors')
        
        # Construct the path to config file in the 'config' directory
        config_file_path = os.path.join(package_share_directory, 'config', 'camera_config.yaml')
        
        return config_file_path
    
    def publish_image(self):
        ret, frame = self.cap.read()
        
        if ret:
            if frame.shape[2] == 4:  # If image is RGBA
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)
            else:
                frame_rgb = frame

            # Handle timestamp based on configuration
            if self.timestamp == "System_Time":
                image_timestamp = self.get_clock().now().to_msg()
            else:
                # Default to current ROS time if timestamp mode is not recognized
                image_timestamp = self.get_clock().now().to_msg()

            # Undistort image using camera parameters
            undistorted_img = cv2.undistort(frame_rgb, self.camera_k, self.camera_distortion)
            image_msg = self.bridge.cv2_to_imgmsg(undistorted_img, "bgr8")

            # Create and set header
            header = Header()
            header.frame_id = self.config['ROS']['frame_id']
            header.stamp = image_timestamp

            image_msg.header = header
            self.image_publisher.publish(image_msg)

            # Publish camera info
            self.camera_info.header.stamp = image_timestamp
            self.camera_info_publisher.publish(self.camera_info)
        else:
            self.get_logger().warn("Failed to capture frame from camera")

    def get_camera_info(self):
        self.camera_info.header.frame_id = self.config['ROS']['frame_id']
        
        self.camera_info.height = self.config['camera']['height']  
        self.camera_info.width = self.config['camera']['width'] 
        
        # Set intrinsic matrix K
        self.camera_info.k = self.config['intrinsics']['camera_matrix_K']
    
        # Distortion coefficients
        self.camera_info.d = self.config['intrinsics']['distortion']
        
        # Rectification matrix (identity for monocular cameras)
        self.camera_info.r = self.config['intrinsics']['rectification']
        
        # Projection matrix P
        self.camera_info.p = self.config['intrinsics']['projection']
        
        self.camera_info.distortion_model = "plumb_bob"
    
    def shutdown(self):
        if self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.shutdown()
        camera_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()