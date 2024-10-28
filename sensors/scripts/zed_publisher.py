import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge
import pyzed.sl as sl
from std_msgs.msg import Header
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import yaml
import os
from ament_index_python.packages import get_package_share_directory
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSLivelinessPolicy

class ZEDCameraPublisher(Node):
    def __init__(self):
        super().__init__('zed_camera_publisher')

        # Now we can safely get and use the parameter
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
        self.camera_distortion =  np.array(self.config['intrinsics']['distortion'])

        self.zed = sl.Camera()
        self.bridge = CvBridge()

        # Initialize camera
        init_params = sl.InitParameters()
        init_params.set_from_camera_id(self.config['camera']['id'])
        init_params.camera_fps = self.config['camera']['fps']
        init_params.camera_resolution = getattr(sl.RESOLUTION, self.config['camera']['resolution'])
        init_params.depth_mode = getattr(sl.DEPTH_MODE, self.config['camera']['depth_mode'])

        # Open the ZED camera
        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f"Error opening ZED: {status}")
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
        self.camera_info_publisher = self.create_publisher(CameraInfo, self.config['ROS']['camera_info_topic'], 5)
        
        self.create_timer(1.0 /  self.config['camera']['fps'], self.publish_image)
        
        # self.publish_image()
    
    def load_yaml_file(self):
        # Get the directory of the package's shared files
        package_share_directory = get_package_share_directory('sensors')
        
        # Construct the path to 'zed_config.yaml' in the 'config' directory
        config_file_path = os.path.join(package_share_directory, 'config', 'zed_config.yaml')
        
        return config_file_path
    
    def publish_image(self):
        image = sl.Mat()
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(image, sl.VIEW.LEFT)
            frame = image.get_data()

            if frame.shape[2] == 4:
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)
            else:
                frame_rgb = frame

            if self.timestamp == sl.TIME_REFERENCE.IMAGE:
                image_timestamp  =  self.zed.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()
            elif self.timestamp == sl.TIME_REFERENCE.CURRENT:
                image_timestamp = self.zed.get_timestamp(sl.TIME_REFERENCE.CURRENT).get_nanoseconds()
            else:
                image_timestamp =  self.get_clock().now().to_msg()

            undistorted_img = cv2.undistort(frame_rgb, self.camera_k,  self.camera_distortion)
            image_msg = self.bridge.cv2_to_imgmsg(undistorted_img, "bgr8") 

            header = Header()
            header.frame_id = self.config['ROS']['frame_id']
            header.stamp = image_timestamp

            
            image_msg.header = header
            self.image_publisher.publish(image_msg)

            # Publish camera info
            self.camera_info.header.stamp = image_timestamp
            self.camera_info_publisher.publish(self.camera_info)
            # self.get_logger().info(f"Published image with timestamp: {header.stamp.sec}.{header.stamp.nanosec}")

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
        self.zed.close()

def main(args=None):
    rclpy.init(args=args)
    zed_camera_publisher = ZEDCameraPublisher()
    try:
        rclpy.spin(zed_camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        zed_camera_publisher.shutdown()
        zed_camera_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()