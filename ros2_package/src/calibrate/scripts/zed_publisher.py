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
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSLivelinessPolicy
class ZEDCameraPublisher(Node):
    def __init__(self):
        super().__init__('zed_camera_publisher')
        #       # Check if the parameter is already declared
        # if not self.has_parameter('use_sim_time'):
        #     self.declare_parameter('use_sim_time', True)
        # else:
        #     print(self.has_parameter('use_sim_time'))
        #     self.get_logger().info(f"Already using use_sim_time ")
        

        # Now we can safely get and use the parameter
        use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        self.get_logger().info(f'use_sim_time is set to: {use_sim_time}')
        # Load configuration
        
        config_path = self.load_yaml_file()
        print("config_path: ",config_path)
        with open(config_path, 'r') as config_file:
            self.config = yaml.safe_load(config_file)

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

        # sensor_qos = QoSProfile(
        #     reliability=QoSReliabilityPolicy.BEST_EFFORT,
        #     durability=QoSDurabilityPolicy.VOLATILE,
        #     history=QoSHistoryPolicy.KEEP_LAST,
        #     depth=2,
        #     # liveliness=QoSLivelinessPolicy.AUTOMATIC,
        #     # deadline=rclpy.duration.Duration(seconds=0.1),
        #     # lifespan=rclpy.duration.Duration(seconds=0.5),
        # )

        # # QoS profile
        # qos_profile = QoSProfile(
        #     reliability=getattr(ReliabilityPolicy, self.config['qos']['reliability']),
        #     history=getattr(HistoryPolicy, self.config['qos']['history']),
        #     depth=self.config['qos']['depth']
        # )
        self.get_camera_info()
        # Publisher for ROS2 image topic
        self.publisher_ = self.create_publisher(
            Image, 
            self.config['topic']['name'], 
           5 # qos_profile
        )
        self.camera_info_publisher = self.create_publisher(CameraInfo, 'zed/camera_info', 5)
        
        self.create_timer(1.0 /  self.config['camera']['fps'], self.publish_image)
        # self.publish_image()
    def load_yaml_file(self):
        # Get the directory of the package's shared files
        package_share_directory = get_package_share_directory('calibrate')
        
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

            image_timestamp =  self.get_clock().now().to_msg()#self.zed.get_timestamp(sl.TIME_REFERENCE.IMAGE)

            image_msg = self.bridge.cv2_to_imgmsg(frame_rgb, "bgr8")

            header = Header()
            header.frame_id = self.config['frame_id']
            header.stamp = image_timestamp
            # header.stamp.sec = int(image_timestamp.get_seconds())
            # header.stamp.nanosec = int((image_timestamp.get_seconds() - int(image_timestamp.get_seconds())) * 1e9)

            image_msg.header = header

            self.publisher_.publish(image_msg)
            # Publish camera info
            self.camera_info.header.stamp = image_timestamp
            # self.camera_info.header.stamp.sec = int(image_timestamp.get_seconds())
            # self.camera_info.header.stamp.nanosec = int((image_timestamp.get_seconds() - int(image_timestamp.get_seconds())) * 1e9)
            self.camera_info_publisher.publish(self.camera_info)
            # self.get_logger().info(f"Published Zed Timestamp: {header.stamp.sec}.{header.stamp.nanosec}")
            self.get_logger().info(f"Published image with timestamp: {header.stamp.sec}.{header.stamp.nanosec}")

    def get_camera_info(self):
        self.camera_info = CameraInfo()
        self.camera_info.header.frame_id = "zed_camera_frame"
        
        # Use the provided values
        fx = 1153.806674
        fy = 1156.223082
        cx = 942.859434
        cy = 536.992462
        
        self.camera_info.height =1080 #self.zed.get_camera_information().camera_resolution.height
        self.camera_info.width = 1920 #self.zed.get_camera_information().camera_resolution.width
        
        # Intrinsic matrix K
        # self.camera_info.k = [fx,  0, cx,
        #                 0, fy, cy,
        #                 0,  0,  1]
        
        # Set intrinsic matrix K
        self.camera_info.k = [float(fx), 0.0, float(cx),
                      0.0, float(fy), float(cy),
                      0.0, 0.0, 1.0]

        
        # Distortion coefficients
        # Since we don't have specific distortion coefficients, we'll set them to zero
        self.camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Rectification matrix (identity for monocular cameras)
        self.camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        # Projection matrix P
        self.camera_info.p = [float(fx),  0.0,  float(cx), 0.0,
                      0.0,  float(fy),  float(cy), 0.0,
                      0.0,  0.0,  1.0,  0.0]

        

        
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