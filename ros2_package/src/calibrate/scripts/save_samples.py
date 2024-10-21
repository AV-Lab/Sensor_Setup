import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
import cv2
from cv_bridge import CvBridge
import numpy as np
import os
import argparse
from sensor_msgs_py import point_cloud2
import open3d as o3d
import time
import sys

class SensorSyncSaverNode(Node):
    def __init__(self, args):
        super().__init__('sensor_sync_saver_node')

        # Now we can safely get and use the parameter
        use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        self.get_logger().info(f'use_sim_time is set to: {use_sim_time}')
        
        
        self.num_saves = args.num_saves
        self.image_folder = args.image_folder
        self.pcd_folder = args.pcd_folder
        self.delay_set = args.set_delay
        self.delay_frames = args.frame_delay
        self.set_size = args.set_size
        self.save_count = 0
        self.cv_bridge = CvBridge()
        
        # Create output directories
        os.makedirs(self.image_folder, exist_ok=True)
        os.makedirs(self.pcd_folder, exist_ok=True)
        
        # Create subscribers for LiDAR and camera topics
        lidar_sub = Subscriber(self, PointCloud2, '/ouster/lidar_points')
        camera_sub = Subscriber(self, Image, '/zed_image_raw')
        
        # Create approximate time synchronizer
        sync = ApproximateTimeSynchronizer(
            [lidar_sub, camera_sub],
            queue_size=10,
            slop=0.05  # 50ms time difference tolerance
        )
        sync.registerCallback(self.sync_callback)
    
    def print_data(self):
        self.get_logger().info("Save Node initialized with the following parameters:")
        self.get_logger().info(f"Number of saves: {self.num_saves}")
        self.get_logger().info(f"Image folder: {self.image_folder}")
        self.get_logger().info(f"PCD folder: {self.pcd_folder}")
        self.get_logger().info(f"Delay set: {self.delay_set}")
        self.get_logger().info(f"Delay frames: {self.delay_frames}")
        self.get_logger().info(f"Set size: {self.set_size}")
        self.get_logger().info(f"Use simulated time: {self.use_sim_time}")
    def sync_callback(self, lidar_msg, camera_msg):
        if self.save_count >= self.num_saves:
            self.get_logger().info('Finished saving all data pairs. Shutting down...')
            rclpy.shutdown()
            return

        # Save image
        cv_image = self.cv_bridge.imgmsg_to_cv2(camera_msg, desired_encoding='bgr8')
                    

        image_filename = f'{self.image_folder}/img_{self.save_count:04d}.png'
        cv2.imwrite(image_filename, cv_image)

        
        # Save point cloud data
        # Extract point cloud data from the PointCloud2 message
        points = []
        for point in point_cloud2.read_points(lidar_msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
            points.append([point[0], point[1], point[2], point[3]])  # Collect x, y, z, and intensity

        points = np.array(points, dtype=np.float32)
        num_points = points.shape[0]

        # Create Open3D point cloud object
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])  # XYZ

        # If you want to include intensity as colors (in red channel)
        intensities = points[:, 3]
        colors = np.zeros((num_points, 3))  # Create zero array for RGB
        colors[:, 0] = intensities / np.max(intensities)  # Normalize intensity to 0-1 and map to red channel
        pcd.colors = o3d.utility.Vector3dVector(colors)  # Set as color


        pcd_filename =f'{self.image_folder}/pc_{self.save_count:04d}.pcd'
        # Save the point cloud as a .pcd file
        o3d.io.write_point_cloud(pcd_filename, pcd)
        # self.save_pointcloud2(lidar_msg, pcd_filename)
        
        self.get_logger().info(f'Saved synchronized data pair {self.save_count + 1}/{self.num_saves}')
        self.save_count += 1
        if  self.save_count%self.set_size==0:
            print(f"Delay {self.delay_set} Sec!")
            time.sleep(self.delay_set)
            print(f"Delay Finished!")
        else:
            time.sleep(self.delay_frames)

    def save_pointcloud2(self, cloud_msg, filename):
        # Convert PointCloud2 to numpy array
        points = np.frombuffer(cloud_msg.data, dtype=np.float32).reshape(-1, 4)
        
        # Save as binary file
        points.tofile(filename)

def main(args=None):
    rclpy.init(args=args)
    

    # Set up argparse
    parser = argparse.ArgumentParser(description='Save synchronized LiDAR and camera data')
    parser.add_argument('num_saves', type=int,  default=50,help='Number of data pairs to save')
    parser.add_argument('image_folder', type=str,default='./images', help='image folder name')
    parser.add_argument('pcd_folder', type=str, default='./pcds',help='pcd folder name')
    parser.add_argument('--set_size', type=int, default=10, help='Number of frames per set')
    parser.add_argument('--set_delay', type=float, default=10.0, help='Delay between sets in seconds')
    parser.add_argument('--frame_delay', type=float, default=1.0, help='Delay between frames in seconds')
    
    # Parse known args
    parsed_args, ros_args = parser.parse_known_args()

    node = SensorSyncSaverNode(parsed_args) #--ros-args --remap use_sim_time:=false
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
