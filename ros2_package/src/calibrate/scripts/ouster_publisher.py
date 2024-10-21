import yaml
import rclpy
from rclpy.node import Node
from ouster import client
from contextlib import closing
from std_msgs.msg import Header
import numpy as np
from pathlib import Path
import argparse
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
import os
import sys
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSLivelinessPolicy
# # # PATH 
# FILE = Path(__file__).resolve()
# ROOT = FILE.parents[0]  
# if str(ROOT) not in sys.path:
#     sys.path.append(str(ROOT))  # add ROOT to PATH
# ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

class OusterLidarPublisher(Node):
    def __init__(self, args=None):
        super().__init__("ouster_lidar_publisher")
        # sensor_qos = QoSProfile(
        #     reliability=QoSReliabilityPolicy.BEST_EFFORT,
        #     durability=QoSDurabilityPolicy.VOLATILE,
        #     history=QoSHistoryPolicy.KEEP_LAST,
        #     depth=2,
        #     # liveliness=QoSLivelinessPolicy.AUTOMATIC,
        #     # deadline=rclpy.duration.Duration(seconds=0.1),
        #     # lifespan=rclpy.duration.Duration(seconds=0.5),
        # )


        use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        self.get_logger().info(f'use_sim_time is set to: {use_sim_time}')
        
        # Note: Make sure your sensor supports the selected mode
        self.lidar_mode_map = {
            "512x10": client.LidarMode.MODE_512x10,
            "512x20": client.LidarMode.MODE_512x20,
            "1024x10": client.LidarMode.MODE_1024x10,
            "1024x20": client.LidarMode.MODE_1024x20,
            "2048x10": client.LidarMode.MODE_2048x10
        }
        
        timestamp_mode_map = {
            "TIME_FROM_INTERNAL_OSC": client.TimestampMode.TIME_FROM_INTERNAL_OSC,
            "TIME_FROM_PTP_1588": client.TimestampMode.TIME_FROM_PTP_1588,
            "TIME_FROM_SYNC_PULSE_IN": client.TimestampMode.TIME_FROM_SYNC_PULSE_IN,
            "System_Time"   : "system time" 
        }
        
        # # Load configuration
        # config_path = args.config_path if args else "ouster_config.yaml"
        # with open(config_path, "r") as f:
        #     self.config_file = yaml.safe_load(f)


        config_path = self.load_yaml_file()
        print("config_path: ",config_path)
        with open(config_path, 'r') as config_file:
            self.config_file = yaml.safe_load(config_file)  

        # Set configs
        self.config = client.SensorConfig()
        self.config.udp_port_lidar = self.config_file['sensor']['udp_port_lidar']
        self.config.udp_port_imu = self.config_file['sensor']['udp_port_imu']
        self.hostname = self.config_file['sensor']['host_name']
        self.config.operating_mode = client.OperatingMode.OPERATING_NORMAL
        self.lidar_mode_str = self.config_file['lidar']['mode']

        # Set timestamp mode (required for phase lock)
        self.timestamp_mode_str = self.config_file['lidar'].get('timestamp_mode', 'TIME_FROM_PTP_1588')
      
        # self.config.timestamp_mode = 'system'#timestamp_mode_map.get(self.timestamp_mode_str, client.TimestampMode.TIME_FROM_PTP_1588)
                # Set Azimuth Window
        self.config.azimuth_window = self.config_file['lidar'].get('azimuth_window', [0, 360000])
        # Set Phase Lock -> Scan start point
        self.config.phase_lock_enable = self.config_file['lidar'].get('phase_lock_enable', True)
        self.config.phase_lock_offset = self.config_file['lidar'].get('phase_lock_offset', 180000)

        self.config.lidar_mode = self.lidar_mode_map.get( self.lidar_mode_str, client.LidarMode.MODE_1024x10)
        # Set FPS based on the lidar mode
        self.fps = int( self.lidar_mode_str.split('x')[1])
        # print(f" ---------Lidar mode { self.lidar_mode_str } @ {self.fps} Hz! ")
        # QoS profile
        qos_profile = rclpy.qos.QoSProfile(
            reliability=getattr(rclpy.qos.QoSReliabilityPolicy, self.config_file['qos']['reliability']),
            history=getattr(rclpy.qos.QoSHistoryPolicy, self.config_file['qos']['history']),
            depth=self.config_file['qos']['depth']
        )


        client.set_config(self.hostname, self.config, persist=True, udp_dest_auto=True)

        self.source = client.Sensor(self.hostname, self.config.udp_port_lidar, self.config.udp_port_imu)
        self.publisher = self.create_publisher(
            PointCloud2, 
            self.config_file['topic']['name'], 
            5
        )
        # self.create_timer(1.0 / self.fps, self.publish_pointcloud)
        self.metadata = self.source.metadata
        # self.get_logger().info(f"Lidar Mode : {self.source.metadata.mode}")

        # Get the timestamp mode
        self.timestamp_mode = self.get_timestamp_mode()
        # self.get_logger().info(f"Timestamp mode: {self.timestamp_mode}")
        


        # set up details print
        print_mode =  self.config_file['lidar']['info_mode']
        self.print_lidar_setup(print_mode)
        
        self.publish_pointcloud()

       

    def print_lidar_setup(self, mode='minimal'):
        if mode == 'minimal':
            self.print_minimal_setup()
        elif mode == 'detailed':
            self.print_detailed_setup()
        else:
            self.get_logger().warning(f"Unknown print mode: {mode}. Defaulting to minimal.")
            self.print_minimal_setup()

    def print_minimal_setup(self):
        self.get_logger().info("LiDAR Minimal Setup Details:")
        self.get_logger().info(f"Hostname: {self.hostname}")
        self.get_logger().info(f"LiDAR Mode: {self.config.lidar_mode}")
        self.get_logger().info(f"FPS: {self.fps}")
        self.get_logger().info(f"Azimuth Window: {self.config.azimuth_window}")
        self.get_logger().info(f"Phase Lock: {'Enabled' if self.config.phase_lock_enable else 'Disabled'}")
        if self.config.phase_lock_enable:
            self.get_logger().info(f"Phase Lock Offset: {self.config.phase_lock_offset}")

    def print_detailed_setup(self):
        self.get_logger().info("LiDAR Detailed Setup:")
        self.get_logger().info(f"Hostname: {self.hostname}")
        self.get_logger().info(f"UDP Port (LiDAR): {self.config.udp_port_lidar}")
        self.get_logger().info(f"UDP Port (IMU): {self.config.udp_port_imu}")
        self.get_logger().info(f"LiDAR Mode: {self.config.lidar_mode}")
        self.get_logger().info(f"FPS: {self.fps}")
        self.get_logger().info(f"Azimuth Window: {self.config.azimuth_window}")
        self.get_logger().info(f"Phase Lock Enabled: {self.config.phase_lock_enable}")
        self.get_logger().info(f"Phase Lock Offset: {self.config.phase_lock_offset}")
        self.get_logger().info(f"Operating Mode: {self.config.operating_mode}")
        
        self.get_logger().info(f"Sensor Model: {self.metadata.prod_line}")
        self.get_logger().info(f"Serial Number: {self.metadata.sn}")
        self.get_logger().info(f"Firmware Version: {self.metadata.fw_rev}")
        # self.get_logger().info(f"Calibration Status: {'Valid' if self.metadata.cal_status else 'Invalid'}")
        
        # beam_altitude_angles = client.get_beam_intrinsics(self.metadata).altitude_angles
        # self.get_logger().info(f"Number of channels: {len(beam_altitude_angles)}")
        # self.get_logger().info(f"Vertical FOV: {max(beam_altitude_angles) - min(beam_altitude_angles):.2f} degrees")
        
        self.get_logger().info(f"Sensor IP: {self.metadata.hostname}")
        self.get_logger().info(f"Data Destination: {self.config.udp_dest}")
        self.get_logger().info(f"Timestamp Mode: {self.timestamp_mode}")
    def get_timestamp_mode(self):
        try:
            info = client.get_config(self.hostname)
            return info.timestamp_mode
        except Exception as e:
            self.get_logger().error(f"Failed to get timestamp mode: {e}")
            return None
    def load_yaml_file(self):
        # Get the directory of the package's shared files
        package_share_directory = get_package_share_directory('calibrate')
        
        # Construct the path to 'zed_config.yaml' in the 'config' directory
        config_file_path = os.path.join(package_share_directory, 'config', 'ouster_config.yaml')
        
        return config_file_path
    def publish_pointcloud(self):
        with closing(client.Scans(self.source)) as scans:
            self.get_logger().info(f"Scan opened: {self.timestamp_mode}")

            for scan in scans:
                # Read XYZ and intensity fields from the scan
                xyz = client.XYZLut(self.metadata)(scan)  # XYZ points
                intensity = scan.field(client.ChanField.REFLECTIVITY)  # Intensity field

                # Reshape XYZ and intensity fields
                xyz_points = xyz.reshape(-1, 3)  # Reshape to (N, 3) for x, y, z
                intensity_values = intensity.reshape(-1, 1)  # Reshape to (N, 1) for intensity

                # Combine XYZ and intensity into a single array (x, y, z, intensity)
                points_list = np.hstack((xyz_points, intensity_values))  # Shape (N, 4)

                # Create a header for the PointCloud2 message
                header = Header()
                header.frame_id = self.config_file['lidar']['frame_id']

                # Set the timestamp based on the LiDAR's timestamp mode
                # if self.timestamp_mode in [client.TimestampMode.TIME_FROM_PTP_1588, client.TimestampMode.TIME_FROM_SYNC_PULSE_IN]:
                #     # Use the LiDAR's internal timestamp
                #     lidar_time = scan.timestamp
                #     # print(f"lidar_time: {lidar_time}")
                #     header.stamp.sec = int(lidar_time // 1_000_000_000)
                #     header.stamp.nanosec = int(lidar_time % 1_000_000_000)
                # else:
                # Use the current ROS time or system time if not using PTP or sync pulse
                header.stamp = self.get_clock().now().to_msg()

                # Define the fields (x, y, z, intensity) for the PointCloud2 message
                fields = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
                ]

                # Create the PointCloud2 message
                pc2_msg = pc2.create_cloud(header, fields, points_list)

                # Publish the message
                self.publisher.publish(pc2_msg)
                self.get_logger().info(f"Published {len(points_list)} points with intensity. Timestamp: {header.stamp.sec}.{header.stamp.nanosec} {self.timestamp_mode}")
                
                # Only publish one scan per timer callback
                # break

# def parse_opt():
#     parser = argparse.ArgumentParser()
#     parser.add_argument('--config_path', type=str, default=ROOT / 'ouster_config.yaml', help='path to config file')
#     return parser.parse_args()

def main(args=None):
    rclpy.init(args=args)
    # opt = parse_opt()
    ouster_lidar_publisher = OusterLidarPublisher()#OusterLidarPublisher(opt)
    try:
        rclpy.spin(ouster_lidar_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        ouster_lidar_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()