import yaml
import rclpy
from rclpy.node import Node
# import ouster
# import ouster.sensor as sen
import ouster.sdk.sensor as sensor
import ouster.sdk.core as core
from ouster.sdk import client
from ouster.sdk.core import LidarMode, TimestampMode
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


class OusterLidarPublisher(Node):
    def __init__(self, args=None):
        super().__init__("ouster_lidar_publisher")

        use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        self.get_logger().info(f'use_sim_time is set to: {use_sim_time}')
        
        # Note: Make sure your sensor supports the selected mode
        self.lidar_mode_map = {
            "512x10": LidarMode.MODE_512x10,
            "512x20": LidarMode.MODE_512x20,
            "1024x10":LidarMode.MODE_1024x10,
            "1024x20":LidarMode.MODE_1024x20,
            "2048x10":LidarMode.MODE_2048x10
        }
        
        timestamp_mode_map = {
            "TIME_FROM_INTERNAL_OSC":TimestampMode.TIME_FROM_INTERNAL_OSC,
            "TIME_FROM_PTP_1588": TimestampMode.TIME_FROM_PTP_1588,
            "TIME_FROM_SYNC_PULSE_IN": TimestampMode.TIME_FROM_SYNC_PULSE_IN,
        }
        

        # Load configuration
        config_path = self.load_yaml_file()
        with open(config_path, 'r') as config_file:
            self.config_file = yaml.safe_load(config_file)  

        # Set configs
        self.config = core.SensorConfig()
        self.config.udp_port_lidar = self.config_file['sensor']['udp_port_lidar']
        self.config.udp_port_imu = self.config_file['sensor']['udp_port_imu']
        self.hostname = self.config_file['sensor']['host_name']
        self.config.operating_mode = core.OperatingMode.OPERATING_NORMAL
        self.lidar_mode_str = self.config_file['lidar']['mode']
        self.timestamp_mode_str = self.config_file['lidar'].get('timestamp_mode', 'TIME_FROM_PTP_1588')
        self.timestamp_mode = timestamp_mode_map.get(self.timestamp_mode_str, TimestampMode.TIME_FROM_PTP_1588)
        
        # Set timestamp mode (required for phase lock) 
        self.config.timestamp_mode = self.timestamp_mode
        # Set Azimuth Window
        self.config.azimuth_window = self.config_file['lidar'].get('azimuth_window', [0, 360000])
        # Set Phase Lock -> Scan start point
        self.config.phase_lock_enable = self.config_file['lidar'].get('phase_lock_enable', True)
        self.config.phase_lock_offset = self.config_file['lidar'].get('phase_lock_offset', 180000)

        self.config.lidar_mode = self.lidar_mode_map.get( self.lidar_mode_str, LidarMode.MODE_1024x10)
        # Set FPS based on the lidar mode
        self.fps = int( self.lidar_mode_str.split('x')[1])
       
        # QoS profile
        qos_profile = rclpy.qos.QoSProfile(
            reliability=getattr(rclpy.qos.QoSReliabilityPolicy, self.config_file['qos']['reliability']),
            history=getattr(rclpy.qos.QoSHistoryPolicy, self.config_file['qos']['history']),
            depth=self.config_file['qos']['depth']
        )


        sensor.set_config(self.hostname, self.config, persist=True, udp_dest_auto=True)

        self.source = sensor._Sensor(self.hostname,  self.config)
        self.publisher = self.create_publisher(
            PointCloud2, 
            self.config_file['ROS']['topic_name'], 
            qos_profile # self.config_file['topic']['depth']
        )
        # self.create_timer(1.0 / self.fps, self.publish_pointcloud)
        self.metadata = self.source.fetch_metadata()

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
        
        self.get_logger().info(f"Sensor IP: {self.hostname}")
        self.get_logger().info(f"Data Destination: {self.config.udp_dest}")
        self.get_logger().info(f"Timestamp Mode: {self.config_file['lidar']['timestamp_mode']}")
    def get_timestamp_mode(self):
        try:
            info = core.get_config(self.hostname)
            return info.timestamp_mode
        except Exception as e:
            self.get_logger().error(f"Failed to get timestamp mode: {e}")
            return None
    def load_yaml_file(self):
        # Get the directory of the package's shared files
        package_share_directory = get_package_share_directory('sensors')
        
        # Construct the path to 'zed_config.yaml' in the 'config' directory
        config_file_path = os.path.join(package_share_directory, 'config', 'ouster_config.yaml')
        
        return config_file_path

    def publish_pointcloud(self):
        fields = [
            PointField(
                name='x', offset=0, datatype=PointField.FLOAT32, count=1
            ),
            PointField(
                name='y', offset=4, datatype=PointField.FLOAT32, count=1
            ),
            PointField(
                name='z', offset=8, datatype=PointField.FLOAT32, count=1
            ),
            PointField(
                name='intensity',
                offset=12,
                datatype=PointField.FLOAT32,
                count=1,
            ),
        ]

        with closing(
            sensor.SensorScanSource(
                self.hostname,
                lidar_port=self.config.udp_port_lidar,
            )
        ) as stream:
            metadata = stream.sensor_info[0]
            xyz_lut = core.XYZLut(metadata)
            warned_about_empty_scan = False
            warned_about_timestamp = False

            use_sensor_timestamp = (
                self.config_file['lidar']['timestamp_mode'] != 'System_Time'
            )
            if (
                use_sensor_timestamp
                and metadata.config.timestamp_mode
                == TimestampMode.TIME_FROM_INTERNAL_OSC
            ):
                self.get_logger().warning(
                    'LiDAR timestamps are relative to sensor power-on. '
                    'They cannot be synchronized directly with camera epoch '
                    'timestamps.'
                )

            for scans in stream:
                if not rclpy.ok():
                    break

                scan = scans[0]
                if scan is None:
                    continue

                ranges = scan.field(core.ChanField.RANGE)
                reflectivity = scan.field(core.ChanField.REFLECTIVITY)
                xyz = xyz_lut(scan)

                # LidarScan fields and XYZLut output are both in native
                # staggered order. Apply the same mask to all of them so each
                # XYZ point keeps the reflectivity measured by that pixel.
                valid_columns = (scan.status & 0x01) != 0
                valid_points = (
                    (ranges > 0)
                    & valid_columns[np.newaxis, :]
                    & np.isfinite(xyz).all(axis=-1)
                )

                if not np.any(valid_points):
                    if not warned_about_empty_scan:
                        self.get_logger().warning(
                            'Skipping LiDAR scan with no valid points.'
                        )
                        warned_about_empty_scan = True
                    continue

                warned_about_empty_scan = False
                xyz_points = xyz[valid_points].astype(np.float32, copy=False)
                intensity_values = reflectivity[valid_points].astype(
                    np.float32, copy=False
                )
                points = np.column_stack((xyz_points, intensity_values))

                header = Header()
                header.frame_id = self.config_file['lidar']['frame_id']

                if self.config_file['lidar']['timestamp_mode'] == 'System_Time':
                    header.stamp = self.get_clock().now().to_msg()
                else:
                    # A scan contains one timestamp per column. Use the middle
                    # of its valid acquisition interval as the single timestamp
                    # representing the complete PointCloud2 message.
                    valid_timestamps = scan.timestamp[
                        valid_columns & (scan.timestamp > 0)
                    ]
                    if valid_timestamps.size == 0:
                        if not warned_about_timestamp:
                            self.get_logger().warning(
                                'Skipping LiDAR scan with no valid timestamp.'
                            )
                            warned_about_timestamp = True
                        continue

                    warned_about_timestamp = False
                    first_timestamp = int(valid_timestamps.min())
                    last_timestamp = int(valid_timestamps.max())
                    lidar_time_ns = (
                        first_timestamp
                        + (last_timestamp - first_timestamp) // 2
                    )
                    header.stamp.sec = lidar_time_ns // 1_000_000_000
                    header.stamp.nanosec = lidar_time_ns % 1_000_000_000

                pointcloud = pc2.create_cloud(header, fields, points)
                self.publisher.publish(pointcloud)
def main(args=None):
    rclpy.init(args=args)
    # opt = parse_opt()
    ouster_lidar_publisher = OusterLidarPublisher()
    try:
        rclpy.spin(ouster_lidar_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        ouster_lidar_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
