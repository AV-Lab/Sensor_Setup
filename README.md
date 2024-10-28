# Sensor_Setup
ROS2 package to configure, test and start sensors : Camera and Lidar ( zed2 camera and ouster lidar)

# Quick start
- Build and source  
    ``` 
    cd ros2_package  && colcon build
    ```  
    ```
    source install/setup.bash 
    ```
- To start both sensors with one launch file use the following command:
    ```
    ros2 launch calibrate launch_all_sensors.py 
    ```
- To start ouster lidar:   
    ```
    ros2 run calibrate ouster_node --ros-args --remap use_sim_time:=false
    ```


- To run zed camera:  
    ```
    ros2 run calibrate zed_node --ros-args --remap use_sim_time:=false  
    ```


- To Save synchronized samples from rosbag  
    ``` 
    ros2 run calibrate save_node   100 'images_x' 'pcds_x' 10 10 1 --ros-args -p use_sim_time:=true
    ```
 
- To Save synchronized samples from sensors (real-time)  
    ```
    ros2 run calibrate save_node   100 'images_x' 'pcds_x' 10 10 1 --ros-args -p use_sim_time:=false
    ```
 

    ros2 run calibrate save_node   --num_saves --image_folder pcd_folder --set_size  --set_delay --frame_delay --ros-args -p use_sim_time:=true 


# Notes
## Frame Orientations
- Based on the calibration:
   - Ouster lidar [Right hand Rule]
      -  X - forward (depth)
      -  Y - left 
      -  Z - Up
   - Camera frame -> same as image CV2
      -  X - right
      -  Y - down
      -  Z - forward (depth)

## Steps for Consistent Timestamps in ROS2 Rosbags
 - If you plan to record a rosbag and to replay it with the same timestamp follow the following instructions:

### Recording
 
1. Set `use_sim_time` to false for all nodes:
   ```bash
   ros2 param set /your_node use_sim_time false
   ```
   (Repeat for each node, or set in launch files)
    
- add the following at the end of node launch to use system time:  
   ``` 
   ros2 run calibrate node_to_run  --ros-args -p use_sim_time:=False  
   ```
 
2. Record the rosbag:
   ```bash
   ros2 bag record -a -o my_rosbag
   ```
   The `-a` flag records all topics.
 
### Playback
 
1. Set `use_sim_time` to true for all nodes that will receive played back data:
   ```bash
   ros2 param set /your_node use_sim_time true
   ```
   (Repeat for each node, or set in launch files)
   
   add the following at the end of node launch to use sim_time:  
   ``` 
   ros2 run calibrate save_node  --ros-args -p use_sim_time:=true  
   ```
 
2. Play the rosbag with clock publishing:
   ```bash
   ros2 bag play my_rosbag --clock 100
   ```
   The `--clock` option publishes on the `/clock` topic at 100 Hz.


 
### Important Notes
 
- Ensure all nodes are using the same time source during recording and playback.

- For nodes created after bag playback starts, set ` use_sim_time ` parameter in the node's constructor.

- If using launch files, set ` use_sim_time ` parameter for each node in the launch file.

- Verify time settings with ` ros2 param get /your_node use_sim_time `.


# Zed Camera
- In this setup the Zed camera is used a monocular camera (left).
- For this reason the rectification is set to identity:
```
self.camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
```
- To update the camera intrinsic parameters use the  [zed](Sensor_Setup/ros2_package/src/calibrate/config/zed_intrinsic.yaml)