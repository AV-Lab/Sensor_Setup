import rclpy
from rclpy.node import Node
import pyzed.sl as sl
import cv2
import yaml
import os
import open3d as o3d
from ament_index_python.packages import get_package_share_directory
import numpy as np
from datetime import datetime

class InteractiveCalibration(Node):
    def __init__(self):
        super().__init__('InteractiveCalibration')
        # Load configuration
        config_path = self.load_yaml_file()
        with open(config_path, 'r') as config_file:
            self.config = yaml.safe_load(config_file)

        self.Tr_lidar_to_cam = np.array(self.config['transform']['lidar_camera']).reshape(-1,4)
        self.K = np.array(self.config['transform']['intrinsic_k']).reshape(-1,3)
        self.img_folder = self.config['path']['img_folder']
        self.pcd_folder = self.config['path']['pcd_folder']

        self.process_samples()
    
    def load_yaml_file(self):
        # Get the directory of the package's shared files
        package_share_directory = get_package_share_directory('sensors')
        
        # Construct the path to 'zed_config.yaml' in the 'config' directory
        config_file_path = os.path.join(package_share_directory, 'config', 'calibrate.yaml')
        
        return config_file_path
    
    def load_pcd(self,file_path):
        pcd = o3d.io.read_point_cloud(file_path)
        points = np.asarray(pcd.points)
        # Add homogeneous coordinate
        points = np.hstack((points, np.ones((points.shape[0], 1))))
        return points

    def project_points_to_image(self,points, K, Tr_lidar_to_cam):
        # Transform points from lidar to camera coordinate
        points_cam = Tr_lidar_to_cam @ points.T
        
        # Project to image plane
        points_2d = K @ points_cam[:3, :]
        points_2d = points_2d[:2, :] / points_2d[2, :]
        
        return points_2d.T

    def overlay_points_on_image(self,image, points_2d):
        overlay = image.copy()
        for (u, v) in points_2d:
            if 0 <= u < image.shape[1] and 0 <= v < image.shape[0]:
                cv2.circle(overlay, (int(u), int(v)), 4, (0, 255, 0), -1)
        return cv2.addWeighted(overlay, 0.5, image, 0.5, 0)

    def adjust_transformation(self,Tr_lidar_to_cam, adjustment, value):
        adjustment_matrix = np.eye(4)
        if adjustment in ['r', 'l', 'u', 'd']:
            axis = {'r': 0, 'l': 0, 'u': 1, 'd': 1}[adjustment]
            sign = 1 if adjustment in ['r', 'u'] else -1
            adjustment_matrix[axis, 3] = sign * value
        elif adjustment in ['rl', 'rr', 'ru', 'rd']:
            axis = {'rl': 2, 'rr': 2, 'ru': 1, 'rd': 0}[adjustment]
            angle = value if adjustment in ['rr', 'rd'] else -value
            c, s = np.cos(angle), np.sin(angle)
            if axis == 0:
                adjustment_matrix[:3, :3] = [[1, 0, 0], [0, c, -s], [0, s, c]]
            elif axis == 1:
                adjustment_matrix[:3, :3] = [[c, 0, s], [0, 1, 0], [-s, 0, c]]
            else:
                adjustment_matrix[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
        return adjustment_matrix @ Tr_lidar_to_cam

    def update_display(self,image, points, Tr_lidar_to_cam):
        points_2d = self.project_points_to_image(points, self.K, Tr_lidar_to_cam)
        result = self.overlay_points_on_image(image, points_2d)
        cv2.imshow('LiDAR Overlay', result)
        cv2.waitKey(1)

    def save_transformation(self,Tr_lidar_to_cam, pair_number, filename='transformations.yaml'):
        # Load existing transformations
        if os.path.exists(filename):
            with open(filename, 'r') as file:
                data = yaml.safe_load(file) or {}
        else:
            data = {}

        # Create a new entry for this transformation
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        new_entry = {
            f"transformation_{timestamp}": {
                "pair_number": pair_number,
                "Tr_lidar_to_cam": Tr_lidar_to_cam.tolist()
            }
        }

        # Append the new entry
        data.update(new_entry)

        # Save the updated data
        with open(filename, 'w') as file:
            yaml.dump(data, file)
        print(f"Transformation for pair {pair_number} saved to {filename}")

    def process_pair(self,image_path, pcd_path, Tr_lidar_to_cam, pair_number):
        image = cv2.imread(image_path)
        points = self.load_pcd(pcd_path)
        
        # Set the window size to match the image size
        cv2.namedWindow('LiDAR Overlay', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('LiDAR Overlay', image.shape[1], image.shape[0])
        
        # Initial display
        self.update_display(image, points, Tr_lidar_to_cam)
        
        while True:
            choice = input("Adjust [r,l,u,d,rl,rr,ru,rd], save (s), or next (n)? ").lower()
            
            if choice == 's':
                print("New Tr_lidar_to_cam:")
                print(Tr_lidar_to_cam)
                self.save_transformation(Tr_lidar_to_cam, pair_number)
            elif choice == 'n':
                print("Tr_lidar_to_cam:")
                print(Tr_lidar_to_cam)
                break
            elif choice in ['r', 'l', 'u', 'd', 'rl', 'rr', 'ru', 'rd']:
                value = float(input(f"Enter value for {choice}: "))
                Tr_lidar_to_cam = self.adjust_transformation(Tr_lidar_to_cam, choice, value)
                print("New Tr_lidar_to_cam:")
                print(Tr_lidar_to_cam)
                self.update_display(image, points, Tr_lidar_to_cam)
            else:
                print("Invalid choice. Please try again.")
        
        return Tr_lidar_to_cam
    
    def process_samples(self):
        """
        Process all image-pointcloud pairs from self.img_folder and self.pcd_folder
        """
        blank_image = None
        first_image = True

        # Ensure folders exist
        if not os.path.exists(self.img_folder) or not os.path.exists(self.pcd_folder):
            print(f"Error: One or both folders do not exist!")
            print(f"Image folder: {self.img_folder}")
            print(f"PCD folder: {self.pcd_folder}")
            return

        # Get list of all files in both directories
        image_files = sorted([f for f in os.listdir(self.img_folder) if f.startswith('img_') and f.endswith('.png')])
        pcd_files = sorted([f for f in os.listdir(self.pcd_folder) if f.startswith('pc_') and f.endswith('.pcd')])

        # Verify we have matching pairs
        num_pairs = min(len(image_files), len(pcd_files))
        if num_pairs == 0:
            print("No matching pairs found in the directories!")
            print(f"Images found: {len(image_files)}")
            print(f"PCDs found: {len(pcd_files)}")
            return

        print(f"Found {num_pairs} pairs to process")

        # Process each pair
        for i in range(num_pairs):
            image_path = os.path.join(self.img_folder, image_files[i])
            pcd_path = os.path.join(self.pcd_folder, pcd_files[i])

            try:
                print(f"Processing pair {i+1}/{num_pairs}: {image_files[i]} - {pcd_files[i]}")
                self.Tr_lidar_to_cam = self.process_pair(image_path, pcd_path, self.Tr_lidar_to_cam, i)
                
                # Create blank image for visualization on first successful pair
                if first_image:
                    first_image = False
                    img = cv2.imread(image_path)
                    if img is not None:
                        blank_image = np.zeros(img.shape, np.uint8)
                    else:
                        print(f"Warning: Could not read image {image_path}")
            except Exception as e:
                print(f"Error processing pair {i+1}/{num_pairs}: {str(e)}")
                if blank_image is not None:
                    cv2.imshow('LiDAR Overlay', blank_image)
                    cv2.waitKey(1)
                continue

        print("\nProcessing complete!")
        print("\nFinal transformation matrix:")
        print(self.Tr_lidar_to_cam)
        print("\nPress any key in the image window to exit.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    
    def shutdown(self):
        self.zed.close()

def main(args=None):
    rclpy.init(args=args)
    interactive_calibration = InteractiveCalibration()
    try:
        rclpy.spin(interactive_calibration)
    except KeyboardInterrupt:
        pass
    finally:
        interactive_calibration.shutdown()
        interactive_calibration.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()