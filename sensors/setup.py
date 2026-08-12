from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sensors'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # Install the entire config folder
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        # Install the entire launch folder
        # Install all Python launch descriptions.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='avlab_mz',
    maintainer_email='muradsmebrahtu@gmail.com',
    description='ROS2 Ouster/See3CAM capture and calibration utilities',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'clock_node = scripts.clock:main',
            'ouster_node = scripts.ouster_publisher:main',
            'zed_node = scripts.zed_publisher:main',
            'camera_node = scripts.camera_publisher:main',
            'save_node = scripts.save_samples:main',
            'calibration_audit = scripts.calibration_audit:main',
            'calibration_review = scripts.interactive_calibration:main',
            'calibration_tf = scripts.calibration_tf_publisher:main',
        ],
    },
)
