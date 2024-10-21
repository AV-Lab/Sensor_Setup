from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'calibrate'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # Install the entire config folder
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        # Install the entire launch folder
        # (os.path.join('share', package_name, 'launch'), ['launch/launch_all.py']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='avlab_mz',
    maintainer_email='muradsmebrahtu@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'clock_node = scripts.clock:main',  # 
            'ouster_node = scripts.ouster_publisher:main',  # Adjust if you have a main function
            'zed_node = scripts.zed_publisher:main',  # Adjust if you have a main function
            'save_node = scripts.save_samples:main',
        ],
    },
)
