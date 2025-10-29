from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'yolo_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files if you create them later
        # (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include config files if you create them later
        # (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'], # Add external Python deps: e.g., 'ultralytics', 'opencv-python'
    # install_requires=['setuptools', 'ultralytics', 'opencv-python'], # Example
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS 2 Node for YOLOv8 Object Detection and Simple Decision Making',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_node = yolo_detector.yolo_node:main',
            'decision_maker = yolo_detector.decision_maker:main',
            # 'video_publisher = yolo_detector.video_publisher_node:main', # Keep if needed
        ],
    },
)
