from setuptools import setup
import os
from glob import glob

setup(
    name='yolo_realsense',
    version='0.0.0',
    packages=['yolo_realsense'],
    
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/yolo_realsense']),

        ('share/yolo_realsense', ['package.xml']),

        (os.path.join('share', 'yolo_realsense', 'launch'), 
         glob('launch/*_launch.py')),
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ycheng22',
    maintainer_email='ycheng22@gmail.com',
    description='YOLO Object Detection with RealSense',
    license='Apache License 2.0',
    
    entry_points={
        'console_scripts': [
            'yolo_processor = yolo_realsense.yolo_processor:main',
        ],
    },
)