Object detection using YOLOV11n in ROS2. Requires ROS2 Humble and Realsense Wrapper for ROS2.

https://docs.ultralytics.com/models/yolo11/  
https://github.com/IntelRealSense/realsense-ros  
https://docs.ros.org/en/humble/index.html  

1. Initialize RealSense node
ros2 launch yolo_realsense realsense_launch.py

2. Run YOLO inference
ros2 run yolo_realsense yolo_processor

3. Open RVIZ2
ros2 rviz2

4. Load default configuration in RVIZ2
