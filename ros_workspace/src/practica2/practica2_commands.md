cd ~/ros_workspace/src
ros2 pkg create practica2 --build-type ament_python --dependencies rclpy sensor_msgs cv_bridge


ros2 launch launcher_robots_lab_robotica launch_cam.launch.py cam_path:=/dev/video0


cd ~/ros_workspace
colcon build --symlink-install --packages-select practica2
source install/setup.bash
ros2 run practica2 capturador_imagenes
