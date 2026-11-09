# como crear el paquete
cd ~/ros_workspace/src
ros2 pkg create practica2 --build-type ament_python --dependencies rclpy sensor_msgs cv_bridge


## como lanzar la camara

ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0 -p pixel_format:=mjpeg2rgb -p image_width:=640 -p image_height:=480 -p framerate:=15.0

## para ver la imagen con rqt
ros2 run rqt_image_view rqt_image_view /image_raw

## para correr el capturador imagenes
cd ~/Desktop/ros2_mucsi_26_27/ros_workspace
ros2 run practica2 capturador_imagenes --ros-args -p topic_imagen:=/image_raw
