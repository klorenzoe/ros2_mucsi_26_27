# Práctica 2: ingesta, calibración y procesamiento de imágenes

Paquete de ROS 2 con dos nodos:

- `capturador_imagenes`: guarda 10 imágenes distintas de la cámara.
- `aplicador_calibracion`: corrige la imagen con la calibración de la cámara (usando `cv_bridge`).

## Dependencias

```bash
sudo apt install -y ros-lyrical-camera-calibration
```

Con ROS 2 Lyrical, `camera_calibration` falla por dos bugs del paquete (`warn` y `tostring`, que ya no existen en `rclpy` y NumPy). Se corrigen una vez con:

```bash
sudo sed -i 's/get_logger().warn(/get_logger().warning(/' \
  /opt/ros/lyrical/lib/python3.14/site-packages/camera_calibration/camera_calibrator.py
sudo sed -i 's/\.tostring()/.tobytes()/' \
  /opt/ros/lyrical/lib/python3.14/site-packages/camera_calibration/mono_calibrator.py
```

## Preparación

Compilar el paquete y cargar el entorno (en cada terminal nueva):

```bash
cd ~/Desktop/ros2_mucsi_26_27/ros_workspace
colcon build --packages-select practica2
source install/setup.bash
```

## 1. Lanzar la cámara

```bash
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0 -p pixel_format:=mjpeg2rgb -p image_width:=640 -p image_height:=480 -p framerate:=15.0
```

Para ver la imagen:

```bash
ros2 run rqt_image_view rqt_image_view /image_raw
```

## 2. Capturar imágenes

Guarda 10 imágenes distintas en `imagenes_capturadas/` (hay que mover la cámara o la escena entre capturas).

```bash
cd ~/Desktop/ros2_mucsi_26_27/ros_workspace
ros2 run practica2 capturador_imagenes
```

## 3. Calibrar la cámara

Tablero de 8x6 esquinas interiores con cuadros de 25 mm:

```bash
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 --no-service-check --ros-args -r image:=/image_raw
```

Al pulsar **SAVE** se guarda `/tmp/calibrationdata.tar.gz`. Extraer el `ost.yaml` dentro del paquete:

```bash
cd ~/Desktop/ros2_mucsi_26_27/ros_workspace/src/practica2
mkdir -p calibracion
tar -xzf /tmp/calibrationdata.tar.gz -C calibracion ost.yaml
```

## 4. Aplicar la calibración

Usa `calibracion/ost.yaml` por defecto, así que hay que lanzarlo desde la carpeta del paquete:

```bash
cd ~/Desktop/ros2_mucsi_26_27/ros_workspace/src/practica2
ros2 run practica2 aplicador_calibracion
```

Muestra la imagen original y la corregida, y publica la corregida en `/image_rect`.

## Cómo se creó el paquete

```bash
cd ~/Desktop/ros2_mucsi_26_27/ros_workspace/src
ros2 pkg create practica2 --build-type ament_python --dependencies rclpy sensor_msgs cv_bridge
```
