#!/usr/bin/env python3
"""Aplica la calibración de la cámara a las imágenes que llegan por ROS 2."""

from typing import List, Optional

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

TOPIC_ENTRADA_POR_DEFECTO = "/image_raw"
TOPIC_SALIDA = "/image_rect"
RUTA_CALIBRACION_POR_DEFECTO = "calibracion/ost.yaml"
TAMANO_COLA = 10
CODIFICACION = "bgr8"
VENTANA = "Original | Corregida"


class AplicadorCalibracion(Node):
    """Nodo que quita la distorsión de la cámara usando la calibración."""

    def __init__(self) -> None:
        """Carga el fichero de calibración y crea suscriptor y publicador."""
        super().__init__("aplicador_calibracion")

        self.declare_parameter("topic_imagen", TOPIC_ENTRADA_POR_DEFECTO)
        self.declare_parameter("ruta_calibracion", RUTA_CALIBRACION_POR_DEFECTO)
        topic: str = self.get_parameter("topic_imagen").value
        ruta: str = self.get_parameter("ruta_calibracion").value

        # Se lee el fichero que generó el calibrador (ost.yaml)
        with open(ruta, encoding="utf-8") as fichero:
            datos = yaml.safe_load(fichero)
        self.matriz: np.ndarray = np.array(datos["camera_matrix"]["data"]).reshape(3, 3)
        self.distorsion: np.ndarray = np.array(datos["distortion_coefficients"]["data"])

        # cv_bridge convierte entre imágenes de ROS y de OpenCV
        self.bridge = CvBridge()

        self.create_subscription(Image, topic, self._callback_imagen, TAMANO_COLA)
        self.publicador = self.create_publisher(Image, TOPIC_SALIDA, TAMANO_COLA)
        self.get_logger().info(f"Aplicando la calibración de {ruta} a {topic}")

    def _callback_imagen(self, mensaje: Image) -> None:
        """Se ejecuta con cada imagen: la corrige y la publica."""
        # 1. Imagen de ROS -> imagen de OpenCV (con cv_bridge)
        imagen: np.ndarray = self.bridge.imgmsg_to_cv2(mensaje, CODIFICACION)

        # 2. Se quita la distorsión con los parámetros de la calibración
        corregida: np.ndarray = cv2.undistort(imagen, self.matriz, self.distorsion)

        # 3. Imagen de OpenCV -> imagen de ROS (con cv_bridge) y se publica
        salida = self.bridge.cv2_to_imgmsg(corregida, CODIFICACION)
        salida.header = mensaje.header
        self.publicador.publish(salida)

        # 4. Se muestran la original y la corregida, una al lado de la otra
        cv2.imshow(VENTANA, np.hstack((imagen, corregida)))
        cv2.waitKey(1)


def main(args: Optional[List[str]] = None) -> None:
    """Arranca el nodo y lo mantiene girando hasta Ctrl+C."""
    rclpy.init(args=args)
    nodo = AplicadorCalibracion()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    nodo.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
