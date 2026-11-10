#!/usr/bin/env python3
"""Captura N imágenes de la cámara que sean distintas entre sí."""

from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

TOPIC_POR_DEFECTO = "/image_raw"
NUMERO_IMAGENES_POR_DEFECTO = 10
# Diferencia media mínima (0-255) para considerar que dos imágenes son distintas
UMBRAL_DIFERENCIA_POR_DEFECTO = 10.0
CARPETA_POR_DEFECTO = "imagenes_capturadas"
TAMANO_COLA = 10
CODIFICACION = "bgr8"


class CapturadorImagenes(Node):
    """Nodo suscriptor que guarda imágenes distintas entre sí."""

    def __init__(self) -> None:
        """Lee los parámetros y se suscribe al topic de la cámara."""
        super().__init__("capturador_imagenes")

        self.declare_parameter("topic_imagen", TOPIC_POR_DEFECTO)
        self.declare_parameter("numero_imagenes", NUMERO_IMAGENES_POR_DEFECTO)
        self.declare_parameter("umbral_diferencia", UMBRAL_DIFERENCIA_POR_DEFECTO)
        self.declare_parameter("directorio_salida", CARPETA_POR_DEFECTO)
        topic: str = self.get_parameter("topic_imagen").value
        self.numero_imagenes: int = self.get_parameter("numero_imagenes").value
        self.umbral: float = self.get_parameter("umbral_diferencia").value
        self.carpeta = Path(self.get_parameter("directorio_salida").value)
        self.carpeta.mkdir(parents=True, exist_ok=True)

        # cv_bridge convierte entre imágenes de ROS y de OpenCV
        self.bridge = CvBridge()

        # Imágenes en gris de las ya guardadas, para comparar con las nuevas
        self.guardadas_gris: List[np.ndarray] = []
        self.terminado: bool = False

        self.create_subscription(Image, topic, self._callback_imagen, TAMANO_COLA)
        self.get_logger().info(f"Esperando {self.numero_imagenes} imágenes en {topic}")

    def _callback_imagen(self, mensaje: Image) -> None:
        """Se ejecuta con cada imagen: la guarda si es distinta a las anteriores."""
        if self.terminado:
            return

        # 1. Imagen de ROS -> imagen de OpenCV (con cv_bridge)
        imagen: np.ndarray = self.bridge.imgmsg_to_cv2(mensaje, CODIFICACION)
        gris = cv2.cvtColor(imagen, cv2.COLOR_BGR2GRAY)

        # 2. Si se parece a alguna ya guardada, se descarta.
        #    absdiff().mean() es cuánto cambia cada píxel de media.
        for otra in self.guardadas_gris:
            if cv2.absdiff(gris, otra).mean() <= self.umbral:
                return

        # 3. Es nueva: se guarda
        self.guardadas_gris.append(gris)
        numero = len(self.guardadas_gris)
        ruta = self.carpeta / f"imagen_{numero:02d}.png"
        cv2.imwrite(str(ruta), imagen)
        self.get_logger().info(f"Imagen {numero}/{self.numero_imagenes} guardada: {ruta}")

        if numero >= self.numero_imagenes:
            self.terminado = True
            self.get_logger().info("Ingesta completada")


def main(args: Optional[List[str]] = None) -> None:
    """Gira el nodo hasta tener todas las imágenes."""
    rclpy.init(args=args)
    nodo = CapturadorImagenes()
    try:
        while rclpy.ok() and not nodo.terminado:
            rclpy.spin_once(nodo, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    nodo.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
