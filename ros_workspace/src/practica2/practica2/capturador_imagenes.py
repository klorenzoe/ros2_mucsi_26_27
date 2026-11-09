#!/usr/bin/env python3
"""Nodo de ingesta: captura N imágenes consecutivas distintas entre sí."""

from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

NOMBRE_NODO = "capturador_imagenes"
TOPIC_IMAGEN_POR_DEFECTO = "/usb_cam/image_raw"
NUMERO_IMAGENES_POR_DEFECTO = 10
# Un poco grande para evitar que el ruido de la cámara cuente como cambio.
UMBRAL_DIFERENCIA_POR_DEFECTO = 10.0
DIRECTORIO_SALIDA_POR_DEFECTO = "imagenes_capturadas"
TAMANO_COLA_SUSCRIPTOR = 10
CODIFICACION_OPENCV = "bgr8"


class CapturadorImagenes(Node):
    """Suscriptor que almacena N imágenes distintas entre sí.

    Cada imagen recibida se compara con todas las ya guardadas; solo se
    almacena si difiere de todas ellas por encima de un umbral. Al reunir
    el número pedido, el nodo marca ``terminado`` como ``True``.
    """

    def __init__(self) -> None:
        super().__init__(NOMBRE_NODO)
        self.declare_parameter("topic_imagen", TOPIC_IMAGEN_POR_DEFECTO)
        self.declare_parameter("numero_imagenes", NUMERO_IMAGENES_POR_DEFECTO)
        self.declare_parameter("umbral_diferencia", UMBRAL_DIFERENCIA_POR_DEFECTO)
        self.declare_parameter("directorio_salida", DIRECTORIO_SALIDA_POR_DEFECTO)

        self._numero_objetivo: int = self.get_parameter("numero_imagenes").value
        self._umbral_diferencia: float = self.get_parameter("umbral_diferencia").value
        self._directorio_salida = Path(self.get_parameter("directorio_salida").value)
        self._directorio_salida.mkdir(parents=True, exist_ok=True)

        self._bridge = CvBridge()
        # Versiones en gris de las imágenes guardadas, usadas solo para comparar.
        self._referencias_gris: List[np.ndarray] = []
        self.terminado: bool = False

        topic: str = self.get_parameter("topic_imagen").value
        self.create_subscription(Image, topic, self._callback_imagen, TAMANO_COLA_SUSCRIPTOR)
        self.get_logger().info(
            f"Esperando {self._numero_objetivo} imágenes distintas en '{topic}'"
        )

    def _callback_imagen(self, mensaje: Image) -> None:
        """Convierte el mensaje con cv_bridge y lo guarda si es nuevo."""
        if self.terminado:
            return

        imagen: np.ndarray = self._bridge.imgmsg_to_cv2(
            mensaje, desired_encoding=CODIFICACION_OPENCV
        )
        imagen_gris = cv2.cvtColor(imagen, cv2.COLOR_BGR2GRAY)

        if self._es_distinta_de_las_guardadas(imagen_gris):
            self._guardar(imagen, imagen_gris)

    def _es_distinta_de_las_guardadas(self, imagen_gris: np.ndarray) -> bool:
        """Indica si la imagen difiere de todas las almacenadas hasta ahora."""
        return all(
            self._diferencia_media(imagen_gris, referencia) > self._umbral_diferencia
            for referencia in self._referencias_gris
        )

    @staticmethod
    def _diferencia_media(imagen_a: np.ndarray, imagen_b: np.ndarray) -> float:
        """Diferencia absoluta media por píxel entre dos imágenes en gris."""
        return float(cv2.absdiff(imagen_a, imagen_b).mean())

    def _guardar(self, imagen: np.ndarray, imagen_gris: np.ndarray) -> None:
        """Escribe la imagen en disco y comprueba si se alcanzó el objetivo."""
        self._referencias_gris.append(imagen_gris)
        indice = len(self._referencias_gris)
        ruta: Path = self._directorio_salida / f"imagen_{indice:02d}.png"
        cv2.imwrite(str(ruta), imagen)
        self.get_logger().info(f"Imagen {indice}/{self._numero_objetivo} guardada: {ruta}")

        if indice >= self._numero_objetivo:
            self.terminado = True
            self.get_logger().info("Ingesta completada")


def main(args: Optional[List[str]] = None) -> None:
    """Punto de entrada: gira el nodo hasta reunir todas las imágenes."""
    rclpy.init(args=args)
    nodo = CapturadorImagenes()
    try:
        while rclpy.ok() and not nodo.terminado:
            rclpy.spin_once(nodo, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
