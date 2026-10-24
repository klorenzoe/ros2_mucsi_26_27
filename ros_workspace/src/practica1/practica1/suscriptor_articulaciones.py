#!/usr/bin/python3

import threading
from typing import Dict, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Por debajo de esta velocidad consideramos que la articulación está parada
UMBRAL_VELOCIDAD_PARADO = 0.005 # Entendiendo que casi nunca se llega al 0.0 como que siempre hay ruido.

class SuscriptorRegistradorArticulaciones(Node):
    """Suscriptor de /joint_states que, mientras está grabando, guarda cada muestra con su
    marca de tiempo (la del propio mensaje, válida tanto en simulación como en el robot real),
    pero solo cuando el robot se está moviendo (ingesta inteligente: los periodos
    estáticos se ignoran por completo, aunque se esté grabando)."""

    def __init__(self) -> None:
        """Crea el suscriptor, inicialmente sin grabar ninguna muestra."""
        super().__init__("registrador_articulaciones")
        self._muestras: List[dict] = []
        self._grabando = False
        self._lock = threading.Lock()
        self.create_subscription(JointState, "/joint_states", self.callback, 50)

    def callback(self, msg: JointState) -> None:
        """Procesa cada mensaje de /joint_states; solo guarda la muestra si está grabando y en movimiento."""
        if not self._grabando:
            return
        velocidades = dict(zip(msg.name, msg.velocity)) if msg.velocity else {}

        # Ingesta inteligente: si ninguna articulación supera el umbral, el robot está
        # parado y la muestra se ignora por completo (no se procesa ni se guarda)
        if not any(abs(v) > UMBRAL_VELOCIDAD_PARADO for v in velocidades.values()):
            return

        # Obtener el tiempo del mensaje en segundos (incluyendo nanosegundos)
        tiempo = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Guardar la muestra actual con tiempo, posiciones y velocidades de forma segura
         # Almacenamiento seguro usando cerrojo (_lock) para evitar problemas de concurrencia
        with self._lock:
            self._muestras.append({
                "tiempo": tiempo,
                "posiciones": dict(zip(msg.name, msg.position)),
                "velocidades": velocidades,
            })

    def iniciar_grabacion(self) -> None:
        """Vacía las muestras anteriores y empieza a grabar las nuevas."""
        with self._lock:
            self._muestras = []
            self._grabando = True

    def detener_grabacion(self) -> List[dict]:
        """Deja de grabar y devuelve todas las muestras capturadas desde iniciar_grabacion()."""
        with self._lock:
            self._grabando = False
            return self._muestras

__all__ = ["SuscriptorRegistradorArticulaciones"]