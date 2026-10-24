#!/usr/bin/python3
"""
Práctica 1:
 1. Suscriptor de /joint_states que registra el estado de las articulaciones en tiempo real.
 2. Ejecución de 3 trayectorias (joint, lineal y circular) para la ingesta de datos.
 3. Procesado y almacenamiento de posición, velocidad y aceleración por articulación,
    con su marca de tiempo, en un CSV distinto para cada trayectoria.
"""

import copy
import csv
import math
import os
import threading
import time
from typing import Callable, Dict, List

import rclpy
from control_robot import ControlRobot
from suscriptor_articulaciones import SuscriptorRegistradorArticulaciones
from geometry_msgs.msg import Pose
from practica1_msgs.msg import EstadoCinematico
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.publisher import Publisher
from sensor_msgs.msg import JointState

CARPETA_DATOS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "datos")

# función que calcula velocidad y aceleración a partir de las muestras
def calcular_cinematica(muestras: List[dict], articulaciones: List[str]) -> List[dict]:
    """A partir de las muestras, calcula la velocidad y aceleración de cada articulación."""
    filas = []
    vel_anterior = dict.fromkeys(articulaciones, 0.0)
    tiempo_anterior = None
    pos_anterior = {}

    # por cada muestra de articulaciones, calcular velocidad y aceleración
    for muestra in muestras:
        tiempo = muestra["tiempo"]
        dt = tiempo - tiempo_anterior if tiempo_anterior is not None else None
        fila = {"tiempo": tiempo}

        # por cada articulación, calcular posición, velocidad y aceleración
        for nombre_articulacion in articulaciones:
            # traemos todas las posiciones de una misma articulación
            posicion = muestra["posiciones"].get(nombre_articulacion)

            # si hay valocidad previa registrada la usamos directamente
            if nombre_articulacion in muestra["velocidades"]:
                velocidad = muestra["velocidades"][nombre_articulacion]
            # si no, calculamos la velocidad (x2-x1)/delta_tiempo si contamos con los datos previos
            elif dt and dt > 0 and nombre_articulacion in pos_anterior:
                velocidad = (posicion - pos_anterior[nombre_articulacion]) / dt
            else:
                velocidad = 0.0

            # con la velocidad calculamos la aceleración si contamos con los datos necesarios
            if dt and dt > 0 and nombre_articulacion in vel_anterior:
                aceleracion = (velocidad - vel_anterior[nombre_articulacion]) / dt
            else:
                aceleracion = 0.0
            
            fila[f"pos_{nombre_articulacion}"] = posicion
            fila[f"vel_{nombre_articulacion}"] = velocidad
            fila[f"acc_{nombre_articulacion}"] = aceleracion
            vel_anterior[nombre_articulacion] = velocidad
            pos_anterior[nombre_articulacion] = posicion
        filas.append(fila)
        tiempo_anterior = tiempo
    return filas

# funcion para guardar la posicion, velocidad y aceleración en un archivo csv
def guardar_csv(nombre_archivo: str, filas: List[dict], articulaciones: List[str]) -> None:
    """Escribe las filas ya calculadas en un CSV, una columna pos/vel/acc por articulación."""
    os.makedirs(CARPETA_DATOS, exist_ok=True)
    ruta = os.path.join(CARPETA_DATOS, nombre_archivo)

    # Una columna "pos", "vel" y "acc" por cada articulación, en ese orden
    columnas = ["tiempo"]
    for nombre_articulacion in articulaciones:
        for magnitud in ("pos", "vel", "acc"):
            columnas.append(f"{magnitud}_{nombre_articulacion}")

    with open(ruta, "w", newline="") as f:
        escritor = csv.DictWriter(f, fieldnames=columnas)
        escritor.writeheader()
        escritor.writerows(filas)
    print(f"Guardado: {ruta} ({len(filas)} muestras)")

# Ejecuta y registra la actividad de las trayectorias del robot
def ejecutar_y_registrar(control: ControlRobot, registrador: SuscriptorRegistradorArticulaciones,
                          publicador_estado: Publisher, nombre_archivo: str,
                          accion: Callable[[], None]) -> None:
    """Graba una trayectoria completa: ejecuta accion(), guarda el CSV y publica el mensaje personalizado."""
    registrador.iniciar_grabacion()
    accion() # ejecuta la trayectoria que le mandamos por parametro
    muestras = registrador.detener_grabacion() # se detiene la grabación justo al terminar
    filas = calcular_cinematica(muestras, control.articulaciones) # se hacen los calculos pedidos con esta información
    guardar_csv(nombre_archivo, filas, control.articulaciones) # se guardan en un csv.
    publicar_estados(publicador_estado, control.articulaciones, filas) # y se publican como mensaje personalizado

# función que empaqueta cada fila ya calculada (posición+velocidad+aceleración) en un
# mensaje EstadoCinematico y lo publica (reto: mensaje ROS 2 personalizado)
def publicar_estados(publicador_estado: Publisher, articulaciones: List[str], filas: List[dict]) -> None:
    """Publica un EstadoCinematico por cada fila, con el tiempo original de la muestra."""
    for fila in filas:
        estado = EstadoCinematico()
        estado.header.stamp.sec = int(fila["tiempo"])
        estado.header.stamp.nanosec = int(round((fila["tiempo"] - estado.header.stamp.sec) * 1e9))
        estado.nombres = list(articulaciones)
        estado.posiciones = [fila[f"pos_{a}"] for a in articulaciones]
        estado.velocidades = [fila[f"vel_{a}"] for a in articulaciones]
        estado.aceleraciones = [fila[f"acc_{a}"] for a in articulaciones]
        publicador_estado.publish(estado)

# Trayectoria 1: Movimientos en espacio de articulaciones (joint)
def trayectoria_joint(control: ControlRobot) -> None:
    """Trayectoria 1: movimientos en espacio de articulaciones (joint)."""
    # Comprendiendo que pi radianes es 180 grados y pi/2 radianes es 90 grados
    pi_medios = math.pi / 2 # encontramos cuantos radianes son 90 grados

    # comprendiendo que hay 6 articulaciones en el brazo del robot
    # ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    # hombro a 0°, codo/hombro-elevación a -90°, codo a -90°, muñeca 1 a -90°, muñeca 2 a +90°, muñeca 3 a 0°
    control.mover_articulaciones([0, -pi_medios, -pi_medios, -pi_medios, pi_medios, 0])
    control.mover_articulaciones([pi_medios, -pi_medios, -pi_medios / 2, -pi_medios, pi_medios, 0])
    control.mover_articulaciones([0, -pi_medios, -pi_medios, -pi_medios, pi_medios, 0])


# Trayectoria 2: Incluye movimientos lineales 
def trayectoria_lineal(control: ControlRobot) -> None:
    """Trayectoria 2: movimiento lineal, el efector recorre varios puntos en línea recta."""
    # Comprendiendo que Z esta hacia arriba por convencion en ROS

    pose = control.pose_actual() # encontrar donde esta el efector final ahora
    poses = [] # sucesion de poses
    # Con base en la pose actual
    pose.position.z += 0.03 # 1. Movemos 3 cm hacia arriba
    poses.append(copy.deepcopy(pose))
    pose.position.x += 0.03 # 2. Movemos 3 cm hacia la derecha
    poses.append(copy.deepcopy(pose))
    pose.position.x -= 0.03 # 3. Movemos 3 cm hacia la izquierda
    poses.append(copy.deepcopy(pose))
    pose.position.z -= 0.03 # 4. Movemos 3 cm hacia abajo
    poses.append(copy.deepcopy(pose))
    control.mover_trayectoria(poses) # Ejecutamos la trayectoria lineal

# Trayectoria 3: Movimiento circular
def trayectoria_circular(control: ControlRobot, radio: float = 0.08, num_puntos: int = 24) -> None:
    """Trayectoria 3: movimiento circular, es decir ponemos puntos que simulen una circunferencia. (entre mas cercanos los puntos se verá más suave la trayectoria)"""

    # Utilizando la pose actual como centro
    centro = control.pose_actual()
    poses = [] # Enlistamos los puntos que formarán la trayectoria circular

    # un circulo equivale a 360 grados, pero trabajamos en radianes (2*pi)
    radianes_por_punto = 2 * math.pi / num_puntos # obtenemos cuantos radianes corresponde a cada punto
    for i in range(0, num_puntos): # Por default usaremos 24 puntos para la circunferencia
        num_de_punto = i + 1
        angulo = radianes_por_punto * num_de_punto
        pose = copy.deepcopy(centro) # asignamos ese punto como base para la nueva pose

        # usando la formula para encontrar x y y de una circunferencia situada en 0,0
        # x(θ)=r(cosθ-1) para que el círculo comience desde el centro actual en x 
        # (porque cos esta desplazado 1, no como sin que esta en 0)
        # y(θ)=rsinθ
        pose.position.x += radio * (math.cos(angulo) - 1) 
        pose.position.y += radio * math.sin(angulo)
        poses.append(pose)
    control.mover_trayectoria(poses)


if __name__ == '__main__':
    rclpy.init()

    control = ControlRobot()
    registrador = SuscriptorRegistradorArticulaciones()
    ejecutor = MultiThreadedExecutor()
    ejecutor.add_node(registrador)
    threading.Thread(target=ejecutor.spin, daemon=True).start()
    publicador_estado = control.create_publisher(EstadoCinematico, "/estado_cinematico", 10)

    ejecutar_y_registrar(control, registrador, publicador_estado, "trayectoria_1_joint.csv",
                          lambda: trayectoria_joint(control))
    time.sleep(2.0) # Espera 2 segundos antes de la siguiente trayectoria

    ejecutar_y_registrar(control, registrador, publicador_estado, "trayectoria_2_lineal.csv",
                          lambda: trayectoria_lineal(control))
    
    time.sleep(2.0)

    ejecutar_y_registrar(control, registrador, publicador_estado, "trayectoria_3_circular.csv",
                          lambda: trayectoria_circular(control))

    ejecutor.shutdown()
    registrador.destroy_node()
    control.ejecutor.shutdown()
    control.destroy_node()
    rclpy.shutdown()
