#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import euler_from_quaternion
import random  # Importar para generar valores aleatorios
import json
import tf

class EKFSLAMNode:
    def __init__(self):
        rospy.init_node('ekf_slam_node', anonymous=True)
        # Estado del robot y mapa
        self.state = np.zeros(3)  # [x, y, theta]
        self.covariance = np.eye(3)  # Covarianza inicial
        self.landmarks = {}  # Diccionario de landmarks

        # Inicializar min_range
        self.min_range = float('inf')

        # Variables globales para las distancias
        self.front_dist = float('inf')
        self.left_dist = float('inf')
        self.right_dist = float('inf')

        # Configuración
        self.dt = 0.1  # Intervalo de tiempo
        self.motion_noise = np.diag([0.05, 0.05, np.deg2rad(2)])  # Menos ruido en el movimiento
        self.sensor_noise = np.diag([0.1, np.deg2rad(5)])  # Menos ruido en las mediciones

        
        # Configura el mapa
        self.map_width = 100  # Define el ancho del mapa en celdas
        self.map_height = 100  # Define la altura del mapa en celdas
        self.resolution = 0.1  # Resolución en metros por celda
        self.grid = np.zeros((self.map_width, self.map_height), dtype=np.int8)  # Crea un mapa vacío con valores 0 (vacío)

        # Suscriptores
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Publicadores
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.marker_pub = rospy.Publisher('/landmarks', MarkerArray, queue_size=10)
	self.map_pub = rospy.Publisher('/occupancy_grid', OccupancyGrid, queue_size=10)  # Publicador para el mapa de ocupación

        # Temporizador para la lógica principal
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.control_loop)

	# Inicialización de transformaciones TF
        self.tf_broadcaster = tf.TransformBroadcaster()
        rospy.Timer(rospy.Duration(0.1), self.publish_tf)  # Publicar TF periódicamente

	rospy.loginfo("EKF-SLAM Nodo Inicializado")

    def create_occupancy_grid(self):
        """Crea y publica el mapa de ocupación."""
        occupancy_grid = OccupancyGrid()

        # Configuración del mapa
        occupancy_grid.header = Header()
        occupancy_grid.header.stamp = rospy.Time.now()
        occupancy_grid.header.frame_id = "map"

	# Información del mapa
        occupancy_grid.info.resolution = self.resolution
        occupancy_grid.info.width = self.map_width
        occupancy_grid.info.height = self.map_height
        occupancy_grid.info.origin.position.x = 5.0  # Origen del mapa
        occupancy_grid.info.origin.position.y = 5.0
        occupancy_grid.info.origin.orientation.x = 0.0
        occupancy_grid.info.origin.orientation.y = 0.0
        occupancy_grid.info.origin.orientation.z = 1.0
        occupancy_grid.info.origin.orientation.w = 0.0  

        # Convierte la grilla en una lista plana de ocupación (0 libre, 100 ocupado)
        occupancy_grid.data = self.grid.flatten().tolist()

        # Publica el mapa de ocupación
        self.map_pub.publish(occupancy_grid)


    def publish_tf(self, event):
	"""Publica las transformaciones de marco (TF) entre map, odom y base_link."""

	# Validar el estado antes de publicar
	if np.isnan(self.state[2]):
            rospy.logwarn("Theta (orientación) es NaN. No se publica TF.")
            return

        # Transformación entre "map" y "odom"
        self.tf_broadcaster.sendTransform(
            (self.state[0], self.state[1], 0),  # Posición (x, y, z)
            tf.transformations.quaternion_from_euler(0, 0, self.state[2]),  # Orientación
            rospy.Time.now(),
            "odom",  # Hijo
            "map"  # Padre
        )
        # Transformación entre "odom" y "base_link"
        self.tf_broadcaster.sendTransform(
            (0, 0, 0),  # Posición relativa del robot en "odom"
            tf.transformations.quaternion_from_euler(0, 0, 0),  # Sin rotación adicional
            rospy.Time.now(),
            "base_link",  # Hijo
            "odom"  # Padre
        )

    def odom_callback(self, msg):
        """Procesa la odometría para actualizar la posición del robot."""
        q = msg.pose.pose.orientation
        _, _, theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
	theta = np.arctan2(np.sin(theta), np.cos(theta))

        self.state[:2] = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.state[2] = theta

    def associate_measurements(self, lx, ly):
        for key, (x, y) in self.landmarks.items():
            if np.hypot(lx - x, ly - y) < 1.0:  # Umbral de asociación
                return key
        return None  # No asociado

    def ekf_predict(self, control):
	"""Etapa de predicción del EKF: actualiza el estado según el modelo de movimiento."""
        v, omega = control
        theta = self.state[2]

        # Actualización del estado
        self.state[0] += v * np.cos(theta) * self.dt
        self.state[1] += v * np.sin(theta) * self.dt
        self.state[2] += omega * self.dt

        # Jacobiano y propagación de la covarianza
        G = np.eye(3)
        G[0, 2] = -v * np.sin(theta) * self.dt
        G[1, 2] = v * np.cos(theta) * self.dt
        self.covariance = np.dot(np.dot(G, self.covariance), G.T) + self.motion_noise
	print(self.covariance)
	rospy.loginfo("Estado después de predicción: {}".format(self.state))


    def ekf_update(self, measurement, landmark):
	"""Etapa de actualización del EKF: corrige el estado con mediciones."""
        dx = landmark[0] - self.state[0]
        dy = landmark[1] - self.state[1]
        q = dx*2 + dy*2

        # Predicción de la medición
        z_pred = np.array([np.sqrt(q), np.arctan2(dy, dx) - self.state[2]])

        # Jacobiano de la medición
        H = np.zeros((2, 3))
        H[0, 0] = -dx / np.sqrt(q)
        H[0, 1] = -dy / np.sqrt(q)
        H[1, 0] = dy / q
        H[1, 1] = -dx / q
        H[1, 2] = -1

        # Innovación y ganancia de Kalman
        y = measurement - z_pred
        y[1] = np.arctan2(np.sin(y[1]), np.cos(y[1]))  # Normalizar ángulo

        S = np.dot(np.dot(H, self.covariance), H.T) + self.sensor_noise
        K = np.dot(np.dot(self.covariance, H.T), np.linalg.inv(S))

        # Prueba de consistencia
	# Distancia de Mahalanobis para filtrar outliers
        maha_dist = np.dot(y.T, np.dot(np.linalg.inv(S), y))  # Distancia de Mahalanobis
        threshold = 9.21  # Valor del test de chi-cuadrado para 2D al 99%
        if maha_dist > threshold:
            rospy.logwarn("Observación descartada por ser un outlier.")
            return

        # Actualización del estado y covarianza
        self.state += np.dot(K, y)
        self.covariance = np.dot(np.eye(3) - np.dot(K, H), self.covariance)



    def save_landmarks_to_file(self, filename="landmarks.json"):
	"""Guarda los landmarks hasta el valor predefinido en control_loop. Actualemnete comentado ya que esta parte es para realizar pruebas de tiempo"""
        rospy.loginfo("Guardando landmarks en archivo...")
        with open(filename, 'w') as f:
            json.dump(self.landmarks, f)
        rospy.loginfo("Landmarks guardados en ",filename)

    def is_mapping_complete(self):
        if not hasattr(self, 'last_landmark_count'):
            self.last_landmark_count = len(self.landmarks)
            self.stable_count = 0
        if len(self.landmarks) == self.last_landmark_count:
                self.stable_count += 1
        else:
            self.stable_count = 0

        self.last_landmark_count = len(self.landmarks)

        # Considerar mapeo completo si no hay cambios en los landmarks por un tiempo
        return self.stable_count > 500  # Ajustable según tu frecuencia



    def scan_callback(self, msg):
	"""Procesa las mediciones del LIDAR y actualiza landmarks y mapa."""
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # Actualizar distancias específicas
        self.front_dist =  np.min(ranges[345:360].tolist() + ranges[0:16].tolist())  # De 345° a 15°
        self.left_dist = np.min(ranges[10:20])  # Valores entre 10° y 20°
        self.right_dist = np.min(ranges[340:350])  # Valores entre 340° y 350°

        # Mostrar información en la terminal
        rospy.loginfo("---- Laser Scan Data ----")
        rospy.loginfo("Front Distance (±15°): {:.2f} m".format(self.front_dist))
        rospy.loginfo("Left Distance (10°-20°): {:.2f} m".format(self.left_dist))
        rospy.loginfo("Right Distance (340°-350°): {:.2f} m".format(self.right_dist))
        rospy.loginfo("--------------------------")


        # Definir la región frontal: ±30° (convertidos a radianes)
        frontal_mask = (angles >= -np.pi / 6) & (angles <= np.pi / 6)

        # Filtrar distancias dentro de la región frontal
        frontal_ranges = ranges[frontal_mask]

        # Validar las distancias y calcular la mínima
        if np.isfinite(frontal_ranges).any():
            self.min_range = np.min(frontal_ranges[np.isfinite(frontal_ranges)])
        else:
            self.min_range = float('inf')  # No hay obstáculos detectados
        
        for r, angle in zip(ranges, angles):
            if not (np.isfinite(r) and 0.1 <= r <= 3.5):  # Solo procesar rangos válidos
   		 continue

            # Convertir a coordenadas globales

            lx = self.state[0] + r * np.cos(self.state[2] + angle)
            ly = self.state[1] + r * np.sin(self.state[2] + angle)
            if np.isnan(lx) or np.isnan(ly):
                continue


            # Asociar o inicializar landmark
            landmark_key = self.associate_measurements(lx, ly)
            if landmark_key is None:
                landmark_key = len(self.landmarks)
                self.landmarks[landmark_key] = (lx, ly)
            else:
                self.ekf_update(np.array([r, angle]), self.landmarks[landmark_key])

           # Actualizar el mapa de ocupación
            # Aquí asumimos que el mapa está centrado en (0,0) y cada celda es de 0.1m
            grid_x = int((lx - (-self.map_width / 2 * self.resolution)) / self.resolution)
            grid_y = int((ly - (-self.map_height / 2 * self.resolution)) / self.resolution)



            if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
                self.grid[grid_x, grid_y] = 100  # Marcar como ocupado

        # Después de procesar las mediciones del LIDAR, publica el mapa actualizado
        self.create_occupancy_grid()

    def publish_robot_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"  # Asegúrate de que el marco sea "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "robot"
        marker.id = 0
        marker.type = Marker.SPHERE  # Forma del marcador
        marker.action = Marker.ADD
        marker.pose.position.x = self.state[0]
        marker.pose.position.y = self.state[1]
        marker.pose.position.z = 0.1  # Altura del marcador
	quaternion = tf.transformations.quaternion_from_euler(0, 0, self.state[2])
	marker.pose.orientation.x = quaternion[0]
	marker.pose.orientation.y = quaternion[1]
	marker.pose.orientation.z = quaternion[2]
	marker.pose.orientation.w = quaternion[3]
	marker.scale.x = 0.3
	marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0  # Azul para diferenciar del mapa
        # Encapsular el marcador en un MarkerArray
        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        # Publicar el MarkerArray
        self.marker_pub.publish(marker_array)


    def control_loop(self, event):
        # Movimiento básico del robot
        cmd = Twist()
        # Evitar obstáculos si están muy cerca
        if self.front_dist < 0.4:  # Obstáculo dentro de 0.4 m
            rospy.loginfo("Obstáculo detectado. Girando...")
            cmd.linear.x = -0.1  # Detenerse y retrocede
            if self.left_dist > self.right_dist:  # Si hay más espacio a la izquierda
                cmd.angular.z = np.pi  # Girar a la izquierda
                rospy.loginfo("Girando Izquierda")
            else:
                cmd.angular.z = -np.pi  # Girar a la derecha
                rospy.loginfo("Girando Derecha")

        else:
            rospy.loginfo("Sin obstáculos cercanos. Moviendo hacia adelante...")
            cmd.linear.x = min(0.2, 0.5 * self.min_range)  # Más rápido si está despejado
            cmd.angular.z = random.uniform(-1.0, 1.0)  # Velocidad angular

        self.ekf_predict((cmd.linear.x, cmd.angular.z))
        self.cmd_pub.publish(cmd)
        self.publish_robot_marker()


        '''if self.is_mapping_complete():
            rospy.loginfo("El mapeo ha sido completado.")
            self.save_landmarks_to_file("landmarks.json")
            rospy.signal_shutdown("Mapeo completo.")
        '''


if __name__ == "__main__":
    try:
        ekf_slam_node = EKFSLAMNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo interrumpido.")
    finally:
        rospy.loginfo("Finalizando el nodo EKF-SLAM.")
