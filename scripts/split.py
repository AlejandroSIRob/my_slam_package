#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
import numpy as np
import Queue as queue
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import euler_from_quaternion
import tf
import random
import json
from geometry_msgs.msg import Point


class Segment:
    def __init__(self, p0, p1, idx0, idx1):
        self.p0 = p0
        self.p1 = p1
        self.idx0 = idx0
        self.idx1 = idx1

    def length(self): # Calcula longitud del segmento
        return np.linalg.norm(np.array(self.p1, np.float32) - np.array(self.p0, np.float32))

    def distance(self, p): # Distancia perpendicular de un punto al segmento
        l = self.length()
        if l == 0:
            return 0
       
        v = np.array(self.p1) - np.array(self.p0)
        v_perp = np.array([-v[1], v[0]]) / np.linalg.norm(v)
        d = abs(np.dot(v_perp, np.array(p) - np.array(self.p0)))
        return d
   
    def distance_mean_variance(self, point, cov_matrix): # Distancia media y varianza considerando incertidumbre
        v = np.array(self.p1) - np.array(self.p0)
        v_perp = np.array([-v[1], v[0]]) / np.linalg.norm(v)
        mean_distance = abs(np.dot(v_perp, np.array(point) - np.array(self.p0)))
        variance = np.dot(v_perp, np.dot(cov_matrix, v_perp.T))
        return mean_distance, variance

    def angle(self, s): # Angulo entre dos segmentos
        v1 = np.array(self.p1) - np.array(self.p0)
        v2 = np.array(s.p1) - np.array(s.p0)
        angle = np.arctan2(v1[1], v1[0]) - np.arctan2(v2[1], v2[0])
        return np.arctan2(np.sin(angle), np.cos(angle))

    def write(self):
        print('Segment: ', self.p0, '-', self.p1)
        print('Segment: ', self.idx0, '-', self.idx1)

class SplitAndMerge:
    def __init__(self, dist=0.1, angle=np.pi/30, purge_pts=12, purge_len=0.3):
        self.d_th = dist        # Threshold distance (split)
        self.a_th = angle       # Threshold angle (merge)
        self.pur_pts = purge_pts  # Min number of points (purge)
        self.pur_len = purge_len  # Min segment length (purge)
       
    def split(self, Pts): # Divide segmentos basado en distancia máxima
        segments = queue.LifoQueue()
        segments.put(Segment(Pts[0], Pts[-1], 0, len(Pts) - 1))
        final_segments = []

        while not segments.empty():
            segment = segments.get()
            max_dist = 0
            split_idx = -1

            for i in range(segment.idx0 + 1, segment.idx1):
                dist = segment.distance(Pts[i])
                if dist > max_dist:
                    max_dist = dist
                    split_idx = i

            if max_dist > self.d_th:
                segments.put(Segment(Pts[split_idx], segment.p1, split_idx, segment.idx1))
                segments.put(Segment(segment.p0, Pts[split_idx], segment.idx0, split_idx))
            else:
                final_segments.append(segment)

        return final_segments

    def merge(self, segs_in): # Combina segmentos con ángulos similares
        final_segments = []
        i = 0
        while i < len(segs_in) - 1:
            s1 = segs_in[i]
            s2 = segs_in[i + 1]

            if abs(s1.angle(s2)) < self.a_th:
                merged_segment = Segment(s1.p0, s2.p1, s1.idx0, s2.idx1)
                final_segments.append(merged_segment)
                segs_in[i+1] = merged_segment
                i += 1
            else:
                final_segments.append(s1)
                i += 1

        if i == len(segs_in) - 1:
            final_segments.append(segs_in[-1])

        return final_segments

    def purge(self, segs_in): # Filtra segmentos muy pequeños
        segs_out = []
        for seg in segs_in:
            if (seg.idx1 - seg.idx0 >= self.pur_pts) and (seg.length() >= self.pur_len):
                segs_out.append(seg)
        return segs_out
           
    def __call__(self, Pts):
        seg0 = self.split(Pts)
        seg1 = self.merge(seg0)
        seg2 = self.purge(seg1)
        return seg2

class EKFSLAMNode:
    def __init__(self):
        rospy.init_node('ekf_slam_node', anonymous=True)
        print("Nodo creado")
        self.state = np.zeros(3)  # [x, y, theta]
        self.covariance = np.eye(3)
        self.landmarks = {}
        self.H = np.zeros((2, 3))
        self.dt = 0.1
        self.min_range = float('inf')
        self.front_dist = float('inf')
        self.left_dist = float('inf')
        self.right_dist = float('inf')
       
        self.map_width = 100
        self.map_height = 100
        self.resolution = 0.1
        self.grid = np.zeros((self.map_width, self.map_height), dtype=np.int8)
       
        # Inicializar SplitAndMerge con parámetros adecuados
        self.split_merge = SplitAndMerge(dist=0.05, angle=np.pi/6, purge_pts=5, purge_len=0.2)
       
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
       
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.marker_pub = rospy.Publisher('/landmarks', MarkerArray, queue_size=10)
        self.map_pub = rospy.Publisher('/occupancy_grid', OccupancyGrid, queue_size=10)
        self.sensor_cone_pub = rospy.Publisher('/sensor_cone', Marker, queue_size=10)              # Cono teórico (gris)
        self.observed_cone_pub = rospy.Publisher('/observed_sensor_cone', Marker, queue_size=10)   # Cono observado (blanco)


        self.tf_broadcaster = tf.TransformBroadcaster()
        rospy.Timer(rospy.Duration(0.1), self.publish_tf)
        #rospy.Timer(rospy.Duration(0.1), self.control_loop)

        rospy.loginfo("EKF-SLAM Nodo Inicializado")

    def fuse_landmarks(self):
        """Fusiona landmarks que están muy cerca entre sí."""
        keys_to_remove = []
        for key1, (x1, y1) in self.landmarks.items():
            for key2, (x2, y2) in self.landmarks.items():
                if key1 != key2 and np.hypot(x1 - x2, y1 - y2) < 0.3:
                    self.landmarks[key1] = ((x1 + x2) / 2, (y1 + y2) / 2)
                    keys_to_remove.append(key2)
        for key in set(keys_to_remove):
            del self.landmarks[key]
   
    def detect_corners(self, segments, angle_threshold=np.pi/30): # 10 grados np.pi/18
        """Detecta esquinas en los segmentos encontrados."""
        corners = []
        if len(segments) < 2:
            return corners

        # Parámetros para filtrado
        min_segment_length = 0.25  # Longitud segmentos
        join_threshold = 0.08      # Tolerancia

        for i in range(len(segments)-1):
            s1 = segments[i]
            s2 = segments[i+1]

           
            # Verificar que ambos segmentos sean suficientemente largos
            if s1.length() < min_segment_length or s2.length() < min_segment_length:
                continue

            # Distancia entre los extremos que se tocan
            dist_join = np.linalg.norm(np.array(s1.p1) - np.array(s2.p0))

            # Si los extremos no están suficientemente cerca, ignoramos
            if dist_join > join_threshold:
                continue


            # Si s1.p1 y s2.p0 están algo separados pero apuntan en la misma dirección
            dir1 = np.array(s1.p1) - np.array(s1.p0)
            dir2 = np.array(s2.p1) - np.array(s2.p0)
            dir1 /= np.linalg.norm(dir1)
            dir2 /= np.linalg.norm(dir2)
            dot_product = np.dot(dir1, dir2)
            # Si son casi colineales pero separados, probablemente son la misma pared partida → no esquina
            if dot_product > 0.95 and dist_join > 0.04:  # Puedes ajustar 0.04 según el ruido
                continue  # No es una esquina real

            # Verificar si hay continuidad después de s2
            if i + 2 < len(segments):
                s3 = segments[i + 2]
                # Si s2 y s3 están unidos y casi alineados, no es una esquina real
                if np.linalg.norm(np.array(s2.p1) - np.array(s3.p0)) < join_threshold:
                    continue  # Pared continua, no es esquina real
   
            angle = abs(s1.angle(s2))
           
            # Si el ángulo es cercano a 90 grados, consideramos que hay una esquina
            #if abs(angle - np.pi/2) < angle_threshold:
            if abs(angle - np.pi/2) < angle_threshold: # or abs(2*np.pi - angle) < angle_threshold:

                # La esquina es el punto de unión entre los dos segmentos
                corner = s1.p1  # o s2.p0, deberían ser el mismo punto
                corners.append(corner)
               
        return corners

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        _, _, yaw = euler_from_quaternion(quaternion)
        self.state = np.array([position.x, position.y, yaw])

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        self.front_dist = np.min(ranges[345:360].tolist() + ranges[0:16].tolist())
        self.left_dist = np.min(ranges[10:20])
        self.right_dist = np.min(ranges[340:350])
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        points = []
       
        # Convertir datos del láser a coordenadas cartesianas
        for r, angle in zip(ranges, angles):
            if np.isfinite(r) and 0.1 <= r <= 3.5:
                lx = self.state[0] + r * np.cos(self.state[2] + angle)
                ly = self.state[1] + r * np.sin(self.state[2] + angle)
                points.append((lx, ly))
               
        if len(points) < 2:
            return
           
        # Aplicar Split and Merge
        segments = self.split_merge(points)
       
        # Detectar esquinas
        corners = self.detect_corners(segments)
       
        # Asociar landmarks y actualizar el EKF
        self.associate_landmarks(corners)
        self.publish_landmarks(corners)
        #self.create_occupancy_grid()
        self.publish_sensor_cone(msg)               # → Campo teórico (gris claro)
        self.publish_observed_sensor_cone(msg)      # → Campo observado (blanco)
        self.control_loop()

    def publish_sensor_cone(self, laser_msg): # Publicamos rango máximo del sensor
        marker = Marker()
        marker.header.frame_id = "base_link"  
        marker.header.stamp = rospy.Time.now()
        marker.ns = "sensor_cone"
        marker.id = 999
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        # Color gris semitransparente
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 0.3
        max_range = 3.5  # o laser_msg.range_max es lo mismo
        angle_min = laser_msg.angle_min
        angle_max = laser_msg.angle_max
        angle_inc = laser_msg.angle_increment

        angles = np.arange(angle_min, angle_max, angle_inc)
        origin = Point(0, 0, 0)

        for i in range(len(angles) - 1):
            a1 = angles[i]
            a2 = angles[i + 1]
            p1 = Point(max_range * np.cos(a1), max_range * np.sin(a1), 0)
            p2 = Point(max_range * np.cos(a2), max_range * np.sin(a2), 0)

            marker.points.extend([origin, p1, p2])  # Triángulo

        self.sensor_cone_pub.publish(marker)

    def publish_observed_sensor_cone(self, laser_msg): # Publicamos realemnete lo que ve 
        marker = Marker()
        marker.header.frame_id = "base_link"  
        marker.header.stamp = rospy.Time.now()
        marker.ns = "observed_sensor_cone"
        marker.id = 1000
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        # Color gris semitransparente
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.4

        origin = Point(0, 0, 0)
        angles = np.arange(laser_msg.angle_min, laser_msg.angle_max, laser_msg.angle_increment)
        ranges = np.array(laser_msg.ranges)

        valid_points = []

        for r, a in zip(ranges, angles):
            if np.isfinite(r) and laser_msg.range_min <= r <= laser_msg.range_max:
                x = r * np.cos(a)
                y = r * np.sin(a)
                valid_points.append(Point(x, y, 0))

        for i in range(len(valid_points) - 1):
            p1 = valid_points[i]
            p2 = valid_points[i + 1]
            marker.points.extend([origin, p1, p2])

        self.sensor_cone_pub.publish(marker)




    def create_occupancy_grid(self):
        if rospy.is_shutdown():
            return
           
        # Limpiar el grid (opcional, dependiendo de tu implementación)
        self.grid.fill(0)
       
        # Actualizar el grid con los landmarks
        for (lx, ly) in self.landmarks.values():
            x = int((lx + self.map_width / 2 * self.resolution) / self.resolution)
            y = int((ly + self.map_height / 2 * self.resolution) / self.resolution)
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                self.grid[x, y] = 100
               
        # Publicar el grid
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header = Header()
        occupancy_grid.header.stamp = rospy.Time.now()
        occupancy_grid.header.frame_id = "map"
        occupancy_grid.info.resolution = self.resolution
        occupancy_grid.info.width = self.map_width
        occupancy_grid.info.height = self.map_height
        occupancy_grid.info.origin.position.x = -self.map_width / 2 * self.resolution
        occupancy_grid.info.origin.position.y = -self.map_height / 2 * self.resolution
        occupancy_grid.info.origin.orientation.w = 1.0  
        occupancy_grid.data = self.grid.flatten().tolist()
        self.map_pub.publish(occupancy_grid)

    def publish_landmarks(self, corners):
        if not corners:
            rospy.logwarn("No se detectaron landmarks")
            return

        marker_array = MarkerArray()
        for i, corner in enumerate(corners):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "landmarks"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = corner[0]
            marker.pose.position.y = corner[1]
            marker.pose.position.z = 0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.lifetime = rospy.Duration(3.0)  # Los markers desaparecerán después de 3 segundos si no se actualizan
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)
       
    def associate_landmarks(self, new_landmarks):
        for corner in new_landmarks:
            lx, ly = corner
            landmark_key = self.associate_measurements(lx, ly)
            if landmark_key is None:
                landmark_key = len(self.landmarks)
                self.landmarks[landmark_key] = (lx, ly)
            else:
                self.ekf_update(np.array([np.hypot(lx - self.state[0], ly - self.state[1]),
                              np.arctan2(ly - self.state[1], lx - self.state[0]) - self.state[2]]),
                            self.landmarks[landmark_key])

    def associate_measurements(self, lx, ly):
        for key, (x, y) in self.landmarks.items():
            dx = lx - x
            dy = ly - y
            distance = np.hypot(dx, dy)
            if distance < 0.5:
                innovation = np.array([distance, np.arctan2(dy, dx) - self.state[2]])
                S = np.dot(np.dot(self.H, self.covariance), self.H.T) + np.diag([0.1, np.deg2rad(5)])
                mahalanobis_distance = np.dot(np.dot(innovation.T, np.linalg.inv(S)), innovation)
                if mahalanobis_distance < 5.99:
                    return key
        return None

    def ekf_update(self, measurement, landmark):
        dx = landmark[0] - self.state[0]
        dy = landmark[1] - self.state[1]
        q = dx**2 + dy**2

        z_pred = np.array([np.sqrt(q), np.arctan2(dy, dx) - self.state[2]])

        H = np.zeros((2, 3))
        H[0, 0] = -dx / np.sqrt(q)
        H[0, 1] = -dy / np.sqrt(q)
        H[1, 0] = dy / q
        H[1, 1] = -dx / q
        H[1, 2] = -1
       
        y = measurement - z_pred
        y[1] = np.arctan2(np.sin(y[1]), np.cos(y[1]))

        S = np.dot(np.dot(H, self.covariance), H.T) + np.diag([0.1, np.deg2rad(5)])
        K = np.dot(np.dot(self.covariance, H.T), np.linalg.inv(S))

        self.state += np.dot(K, y)
        self.covariance = np.dot(np.eye(3) - np.dot(K, H), self.covariance)

    def ekf_predict(self, control):
        v, omega = control
        theta = self.state[2]
        self.state[0] += v * np.cos(theta) * self.dt
        self.state[1] += v * np.sin(theta) * self.dt
        self.state[2] += omega * self.dt

        G = np.eye(3)
        G[0, 2] = -v * np.sin(theta) * self.dt
        G[1, 2] = v * np.cos(theta) * self.dt

        self.covariance = np.dot(np.dot(G, self.covariance), G.T) + np.diag([0.05, 0.05, np.deg2rad(2)])

    def control_loop(self, event=None):
        cmd = Twist()
        if self.front_dist < 0.4:
            cmd.linear.x = -0.1
            if self.left_dist > self.right_dist:
                cmd.angular.z = np.pi
                rospy.loginfo("Girando Izquierda")
            else:
                cmd.angular.z = -np.pi
                rospy.loginfo("Girando Derecha")
        else:
            cmd.linear.x = min(0.2, 0.5 * self.min_range)
            cmd.angular.z = random.uniform(-1.0, 1.0)
       
        self.ekf_predict((cmd.linear.x, cmd.angular.z))
        self.cmd_pub.publish(cmd)

    def publish_tf(self, event):
        br = tf.TransformBroadcaster()
        br.sendTransform(
            (0, 0, 0),
            tf.transformations.quaternion_from_euler(0, 0, 0),
            rospy.Time.now(),
            "odom",
            "map"
        )
        br.sendTransform(
            (self.state[0], self.state[1], 0),  
            tf.transformations.quaternion_from_euler(0, 0, self.state[2]),  
            rospy.Time.now(),
            "base_link",
            "map"
        )
        br.sendTransform(
            (0, 0, 0),
            tf.transformations.quaternion_from_euler(0, 0, 0),
            rospy.Time.now(),
            "base_footprint",
            "base_link"
        )
        br.sendTransform(
            (-0.032, 0, 0.171),
            tf.transformations.quaternion_from_euler(0, 0, 0),
            rospy.Time.now(),
            "base_scan",
            "base_link"
        )
        br.sendTransform(
            (-0.081, 0, -0.004),
            tf.transformations.quaternion_from_euler(0, 0, 0),
            rospy.Time.now(),
            "caster_back_link",
            "base_link"
        )
        br.sendTransform(
            (0, 0, 0.1),
            tf.transformations.quaternion_from_euler(0, 0, 0),
            rospy.Time.now(),
            "imu_link",
            "base_link"
        )
        br.sendTransform(
            (0, 0.08, 0.023),
            tf.transformations.quaternion_from_euler(-1.57, 0, 0),
            rospy.Time.now(),
            "wheel_left_link",
            "base_link"
        )
        br.sendTransform(
            (0, -0.08, 0.023),
            tf.transformations.quaternion_from_euler(-1.57, 0, 0),
            rospy.Time.now(),
            "wheel_right_link",
            "base_link"
        )

if __name__ == "__main__":
    ekf_slam_node = EKFSLAMNode()
    rospy.loginfo("EKF-SLAM iniciado. Presiona Ctrl+C para salir.")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Nodo EKF-SLAM detenido por el usuario.")


