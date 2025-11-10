import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import math


class Maze_Navigation_Node(Node):
    def __init__(self):
        super().__init__("maze_navigation_node")
        self.get_logger().info("🚀 Nodo de movimiento iniciado")

        # Publisher a cmd_vel
        self.cmd_vel_publisher = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        # Subscriber a scan --> Lidar
        self.pose_subscriber = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

    def scan_callback(self, msg):
        
        # msg para cmd_vel
        cmd = Twist()

        # msg del lidar
        self.lidar_msg = msg

        # Lectura de datos del LIDAR
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # Rango frontal -80° a 80°
        mask = (np.isfinite(ranges)) & (np.radians(-80) <= angles) & (angles <= np.radians(80))
        valid_ranges = ranges[mask]
        valid_angles = angles[mask]

        # Revisión por si no hay datos validos
        if len(valid_ranges) == 0:
            self.get_logger().warn("🚧 No hay datos válidos en -80° a 80°.")
            return

        # Suavizado de ruido
        smooth_ranges = np.convolve(valid_ranges, np.ones(3)/3, mode='same')
        new_ranges = []
        new_angles = []

        # Filtramos puntos muy cercanos (< 1.0 m)
        for i in range(len(smooth_ranges)):
            if smooth_ranges[i] > 1.0:
                new_ranges.append(smooth_ranges[i])
                new_angles.append(valid_angles[i])

        # Revisión por si no hay datos validos
        if len(new_ranges) == 0:
            self.get_logger().warn("🚧 No hay puntos válidos después del filtrado.")
            return

        # Promediamos en bloques
        valid_ranges_2, valid_angles_2 = self.average_lidar_in_blocks(new_ranges, new_angles, block_size=10)

        # Pasamos a coordenadas cartesianas
        x_points = valid_ranges_2 * np.cos(valid_angles_2)
        y_points = valid_ranges_2 * np.sin(valid_angles_2)

        # Calculamos el centroide como "goal"
        goal_x = np.mean(x_points)
        goal_y = np.mean(y_points)

        # Distancia frontal (-5° a 5°)
        mask = (np.isfinite(ranges)) & (np.radians(-5) <= angles) & (angles <= np.radians(5))
        front_distance = ranges[mask]
        front_distance = min(front_distance) if len(front_distance) > 0 else 10.0

        # Estrategia de evasión de obstáculos -----------------------------
        
        # Si la distancia frontal es menor de 1.2 metros, asumimos que hay una pared delante.
        if front_distance < 1.2:

            # Mostramos un mensaje de advertencia en el registro, indicando la distancia detectada a la pared.
            self.get_logger().warning(f"⚠️ Pared detectada a {front_distance:.2f} m")

            # Creamos una máscara (filtro booleano) para los rangos del lado izquierdo:
            # seleccionamos los valores finitos cuyos ángulos estén entre 10° y 80°.
            mask_left = (np.isfinite(ranges)) & (np.radians(10) <= angles) & (angles <= np.radians(80))

            # Creamos otra máscara para el lado derecho:
            # seleccionamos los valores finitos cuyos ángulos estén entre -80° y -10°.
            mask_right = (np.isfinite(ranges)) & (np.radians(-80) <= angles) & (angles <= np.radians(-10))

            # Calculamos la distancia media hacia la izquierda.
            # Si hay datos válidos en mask_left, tomamos la media; si no, asumimos 0.0.
            left_distance = np.mean(ranges[mask_left]) if np.any(mask_left) else 0.0

            # Calculamos la distancia media hacia la derecha, con la misma lógica que para la izquierda.
            right_distance = np.mean(ranges[mask_right]) if np.any(mask_right) else 0.0

            # Si la distancia izquierda es mayor que la derecha, significa que hay más espacio a la izquierda,
            # por lo que el robot girará hacia ese lado (izquierda, 60°).
            if left_distance > right_distance:
                goal_x = front_distance * np.cos(np.radians(60))
                goal_y = front_distance * np.sin(np.radians(60))

            # En caso contrario, hay más espacio a la derecha y el robot girará hacia ese lado (-60°).
            else:
                goal_x = front_distance * np.cos(np.radians(-60))
                goal_y = front_distance * np.sin(np.radians(-60))
        
        # ---------------------------------------------------------------



        # Calculamos el ángulo hacia el "goal"
        best_angle = math.atan2(goal_y, goal_x)

        # Limitar ángulo máximo a ±120°
        if abs(best_angle) > math.radians(120):
            self.get_logger().warn(f"⚠️ Ángulo excesivo ({math.degrees(best_angle):.1f}°). Ajustando...")
            best_angle = np.sign(best_angle) * math.radians(120)

        # Control proporcional para el giro
        k_ang = 1.0  # Ganancia angular
        cmd.angular.z = k_ang * best_angle # Vemos la velocidad angular z del robot para el mensaje

        # Ajustar velocidad lineal según proximidad
        if front_distance < 0.7:      # Muy cerca → detenerse
            cmd.linear.x = 0.0
        elif front_distance < 1.2:    # Preparar giro
            cmd.linear.x = 0.5
        else:                         # Camino libre
            cmd.linear.x = 1.0

        # Publicamos comando
        self.cmd_vel_publisher.publish(cmd)

        self.get_logger().info(
            f"🎯 Goal=({goal_x:.2f},{goal_y:.2f}) "
            f"yaw={math.degrees(best_angle):.1f}° "
            f"vel=({cmd.linear.x:.2f},{cmd.angular.z:.2f})"
        )


    def average_lidar_in_blocks(self, ranges, angles, block_size=10):
    
        # Agrupa los datos del LIDAR en bloques de tamaño fijo y calcula
        # el promedio de distancia y ángulo dentro de cada bloque.

        # Esto sirve para reducir el número de puntos y suavizar el ruido
        # de las mediciones del sensor.
        

        # Convertimos las listas (si vienen como tal) a arrays de NumPy,
        # lo que permite hacer operaciones vectorizadas (más rápidas y limpias).
        ranges = np.array(ranges)
        angles = np.array(angles)

        # Calculamos cuántos elementos se pueden agrupar completamente
        # en bloques del tamaño especificado. 
        # Ejemplo: si hay 103 puntos y block_size=10, se usarán 100 puntos.
        n = len(ranges) - (len(ranges) % block_size)

        # Descartamos los puntos sobrantes que no encajan en un bloque completo.
        ranges = ranges[:n]
        angles = angles[:n]

        # Reorganizamos los arrays en forma de matriz:
        # cada fila representa un bloque de "block_size" mediciones consecutivas.
        range_blocks = ranges.reshape(-1, block_size)
        angle_blocks = angles.reshape(-1, block_size)

        # Calculamos el promedio de cada bloque (por filas),
        # obteniendo un valor representativo de distancia y ángulo por bloque.
        avg_ranges = np.mean(range_blocks, axis=1)
        avg_angles = np.mean(angle_blocks, axis=1)

        # Devolvemos los promedios como dos arrays (distancias y ángulos promediados).
        return avg_ranges, avg_angles



def main(args=None):
    rclpy.init(args=args)
    node = Maze_Navigation_Node()
    rclpy.spin(node)
    rclpy.shutdown()
