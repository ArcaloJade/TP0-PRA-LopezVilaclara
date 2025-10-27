#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math
import time

class Part0Node(Node):

    def __init__(self):
        super().__init__('p0_node')

        # Flag para cambiar los robots, si es False uso el turtlebot4
        self.declare_parameter('is_turtlebot3', True)
        self.is_turtlebot3 = self.get_parameter('is_turtlebot3').get_parameter_value().bool_value

        # Velocidades y umbrales
        self.max_linear_speed = 0.5   # m/s
        self.rotation_speed = 1.0     # rad/s
        self.obstacle_distance_thresh = 0.5  # m

        # Ángulo de rotación objetivo (grados)
        self.base_turn_deg = 110.0

        # Matriz de transformación
        if self.is_turtlebot3:
            self.lidar_tf = np.eye(4)
        else:
            # LIDAR está rotado 90 grados y desplazado!!!
            self.lidar_tf = np.array([
                [0.0, -1.0,  0.0, -0.04],
                [1.0,  0.0,  0.0,  0.0 ],
                [0.0,  0.0,  1.0,  0.193],
                [0.0,  0.0,  0.0,  1.0 ]
            ])

        # Si es Turtlebot4, ajusto el ángulo por la deceleración (20.5 grados)
        if self.is_turtlebot3:
            self.turn_deg = self.base_turn_deg
        else:
            self.turn_deg = self.base_turn_deg - 20.5
        self.turn_rad = math.radians(self.turn_deg)

        # Defino estados 'ROTATING' y 'MOVING'
        self.state = 'MOVING'
        self.rotation_end_time = None # Esto sería el tiempo en el q termina la rotación

        # Tópicos
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/calc_odom', self.odom_callback, 10)

        # Timer
        self.control_period = 0.1  # segs
        self.timer = self.create_timer(self.control_period, self.control_loop)

        # Último scan para procesar en el loop
        self.last_scan = None

        self.get_logger().info(f"Part0Node iniciado. is_turtlebot3={self.is_turtlebot3}, turn_deg={self.turn_deg:.2f}")



    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        self.get_logger().debug(f"Odometría: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}")



    def scan_callback(self, msg: LaserScan):
        self.last_scan = msg



    def control_loop(self):
        cmd = Twist()

        if self.state == 'MOVING':
            # Reviso si hay obstáculo adelante
            obstacle = False
            if self.last_scan is not None:
                obstacle = self.check_obstacle_in_front(self.last_scan)

            if obstacle:
                # Freno y preparo rotación
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)

                # Empiezo a rotar
                rot_duration = abs(self.turn_rad) / self.rotation_speed  # segs
                self.rotation_end_time = time.monotonic() + rot_duration
                self.state = 'ROTATING'
                return
            else:
                # Avanzo
                cmd.linear.x = self.max_linear_speed
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                return

        elif self.state == 'ROTATING':
            remaining = 0.0
            if self.rotation_end_time is not None:
                remaining = self.rotation_end_time - time.monotonic()

            if remaining > 0.0:
                cmd.linear.x = 0.0
                # Giro hacia la izquierda, si lo pongo negativo va a la derecha
                cmd.angular.z = self.rotation_speed  # rad/s
                self.cmd_pub.publish(cmd)
                return
            else:
                # Termino de rotar y paso a avanzar
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                self.state = 'MOVING'
                self.rotation_end_time = None
                return
            
        else:
            # Esto no debería pasar
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            self.get_logger().warn(f"ERROR: estado desconocido??? whats!!: {self.state}.")



    def check_obstacle_in_front(self, scan: LaserScan) -> bool:
        n = len(scan.ranges)
        angles = [scan.angle_min + i * scan.angle_increment for i in range(n)]

        has_intensity = hasattr(scan, 'intensities') and len(scan.intensities) == n

        for i in range(n):
            r = scan.ranges[i]
            if math.isinf(r) or math.isnan(r):
                continue

            # Filtro las intensidades 0.0 en turtlebot4
            if (not self.is_turtlebot3) and has_intensity:
                if scan.intensities[i] == 0.0:
                    continue

            # Punto en la terna del LIDAR
            theta = angles[i]
            x_l = r * math.cos(theta)
            y_l = r * math.sin(theta)
            point_l = np.array([x_l, y_l, 0.0, 1.0])  # z=0

            # Transformo a la terna del robot
            p_base = self.lidar_tf.dot(point_l)
            x_b = float(p_base[0])
            y_b = float(p_base[1])
            dist = math.hypot(x_b, y_b)

            # Es obstáculo si está adelante (x_b > 0) y a distancia menor al umbral
            if x_b > 0.0 and dist <= self.obstacle_distance_thresh:
                return True

        return False

    

def main(args=None):
    rclpy.init(args=args)
    node = Part0Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo detenido por teclado")
    finally:
        # Freno antes de apagar el nodo, calculo q hace falta hacerlo?
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        node.cmd_pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()