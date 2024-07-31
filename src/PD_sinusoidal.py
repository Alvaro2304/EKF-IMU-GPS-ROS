#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Point
import math
import tf
import time

class BacksteppingController:
    def __init__(self):
        rospy.init_node('backstepping_controller')
        
        rospy.Subscriber('/ekf_pose', PoseWithCovarianceStamped, self.pose_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.target_point_pub = rospy.Publisher('/target_point', Point, queue_size=10)  # Publicador para los puntos objetivo
        
        self.max_linear_speed = 0.4  # m/s
        self.max_angular_speed = 1.0  # rad/s
        
        self.rate = rospy.Rate(100)  # 100Hz
        
        self.current_pose = None
        self.current_orientation = None
        
        # Ganancias del controlador
        self.k1 = 0.2  # Ganancia proporcional para la velocidad lineal
        self.k2 = 2.0  # Ganancia proporcional para la velocidad angular
        self.kd_v = 0.1  # Ganancia derivativa para la velocidad lineal
        self.kd_omega = 1.0  # Ganancia derivativa para la velocidad angular
        
        # Parámetros de la trayectoria sinusoidal
        self.amplitude = 5.0  # Amplitud de la onda sinusoidal
        self.wavelength = 2.0  # Longitud de onda (periodo)
        self.num_points = 500  # Número de puntos en la trayectoria
        
        # Generar los puntos de la trayectoria sinusoidal
        self.trajectory_points = self.generate_sinusoidal_trajectory()
        
        self.target_index = 0  # Índice del punto de la trayectoria objetivo
        
        self.prev_error_distance = 0  # Error de distancia previo
        self.prev_error_theta = 0  # Error de orientación previo
        self.prev_time = time.time()  # Tiempo previo para cálculo derivativo

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        self.current_orientation = self.quaternion_to_euler(orientation_q)

    def quaternion_to_euler(self, q):
        orientation_list = [q.x, q.y, q.z, q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        return yaw

    def generate_sinusoidal_trajectory(self):
        num_cycles = 6  # Número de ciclos de la sinusoide
        sinusoidal_trajectory_points = []
        total_length = num_cycles * self.wavelength  # Longitud total de la trayectoria
        
        for i in range(self.num_points):
            x = i * total_length / (self.num_points - 1)
            y = self.amplitude * math.sin(2 * math.pi * x / self.wavelength)
            sinusoidal_trajectory_points.append((x, y))
        
        return sinusoidal_trajectory_points

    def publish_target_point(self, x, y):
        point_msg = Point()
        point_msg.x = x
        point_msg.y = y
        point_msg.z = 0.0
        self.target_point_pub.publish(point_msg)

    def run(self):
        rospy.loginfo("Backstepping Controller started.")
        self.publish_target_point(*self.trajectory_points[self.target_index])  # Publica el primer punto objetivo
        while not rospy.is_shutdown():
            if self.current_pose and self.current_orientation is not None:
                current_x = self.current_pose.x
                current_y = self.current_pose.y
                current_theta = self.current_orientation

                # Obtener el punto objetivo actual
                target_x, target_y = self.trajectory_points[self.target_index]
                
                # Calcular errores
                error_x = target_x - current_x
                error_y = target_y - current_y
                error_distance = math.sqrt(error_x**2 + error_y**2)
                
                # Transformar los errores al marco de referencia del robot
                transformed_error_x = error_x * math.cos(current_theta + math.pi/2) + error_y * math.sin(current_theta + math.pi/2)
                transformed_error_y = -error_x * math.sin(current_theta + math.pi/2) + error_y * math.cos(current_theta + math.pi/2)
                
                # Calcular orientación deseada
                theta_d = math.atan2(transformed_error_y, transformed_error_x)
                error_theta = theta_d
                
                # Calcular la derivada de los errores
                current_time = time.time()
                dt = current_time - self.prev_time
                d_error_distance = (error_distance - self.prev_error_distance) / dt if dt > 0 else 0
                d_error_theta = (error_theta - self.prev_error_theta) / dt if dt > 0 else 0
                
                # Controladores de backstepping (PD para velocidad lineal y angular)
                v = self.k1 * error_distance + self.kd_v * d_error_distance
                omega = self.k2 * error_theta + self.kd_omega * d_error_theta
                
                # Limitar las velocidades dentro de los máximos permitidos
                v = max(min(v, self.max_linear_speed), -self.max_linear_speed)
                omega = max(min(omega, self.max_angular_speed), -self.max_angular_speed)
                
                # Crear mensaje Twist con las velocidades calculadas
                twist_cmd = Twist()
                twist_cmd.linear.x = v
                twist_cmd.angular.z = omega
                
                # Publicar comando de velocidad
                self.cmd_vel_pub.publish(twist_cmd)
                
                # Actualizar valores previos para el siguiente ciclo
                self.prev_error_distance = error_distance
                self.prev_error_theta = error_theta
                self.prev_time = current_time
                
                # Verificar si el punto objetivo ha sido alcanzado y actualizar el índice del punto objetivo
                if error_distance < 0.5:
                    self.target_index += 1
                    if self.target_index >= len(self.trajectory_points):
                        # Detener el robot
                        stop_cmd = Twist()
                        stop_cmd.linear.x = 0
                        stop_cmd.angular.z = 0
                        self.cmd_vel_pub.publish(stop_cmd)
                        rospy.loginfo("Last point reached. Stopping the robot.")
                        break
                    else:
                        self.publish_target_point(*self.trajectory_points[self.target_index])  # Publica el siguiente punto objetivo
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = BacksteppingController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
