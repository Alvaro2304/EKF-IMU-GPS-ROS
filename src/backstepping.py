#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import math
import tf
import numpy as np

class BacksteppingController:
    def __init__(self):
        rospy.init_node('backstepping_controller')
        
        rospy.Subscriber('/ekf_pose', PoseWithCovarianceStamped, self.pose_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.max_linear_speed = 0.4  # m/s
        self.max_angular_speed = 1.0  # rad/s
        
        self.rate = rospy.Rate(60)  # 60Hz
        
        self.current_pose = None
        self.current_orientation = None
        
        # Ganancias del controlador
        self.k1 = 0.5
        self.k2 = 2.0
        
        self.side_length = 10.0  # Longitud del lado del cuadrado
        
        # Generar puntos para la trayectoria en forma de cuadrado
        self.generate_square_trajectory()
        
        self.target_index = 0  # Índice del punto de la trayectoria objetivo

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        self.current_orientation = self.quaternion_to_euler(orientation_q)

    def quaternion_to_euler(self, q):
        orientation_list = [q.x, q.y, q.z, q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        return yaw

    def generate_square_trajectory(self):
        self.square_trajectory_points = []
        num_points_per_side = 20
        offset = 0.0  # Desplazamiento para que la trayectoria no comience en el origen
        
        # Generar puntos para el primer lado del cuadrado
        for i in range(num_points_per_side):
            x = offset + i * self.side_length / num_points_per_side
            y = offset
            self.square_trajectory_points.append((x, y))
        
        # Generar puntos para el segundo lado del cuadrado
        for i in range(num_points_per_side):
            x = offset + self.side_length
            y = offset + i * self.side_length / num_points_per_side
            self.square_trajectory_points.append((x, y))
        
        # Generar puntos para el tercer lado del cuadrado
        for i in range(num_points_per_side):
            x = offset + self.side_length - i * self.side_length / num_points_per_side
            y = offset + self.side_length
            self.square_trajectory_points.append((x, y))
        
        # Generar puntos para el cuarto lado del cuadrado
        for i in range(num_points_per_side):
            x = offset
            y = offset + self.side_length - i * self.side_length / num_points_per_side
            self.square_trajectory_points.append((x, y))

    def run(self):
        rospy.loginfo("Backstepping Controller started.")
        while not rospy.is_shutdown():
            if self.current_pose and self.current_orientation is not None:
                current_x = self.current_pose.x
                current_y = self.current_pose.y
                current_theta = self.current_orientation

                # Obtener el punto objetivo actual
                target_x, target_y = self.square_trajectory_points[self.target_index]
                
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
                
                # Controladores de backstepping
                v = self.k1 * error_distance
                omega = self.k2 * error_theta + math.atan2(transformed_error_y, transformed_error_x)
                
                # Limitar las velocidades dentro de los máximos permitidos
                v = max(min(v, self.max_linear_speed), -self.max_linear_speed)
                omega = max(min(omega, self.max_angular_speed), -self.max_angular_speed)
                
                # Crear mensaje Twist con las velocidades calculadas
                twist_cmd = Twist()
                twist_cmd.linear.x = v
                twist_cmd.angular.z = omega
                
                # Publicar comando de velocidad
                self.cmd_vel_pub.publish(twist_cmd)
                
                # Verificar si el punto objetivo ha sido alcanzado y actualizar el índice del punto objetivo
                if error_distance < 0.1:
                    self.target_index = (self.target_index + 1) % len(self.square_trajectory_points)
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = BacksteppingController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
