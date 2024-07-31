#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import math
import tf
import numpy as np

class SquareTrajectoryController:
    def __init__(self):
        rospy.init_node('square_trajectory_controller')
        
        rospy.Subscriber('/ekf_pose', PoseWithCovarianceStamped, self.pose_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.max_linear_speed = 0.4  # m/s
        self.max_angular_speed = 1.0  # rad/s
        
        self.rate = rospy.Rate(60)  # 60Hz
        
        self.current_pose = None
        self.current_orientation = None
        
        self.kp_linear = 0.5   #4.0
        self.kd_linear = 0.5    #3.0
        self.kp_angular = 2.0   #6.5
        self.kd_angular = 1.0   #3.5
        
        self.side_length = 10.0  # Longitud del lado del cuadrado
        
        # Variables para almacenar errores previos
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_angular_error = 0.0
        
        # Generar puntos para la trayectoria en forma de cuadrado
        self.generate_square_trajectory()
        
        # Índice del punto objetivo en la trayectoria
        self.target_index = 0

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
        num_points_per_side = 10  # Número de puntos por lado
        
        # Generar puntos para el primer lado del cuadrado
        for i in range(num_points_per_side):
            x = i * self.side_length / num_points_per_side
            y = 0.0
            self.square_trajectory_points.append((x, y))
        
        # Generar puntos para el segundo lado del cuadrado
        for i in range(num_points_per_side):
            x = self.side_length
            y = i * self.side_length / num_points_per_side
            self.square_trajectory_points.append((x, y))
        
        # Generar puntos para el tercer lado del cuadrado
        for i in range(num_points_per_side):
            x = self.side_length - i * self.side_length / num_points_per_side
            y = self.side_length
            self.square_trajectory_points.append((x, y))
        
        # Generar puntos para el cuarto lado del cuadrado
        for i in range(num_points_per_side):
            x = 0.0
            y = self.side_length - i * self.side_length / num_points_per_side
            self.square_trajectory_points.append((x, y))

    def run(self):
        rospy.loginfo("Square Trajectory Controller started.")
        while not rospy.is_shutdown():
            if self.current_pose and self.current_orientation is not None:
                current_x = self.current_pose.x
                current_y = self.current_pose.y
                current_theta = self.current_orientation

                # Encontrar el punto objetivo actual en la trayectoria
                target_x, target_y = self.square_trajectory_points[self.target_index]
                
                # Calcular los errores de posición
                error_x = target_x - current_x
                error_y = target_y - current_y
                error_distance = math.sqrt(error_x**2 + error_y**2)
                
                # Transformar los errores al marco de referencia del robot
                transformed_error_x = error_x * math.cos(current_theta + math.pi/2) + error_y * math.sin(current_theta + math.pi/2)
                transformed_error_y = -error_x * math.sin(current_theta + math.pi/2) + error_y * math.cos(current_theta + math.pi/2)
                
                # Calcular el ángulo deseado
                desired_orientation = math.atan2(transformed_error_y, transformed_error_x)
                error_angular = desired_orientation
                
                # Calcular errores derivados
                derivative_error_distance = error_distance - math.sqrt(self.prev_error_x**2 + self.prev_error_y**2)
                derivative_error_angular = error_angular - self.prev_angular_error
                
                # Velocidades lineal y angular deseadas con control PD
                linear_velocity = self.kp_linear * error_distance + self.kd_linear * derivative_error_distance
                angular_velocity = self.kp_angular * error_angular + self.kd_angular * derivative_error_angular
                
                # Limitar las velocidades dentro de los máximos permitidos
                linear_velocity = max(min(linear_velocity, self.max_linear_speed), -self.max_linear_speed)
                angular_velocity = max(min(angular_velocity, self.max_angular_speed), -self.max_angular_speed)
                
                # Crear mensaje Twist con las velocidades calculadas
                twist_cmd = Twist()
                twist_cmd.linear.x = linear_velocity
                twist_cmd.angular.z = angular_velocity
                
                # Publicar comando de velocidad
                self.cmd_vel_pub.publish(twist_cmd)
                
                # Actualizar índice del punto objetivo si se alcanza el actual
                if error_distance < 0.1:
                    self.target_index = (self.target_index + 1) % len(self.square_trajectory_points)
                
                # Almacenar errores previos
                self.prev_error_x = transformed_error_x
                self.prev_error_y = transformed_error_y
                self.prev_angular_error = error_angular
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = SquareTrajectoryController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
