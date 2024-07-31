#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import math
import tf
import numpy as np

class STrajectoryController:
    def __init__(self):
        rospy.init_node('s_trajectory_controller')
        
        rospy.Subscriber('/ekf_pose', PoseWithCovarianceStamped, self.pose_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
                                                #valores tuneando
        self.max_linear_speed = 0.4  # m/s           0.3                      | 0.3
        self.max_angular_speed = 1.0  # rad/s        0.3                      | 0.5
        
        self.rate = rospy.Rate(60)  # 30Hz -- probé con varias frecuencias, más baja es respuesta mas suave pero reacciona lento
        
        
        self.current_pose = None
        self.current_orientation = None
        
        self.kp_linear = 4.0                        #0.4  0.3  0.8  0.8  4.0  | 4.0   5.0   4.0
        self.kd_linear = 3.0                        #0.4  0.3  0.8  0.8  4.0  | 4.0   4.0   3.0
        self.kp_angular = 6.5                      #3.5  2.5  8.5 12.5  17.5 | 30.5  30.5  10.5 6.5
        self.kd_angular = 3.5                      #3.5  1.5  7.5  7.5  15.5 | 25.5  25.5   5.5 2.5
        
        self.amplitude = 4.0
        self.period = 6.00
        self.num_de_ciclos=3
        
        # Variables para almacenar errores previos
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_angular_error = 0.0
        
        # Generar puntos para la trayectoria en forma de S
        self.generate_s_trajectory()

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        self.current_orientation = self.quaternion_to_euler(orientation_q)

    def quaternion_to_euler(self, q):
        orientation_list = [q.x, q.y, q.z, q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        return yaw

    def generate_s_trajectory(self):
        self.s_trajectory_points = []
        num_points = 2000
        for i in range(num_points):
            x = i * self.num_de_ciclos*self.period / num_points  # Distribuir los puntos uniformemente en el periodo
            y = self.amplitude * math.sin((2 * math.pi / self.period) * x)
            self.s_trajectory_points.append((x, y))

    def run(self):
        rospy.loginfo("S Trajectory Controller started.")
        while not rospy.is_shutdown():
            if self.current_pose and self.current_orientation is not None:
                current_x = self.current_pose.x
                current_y = self.current_pose.y
                current_theta = self.current_orientation

                # Encontrar el punto más cercano en la trayectoria en forma de S
                min_distance = float('inf')
                closest_point = None
                for point in self.s_trajectory_points:
                    x_s, y_s = point
                    distance = math.sqrt((current_x - x_s)**2 + (current_y - y_s)**2)
                    if distance < min_distance:
                        min_distance = distance
                        closest_point = point
                
                # Usar el punto más cercano como target
                target_x, target_y = closest_point
                
                # Calcular los errores
                error_x = target_x - current_x
                error_y = target_y - current_y
                error_distance = math.sqrt(error_x**2 + error_y**2)
                
                # Calcular el ángulo deseado
                desired_orientation = math.atan2(error_y, error_x)
                
                # Calcular errores derivados
                error_angular = desired_orientation - current_theta
                if error_angular > math.pi:
                    error_angular -= 2 * math.pi
                elif error_angular < -math.pi:
                    error_angular += 2 * math.pi
                
                # Velocidades lineal y angular deseadas con control PD
                linear_velocity = (self.kp_linear * error_distance) + (self.kd_linear * (error_distance - math.sqrt(self.prev_error_x**2 + self.prev_error_y**2)))
                angular_velocity = (self.kp_angular * error_angular) + (self.kd_angular * (error_angular - self.prev_angular_error))
                
                # Limitar las velocidades dentro de los máximos permitidos
                linear_velocity = max(min(linear_velocity, self.max_linear_speed), -self.max_linear_speed)
                angular_velocity = max(min(angular_velocity, self.max_angular_speed), -self.max_angular_speed)
                
                # Crear mensaje Twist con las velocidades calculadas
                twist_cmd = Twist()
                twist_cmd.linear.x = linear_velocity
                twist_cmd.angular.z = angular_velocity
                
                # Publicar comando de velocidad
                self.cmd_vel_pub.publish(twist_cmd)
                
                # Almacenar errores previos
                self.prev_error_x = error_x
                self.prev_error_y = error_y
                self.prev_angular_error = error_angular
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = STrajectoryController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
