#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import math
import tf

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
        
        self.zigzag_length = 10.0  # Longitud del zigzag
        
        # Generar puntos para la trayectoria en forma de zigzag
        self.generate_zigzag_trajectory()
        
        self.target_index = 0  # Índice del punto de la trayectoria objetivo

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        self.current_orientation = self.quaternion_to_euler(orientation_q)

    def quaternion_to_euler(self, q):
        orientation_list = [q.x, q.y, q.z, q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        return yaw

    def generate_zigzag_trajectory(self):
        self.zigzag_trajectory_points = []
        num_zigzags = 6  # Número de zigzags
        segment_length = self.zigzag_length / num_zigzags
        up = True
        
        for i in range(num_zigzags+1):
            # Generar puntos para el segmento hacia arriba o hacia abajo alternando
            if up:
                self.zigzag_trajectory_points.append((i * segment_length, 0))
                self.zigzag_trajectory_points.append((i * segment_length, self.zigzag_length))
            else:
                self.zigzag_trajectory_points.append((i * segment_length, self.zigzag_length))
                self.zigzag_trajectory_points.append((i * segment_length, 0))
            up = not up

    def run(self):
        rospy.loginfo("Backstepping Controller started.")
        while not rospy.is_shutdown():
            if self.current_pose and self.current_orientation is not None:
                current_x = self.current_pose.x
                current_y = self.current_pose.y
                current_theta = self.current_orientation

                # Obtener el punto objetivo actual
                target_x, target_y = self.zigzag_trajectory_points[self.target_index]
                
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
                omega = self.k2 * error_theta 
                
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
                    self.target_index += 1
                    if self.target_index >= len(self.zigzag_trajectory_points):
                        # Detener el robot
                        stop_cmd = Twist()
                        stop_cmd.linear.x = 0
                        stop_cmd.angular.z = 0
                        self.cmd_vel_pub.publish(stop_cmd)
                        rospy.loginfo("Last point reached. Stopping the robot.")
                        break
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = BacksteppingController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
