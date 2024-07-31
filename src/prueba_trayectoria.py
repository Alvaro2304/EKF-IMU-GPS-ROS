#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import math
import tf

class STrajectoryController:
    def __init__(self):
        rospy.init_node('s_trajectory_controller')
        
        # Suscribe al tópico de la posición estimada del EKF
        rospy.Subscriber('/ekf_pose', PoseWithCovarianceStamped, self.pose_callback)
        
        # Publica en el tópico de comandos de velocidad
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Parámetros de la trayectoria en forma de S
        self.amplitude = 3.0  # Amplitud de la onda sinusoidal (4 metros de ancho total)
        self.period = 6.0     # Período de la onda sinusoidal (4 metros de largo total)
        
        # Velocidades máximas del robot diferencial
        self.max_linear_speed = 0.3  # m/s
        self.max_angular_speed = 0.3  # rad/s
        
        # Frecuencia de bucle
        self.rate = rospy.Rate(20)  # 10Hz
        
        # Variable para almacenar la posición estimada del robot
        self.current_pose = None
        self.current_orientation = None
        
        # Variables de control PID
        self.kp_linear = 0.5
        self.kp_angular = 2.5

    def pose_callback(self, msg):
        # Actualizar la posición estimada del robot
        self.current_pose = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        self.current_orientation = self.quaternion_to_euler(orientation_q)

    def quaternion_to_euler(self, q):
        # Convertir la orientación del cuaternión a ángulos de Euler usando tf.transformations
        orientation_list = [q.x, q.y, q.z, q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        return yaw  

    def run(self):
        rospy.loginfo("S Trajectory Controller started.")
        while not rospy.is_shutdown():
            if self.current_pose and self.current_orientation is not None:
                # Obtener la posición actual del robot desde el filtro EKF
                current_x = self.current_pose.x
                current_y = self.current_pose.y
                current_theta = self.current_orientation
                
                # Calcular la posición deseada en la trayectoria en forma de S
                target_x = current_x +0.1  # Avanzar un pequeño paso en X
                target_y = self.amplitude * math.sin((math.pi / self.period) * current_x)
                
                # Calcular el error de posición
                error_x = target_x - current_x
                error_y = target_y - current_y
                
                # Calcular el ángulo deseado (orientación hacia el siguiente punto en la S)
                desired_orientation = math.atan2(error_y, error_x)
                
                # Calcular la velocidad lineal y angular deseada utilizando control PID
                distance = math.sqrt(error_x**2 + error_y**2)
                linear_velocity = self.kp_linear * distance
                
                angular_error = desired_orientation - current_theta
                # Ajuste para que el angular_error esté en el rango [-pi, pi]
                if (angular_error > math.pi):
                    angular_error -= 2 * math.pi
                elif (angular_error < -math.pi):
                    angular_error += 2 * math.pi
                
                angular_velocity = self.kp_angular * angular_error
                
                # Limitar las velocidades dentro de los máximos permitidos
                linear_velocity = max(min(linear_velocity, self.max_linear_speed), -self.max_linear_speed)
                angular_velocity = max(min(angular_velocity, self.max_angular_speed), -self.max_angular_speed)
                
                # Crear el mensaje Twist con las velocidades calculadas
                twist_cmd = Twist()
                twist_cmd.linear.x = linear_velocity
                twist_cmd.angular.z = angular_velocity
                
                # Publicar el comando de velocidad
                self.cmd_vel_pub.publish(twist_cmd)
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = STrajectoryController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
