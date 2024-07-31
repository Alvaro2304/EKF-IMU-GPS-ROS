#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os
from datetime import datetime
from geometry_msgs.msg import Twist
import roslib

class VelocityDataLogger:
    def __init__(self):
        rospy.init_node('velocity_data_logger', anonymous=True)

        # Suscribirse al tópico de /cmd_vel
        self.vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

        # Obtener la ruta del paquete
        package_path = roslib.packages.get_pkg_dir('my_robot_ekf')
        data_path = os.path.join(package_path, 'data')
        if not os.path.exists(data_path):
            os.makedirs(data_path)

        # Crear archivo de datos con timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.file_path = os.path.join(data_path, "velocity_data_{}.txt".format(timestamp))

        # Escribir encabezado en el archivo
        with open(self.file_path, 'w') as file:
            file.write("Timestamp, Linear_Velocity, Angular_Velocity\n")

        self.rate = rospy.Rate(10)  # Tasa de muestreo de 10Hz

        # Loop principal
        while not rospy.is_shutdown():
            rospy.spin()

    def cmd_vel_callback(self, msg):
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Escribir datos en el archivo
        with open(self.file_path, 'a') as file:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            file.write("{}, {}, {}\n".format(timestamp, linear_velocity, angular_velocity))

        rospy.loginfo("Linear Velocity: {}, Angular Velocity: {}".format(linear_velocity, angular_velocity))

if __name__ == '__main__':
    try:
        VelocityDataLogger()
    except rospy.ROSInterruptException:
        pass
