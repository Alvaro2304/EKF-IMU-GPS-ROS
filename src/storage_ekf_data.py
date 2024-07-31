#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import os
from datetime import datetime
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
import roslib

class EKFDataLogger:
    def __init__(self):
        self.ekf_pose_coords = None
        self.ekf_covariance = None
        self.target_point = None

        rospy.init_node('ekf_data_logger', anonymous=True)

        # Suscribirse a los tópicos de EKF y de puntos objetivo
        self.ekf_sub = rospy.Subscriber("/ekf_pose", PoseWithCovarianceStamped, self.ekf_pose_callback)
        self.target_point_sub = rospy.Subscriber("/target_point", Point, self.target_point_callback)

        # Obtener la ruta del paquete
        package_path = roslib.packages.get_pkg_dir('my_robot_ekf')
        data_path = os.path.join(package_path, 'data')
        if not os.path.exists(data_path):
            os.makedirs(data_path)

        # Crear archivo de datos con timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.file_path = os.path.join(data_path, "ekf_data_{}.txt".format(timestamp))

        # Escribir encabezado en el archivo
        with open(self.file_path, 'w') as file:
            file.write("Timestamp, EKF_X, EKF_Y, Covariance[0,0], Covariance[0,1], Covariance[1,0], Covariance[1,1], Target_X, Target_Y\n")

        self.rate = rospy.Rate(10)  # Tasa de muestreo de 10Hz

        # Loop principal
        while not rospy.is_shutdown():
            if self.ekf_pose_coords and self.ekf_covariance and self.target_point:
                self.log_data()
            self.rate.sleep()

    def ekf_pose_callback(self, data):
        self.ekf_pose_coords = (data.pose.pose.position.x, data.pose.pose.position.y)
        self.ekf_covariance = data.pose.covariance

    def target_point_callback(self, data):
        self.target_point = (data.x, data.y)

    def log_data(self):
        ekf_x, ekf_y = self.ekf_pose_coords
        cov_matrix = self.ekf_covariance

        # Extraer las componentes relevantes de la matriz de covarianza (para simplificación, consideramos las 2D)
        cov_00 = cov_matrix[0]
        cov_01 = cov_matrix[1]
        cov_10 = cov_matrix[6]
        cov_11 = cov_matrix[7]

        target_x, target_y = self.target_point

        # Escribir datos en el archivo
        with open(self.file_path, 'a') as file:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            file.write("{}, {}, {}, {}, {}, {}, {}, {}, {}\n".format(timestamp, ekf_x, ekf_y, cov_00, cov_01, cov_10, cov_11, target_x, target_y))

        rospy.loginfo("EKF Coordinates: X: {}, Y: {}, Cov: [{}, {}, {}, {}], Target Point: X: {}, Y: {}".format(ekf_x, ekf_y, cov_00, cov_01, cov_10, cov_11, target_x, target_y))

if __name__ == '__main__':
    try:
        EKFDataLogger()
    except rospy.ROSInterruptException:
        pass
