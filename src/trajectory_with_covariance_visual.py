#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import math

class RealTimeTrajectoryVisualizer:
    def __init__(self):
        rospy.init_node('realtime_trajectory_visualizer')
        
        # Suscribe al tópico de la posición estimada del EKF
        rospy.Subscriber('/ekf_pose', PoseWithCovarianceStamped, self.pose_callback)
        
        # Parámetros de la trayectoria zigzag
        self.zigzag_length = 10.0  # Longitud total de la trayectoria zigzag
        self.num_zigzags = 3  # Número de zigzags
        
        # Listas para almacenar las posiciones
        self.estimated_positions = []
        self.desired_positions = self.generate_zigzag_trajectory()
        
        # Frecuencia de bucle
        self.rate = rospy.Rate(30)  # 30Hz
        
        # Variable para almacenar la posición estimada del robot
        self.current_pose = None

        # Configuración de Matplotlib para el modo interactivo
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.estimated_line, = self.ax.plot([], [], label='Estimated Trajectory (EKF)', linestyle='-')
        self.desired_line, = self.ax.plot([], [], 'ro', label='Desired Points (Zigzag)')
        self.covariance_ellipse = Ellipse((0, 0), width=0, height=0, angle=0, color='red', fill=False, label='Covariance Ellipse')
        self.ax.add_patch(self.covariance_ellipse)
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_title('Real-Time Trajectory Visualization')
        self.ax.legend()
        self.ax.grid(True)

    def pose_callback(self, msg):
        # Actualizar la posición estimada del robot
        self.current_pose = msg.pose.pose.position
        self.estimated_positions.append((self.current_pose.x, self.current_pose.y))
        
        # Extraer y actualizar la elipse de covarianza
        covariance_matrix = np.array(msg.pose.covariance).reshape(6, 6)
        self.update_covariance_ellipse(covariance_matrix[:2, :2], self.current_pose.x, self.current_pose.y)

    def update_covariance_ellipse(self, covariance_matrix, mean_x, mean_y):
        # Calcular los valores propios y los vectores propios de la matriz de covarianza
        eigenvalues, eigenvectors = np.linalg.eigh(covariance_matrix)
        
        # La elipse se define en términos de la longitud del eje mayor y menor
        axis_length = np.sqrt(eigenvalues) * 2  # Para una elipse de 1 desviación estándar
        
        # La orientación de la elipse está dada por los vectores propios
        angle = np.arctan2(*eigenvectors[:,0][::-1])
        
        # Actualizar la elipse de covarianza
        self.covariance_ellipse.width = axis_length[0]
        self.covariance_ellipse.height = axis_length[1]
        self.covariance_ellipse.angle = np.degrees(angle)
        self.covariance_ellipse.center = (mean_x, mean_y)

    def generate_zigzag_trajectory(self):
        zigzag_trajectory_points = []
        num_zigzags = self.num_zigzags  # Número de zigzags
        segment_length = self.zigzag_length / num_zigzags
        up = True
        
        for i in range(num_zigzags+1):
            x = i * segment_length
            if up:
                zigzag_trajectory_points.append((x, 0))
                zigzag_trajectory_points.append((x, self.zigzag_length))
            else:
                zigzag_trajectory_points.append((x, self.zigzag_length))
                zigzag_trajectory_points.append((x, 0))
            up = not up

        # Agregar el último punto final
        zigzag_trajectory_points.append((self.zigzag_length, 0))

        return zigzag_trajectory_points

    def update_plot(self):
        if self.estimated_positions:
            estimated_x, estimated_y = zip(*self.estimated_positions)
            self.estimated_line.set_data(estimated_x, estimated_y)
        
        if self.desired_positions:
            desired_x, desired_y = zip(*self.desired_positions)
            self.desired_line.set_data(desired_x, desired_y)
        
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def run(self):
        rospy.loginfo("Real-Time Trajectory Visualizer started.")
        while not rospy.is_shutdown():
            self.update_plot()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        visualizer = RealTimeTrajectoryVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass
