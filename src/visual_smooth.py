#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import matplotlib.pyplot as plt
import math

class RealTimeTrajectoryVisualizer:
    def __init__(self):
        rospy.init_node('realtime_trajectory_visualizer')
        
        # Suscribe al tópico de la posición estimada del EKF
        rospy.Subscriber('/ekf_pose', PoseWithCovarianceStamped, self.pose_callback)
        
        # Definir los puntos de la trayectoria manualmente
        self.trajectory_points = [
    (0, 0), (0, 1), (0, 2), (0, 3), (0, 4), (0, 5), (0, 6), (0, 7), (0, 8), (0, 9), (0, 10),
    (0.1, 10.15), (0.2, 10.29), (0.3, 10.43), (0.4, 10.56), (0.5, 10.69), (0.6, 10.81), (0.7, 10.92), (0.8, 11.02), (0.9, 11.11), (1, 11.19), (1.1, 11.26), (1.2, 11.32), (1.3, 11.37), (1.4, 11.41), (1.5, 11.44),
    (1.6, 11.45), (1.7, 11.45), (1.8, 11.44), (1.9, 11.41), (2, 11.37), (2.1, 11.32), (2.2, 11.26), (2.3, 11.19), (2.4, 11.11), (2.5, 11.02), (2.6, 10.92), (2.7, 10.81), (2.8, 10.69), (2.9, 10.56), (3, 10.43), (3.1, 10.29), (3.2, 10.15), (3.3, 10), (3.4, 9.85), (3.5, 9.71), (3.6, 9.56), (3.7, 9.43), (3.8, 9.29), (3.9, 9.15),
    (4, 9), (4, 8), (4, 7), (4, 6), (4, 5), (4, 4), (4, 3), (4, 2), (4, 1), (4, 0),
    (4.1, -0.15), (4.2, -0.29), (4.3, -0.43), (4.4, -0.56), (4.5, -0.69), (4.6, -0.81), (4.7, -0.92), (4.8, -1.02), (4.9, -1.11), (5, -1.19), (5.1, -1.26), (5.2, -1.32), (5.3, -1.37), (5.4, -1.41), (5.5, -1.44),
    (5.6, -1.45), (5.7, -1.45), (5.8, -1.44), (5.9, -1.41), (6, -1.37), (6.1, -1.32), (6.2, -1.26), (6.3, -1.19), (6.4, -1.11), (6.5, -1.02), (6.6, -0.92), (6.7, -0.81), (6.8, -0.69), (6.9, -0.56), (7, -0.43), (7.1, -0.29), (7.2, -0.15), (7.3, 0), (7.4, 0.15), (7.5, 0.29), (7.6, 0.43), (7.7, 0.56), (7.8, 0.69), (7.9, 0.81),
    (8, 1), (8, 2), (8, 3), (8, 4), (8, 5), (8, 6), (8, 7), (8, 8), (8, 9), (8, 10),
    (8.1, 10.15), (8.2, 10.29), (8.3, 10.43), (8.4, 10.56), (8.5, 10.69), (8.6, 10.81), (8.7, 10.92), (8.8, 11.02), (8.9, 11.11), (9, 11.19), (9.1, 11.26), (9.2, 11.32), (9.3, 11.37), (9.4, 11.41), (9.5, 11.44),
    (9.6, 11.45), (9.7, 11.45), (9.8, 11.44), (9.9, 11.41), (10, 11.37), (10.1, 11.32), (10.2, 11.26), (10.3, 11.19), (10.4, 11.11), (10.5, 11.02), (10.6, 10.92), (10.7, 10.81), (10.8, 10.69), (10.9, 10.56), (11, 10.43), (11.1, 10.29), (11.2, 10.15), (11.3, 10), (11.4, 9.85), (11.5, 9.71), (11.6, 9.56), (11.7, 9.43), (11.8, 9.29), (11.9, 9.15),
    (12, 9), (12, 8), (12, 7), (12, 6), (12, 5), (12, 4), (12, 3), (12, 2), (12, 1), (12, 0),
    (12.1, -0.15), (12.2, -0.29), (12.3, -0.43), (12.4, -0.56), (12.5, -0.69), (12.6, -0.81), (12.7, -0.92), (12.8, -1.02), (12.9, -1.11), (13, -1.19), (13.1, -1.26), (13.2, -1.32), (13.3, -1.37), (13.4, -1.41), (13.5, -1.44),
    (13.6, -1.45), (13.7, -1.45), (13.8, -1.44), (13.9, -1.41), (14, -1.37), (14.1, -1.32), (14.2, -1.26), (14.3, -1.19), (14.4, -1.11), (14.5, -1.02), (14.6, -0.92), (14.7, -0.81), (14.8, -0.69), (14.9, -0.56), (15, -0.43), (15.1, -0.29), (15.2, -0.15), (15.3, 0), (15.4, 0.15), (15.5, 0.29), (15.6, 0.43), (15.7, 0.56), (15.8, 0.69), (15.9, 0.81),
    (16, 1), (16, 2), (16, 3), (16, 4), (16, 5), (16, 6), (16, 7), (16, 8), (16, 9), (16, 10)
]
        
        # Listas para almacenar las posiciones
        self.estimated_positions = []
        self.desired_positions = self.trajectory_points
        
        # Frecuencia de bucle
        self.rate = rospy.Rate(30)  # 30Hz
        
        # Variable para almacenar la posición estimada del robot
        self.current_pose = None

        # Configuración de Matplotlib para el modo interactivo
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.estimated_line, = self.ax.plot([], [], label='Estimated Trajectory (EKF)', linestyle='-')
        self.desired_line, = self.ax.plot([], [], 'ro', label='Desired Points')
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_title('Real-Time Trajectory Visualization')
        self.ax.legend()
        self.ax.grid(True)

    def pose_callback(self, msg):
        # Actualizar la posición estimada del robot
        self.current_pose = msg.pose.pose.position
        self.estimated_positions.append((self.current_pose.x, self.current_pose.y))
        
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
