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
            (0, 0.0),
            (0, 0.5),
            (0, 1.0),
            (0, 1.5),
            (0, 2.0),
            (0, 2.5),
            (0, 3),
            (0, 3.5),
            (0, 4),
            (0, 4.5),
            (0, 5),
            (0, 5.5),
            (0, 6),
            (0, 6.5),
            (0, 7),
            (0, 7.5),
            (0, 8),
            (0, 8.5),
            (0, 9),
            (0.2, 9),
            (0.4, 9),
            (0.6, 9),
            (0.8, 9),
            (1.0, 9),
            (1.2, 9),
            (1.4, 9),
            (1.6, 9),
            (1.8, 9),
            (2.0, 9),
            (2.2, 9),
            (2.4, 9),
            (2.6, 9),
            (2.8, 9),
            (3, 9),
            (3, 8.5),
            (3, 8),
            (3, 7.5),
            (3, 7),
            (3, 6.5),
            (3, 6),
            (3, 5.5),
            (3, 5),
            (3, 4.5),
            (3, 4),
            (3, 3.5),
            (3, 3),
            (3, 2.5),
            (3, 2),
            (3, 1.5),
            (3, 1),
            (3, 0.5),
            (3, 0),
            (3.2, 0),
            (3.4, 0),
            (3.6, 0),
            (3.8, 0),
            (4.0, 0),
            (4.2, 0),
            (4.4, 0),
            (4.6, 0),
            (4.8, 0),
            (5.0, 0),
            (5.2, 0),
            (5.4, 0),
            (5.6, 0),
            (5.8, 0),
            (6, 0),
            (6, 0.5),
            (6, 1),
            (6, 1.5),
            (6, 2),
            (6, 2.5),
            (6, 3),
            (6, 3.5),
            (6, 4),
            (6, 4.5),
            (6, 5),
            (6, 5.5),
            (6, 6),
            (6, 6.5),
            (6, 7),
            (6, 7.5),
            (6, 8),
            (6, 8.5),
            (6, 9)
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
