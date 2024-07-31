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
        
        # Parámetros de la trayectoria sinusoidal
        self.amplitude = 4.5  # Amplitud de la onda sinusoidal
        self.wavelength = 6.0  # Longitud de onda (periodo)
        self.num_points = 70  # Número de puntos en la trayectoria
        
        # Listas para almacenar las posiciones
        self.estimated_positions = []
        self.desired_positions = self.generate_sinusoidal_trajectory()
        
        # Frecuencia de bucle
        self.rate = rospy.Rate(30)  # 30Hz
        
        # Variable para almacenar la posición estimada del robot
        self.current_pose = None

        # Configuración de Matplotlib para el modo interactivo
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.estimated_line, = self.ax.plot([], [], label='Estimated Trajectory (EKF)', linestyle='-')
        self.desired_line, = self.ax.plot([], [], 'ro', label='Desired Points (Sinusoidal)')
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_title('Real-Time Trajectory Visualization')
        self.ax.legend()
        self.ax.grid(True)

    def pose_callback(self, msg):
        # Actualizar la posición estimada del robot
        self.current_pose = msg.pose.pose.position
        self.estimated_positions.append((self.current_pose.x, self.current_pose.y))
        
    def generate_sinusoidal_trajectory(self):
        num_cycles = 1  # Número de ciclos de la sinusoide
        sinusoidal_trajectory_points = []
        total_length = num_cycles * self.wavelength  # Longitud total de la trayectoria
        
        for i in range(self.num_points):
            x = i * total_length / (self.num_points - 1)
            y = self.amplitude * math.sin(2 * math.pi * x / self.wavelength)
            sinusoidal_trajectory_points.append((x, y))
        
        return sinusoidal_trajectory_points


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
