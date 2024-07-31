#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

class VelocityPlotter:
    def __init__(self):
        rospy.init_node('velocity_plotter')
        
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        # Colas para almacenar los datos de velocidad
        self.linear_velocities = deque(maxlen=1000)
        self.angular_velocities = deque(maxlen=1000)
        self.times = deque(maxlen=1000)
        
        self.start_time = rospy.get_time()
        
        # Configuración de las gráficas
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1)
        self.line1, = self.ax1.plot([], [], 'b-')
        self.line2, = self.ax2.plot([], [], 'r-')
        
        self.ax1.set_xlim(0, 10)
        self.ax1.set_ylim(-0.5, 0.5)
        self.ax1.set_title("Linear Velocity")
        self.ax1.set_xlabel("Time (s)")
        self.ax1.set_ylabel("Linear Velocity (m/s)")
        
        self.ax2.set_xlim(0, 10)
        self.ax2.set_ylim(-1.0, 1.0)
        self.ax2.set_title("Angular Velocity")
        self.ax2.set_xlabel("Time (s)")
        self.ax2.set_ylabel("Angular Velocity (rad/s)")
        
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=100)
        
    def cmd_vel_callback(self, msg):
        current_time = rospy.get_time() - self.start_time
        self.times.append(current_time)
        self.linear_velocities.append(msg.linear.x)
        self.angular_velocities.append(msg.angular.z)
        
    def update_plot(self, frame):
        if len(self.times) == 0:
            return self.line1, self.line2
        
        # Actualizar los datos de las gráficas
        self.line1.set_data(self.times, self.linear_velocities)
        self.line2.set_data(self.times, self.angular_velocities)
        
        # Ajustar los límites de las gráficas si es necesario
        self.ax1.set_xlim(max(0, self.times[0]), max(10, self.times[-1]))
        self.ax2.set_xlim(max(0, self.times[0]), max(10, self.times[-1]))
        
        return self.line1, self.line2
    
    def run(self):
        rospy.loginfo("Velocity Plotter started.")
        plt.show()

if __name__ == '__main__':
    try:
        plotter = VelocityPlotter()
        plotter.run()
    except rospy.ROSInterruptException:
        pass
