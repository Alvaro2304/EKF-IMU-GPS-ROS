#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Path
import math
import tf

class PurePursuitController:
    def __init__(self):
        rospy.init_node('pure_pursuit_controller')

        rospy.Subscriber('/ekf_pose', PoseWithCovarianceStamped, self.pose_callback)
        rospy.Subscriber('/desired_path', Path, self.path_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.max_linear_speed = 0.3  # m/s
        self.max_angular_speed = 0.8  # rad/s

        self.lookahead_distance = 1.0  # meters

        self.rate = rospy.Rate(30)  # 30Hz

        self.current_pose = None
        self.current_orientation = None
        self.path_points = []

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        self.current_orientation = self.quaternion_to_euler(orientation_q)

    def path_callback(self, msg):
        self.path_points = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]

    def quaternion_to_euler(self, q):
        orientation_list = [q.x, q.y, q.z, q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        return yaw

    def run(self):
        rospy.loginfo("Pure Pursuit Controller started.")
        while not rospy.is_shutdown():
            if self.current_pose and self.current_orientation is not None and self.path_points:
                current_x = self.current_pose.x
                current_y = self.current_pose.y
                current_theta = self.current_orientation

                closest_point, lookahead_point = self.find_closest_and_lookahead_points(current_x, current_y)

                if closest_point is not None and lookahead_point is not None:
                    target_x, target_y = lookahead_point

                    # Transform the lookahead point to the robot's local frame
                    dx = target_x - current_x
                    dy = target_y - current_y
                    target_x_robot = dx * math.cos(current_theta) + dy * math.sin(current_theta)
                    target_y_robot = -dx * math.sin(current_theta) + dy * math.cos(current_theta)

                    # Control law
                    alpha = math.atan2(target_y_robot, target_x_robot)
                    linear_velocity = self.max_linear_speed
                    angular_velocity = (2 * linear_velocity * math.sin(alpha)) / self.lookahead_distance

                    # Limitar las velocidades dentro de los máximos permitidos
                    angular_velocity = max(min(angular_velocity, self.max_angular_speed), -self.max_angular_speed)

                    # Crear mensaje Twist con las velocidades calculadas
                    twist_cmd = Twist()
                    twist_cmd.linear.x = linear_velocity
                    twist_cmd.angular.z = angular_velocity

                    # Publicar comando de velocidad
                    self.cmd_vel_pub.publish(twist_cmd)

            self.rate.sleep()

    def find_closest_and_lookahead_points(self, current_x, current_y):
        closest_point = None
        lookahead_point = None
        min_distance = float('inf')

        for point in self.path_points:
            distance = math.sqrt((current_x - point[0]) ** 2 + (current_y - point[1]) ** 2)
            if distance < min_distance:
                min_distance = distance
                closest_point = point

        if closest_point:
            for point in self.path_points:
                distance = math.sqrt((current_x - point[0]) ** 2 + (current_y - point[1]) ** 2)
                if distance > self.lookahead_distance:
                    lookahead_point = point
                    break

        return closest_point, lookahead_point

if __name__ == '__main__':
    try:
        controller = PurePursuitController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
