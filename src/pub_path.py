#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

class PathPublisher:
    def __init__(self):
        rospy.init_node('path_publisher')
        self.path_pub = rospy.Publisher('/desired_path', Path, queue_size=10)
        self.path_offset = 5.0  # Offset to move the path away from the origin
        self.generate_s_trajectory()

    def generate_s_trajectory(self):
        path_msg = Path()
        path_msg.header.frame_id = 'odom_ekf'
        num_points = 20

        for i in range(num_points):
            x = i * 6.0 * 3 / num_points + self.path_offset
            y = 4.0 * math.sin((2 * math.pi / 6.0) * x)
            
            pose = PoseStamped()
            pose.header.frame_id = 'odom_ekf'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0
            pose.pose.orientation.w = 1.0  # Identity orientation
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        rospy.loginfo("Path published.")

if __name__ == '__main__':
    try:
        PathPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
