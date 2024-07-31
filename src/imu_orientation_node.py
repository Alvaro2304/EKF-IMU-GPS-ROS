#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from sensor_msgs.msg import Imu

def imu_callback(data):
    # Extracción de la orientación en forma de quaternion
    orientation_q = data.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    
    # Convertir quaternion a euler (roll, pitch, yaw)
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
    
    # Imprimir los valores de roll, pitch, y yaw
    rospy.loginfo("Roll: %f, Pitch: %f, Yaw: %f" % (roll, pitch, yaw))

def imu_orientation_node():
    rospy.init_node('imu_orientation_node', anonymous=True)
    
    rospy.Subscriber('/imu', Imu, imu_callback)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        imu_orientation_node()
    except rospy.ROSInterruptException:
        pass
