#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from tf.transformations import *

# from tf.transformations import euler_from_quaternion
x,y,z = 0.0, 0.0, 0.0
roll, pitch ,yaw = 0,0,0

#  initialize a direction 
direction = 1
index  = 0
q = quaternion_from_euler(roll,pitch,yaw,'sxyz')


if __name__ =='__main__':
    # initialize node
    rospy.init_node("current_state_test")
    rospy.loginfo("current_state_test node has been started.")

    pub = rospy.Publisher("current_state_test",PoseWithCovarianceStamped, queue_size=10)

    rate = rospy.Rate(2)

    # while index < 2:

    while not rospy.is_shutdown():
        # switch between moving forward and backwards
        if direction == 1:
            x += 1
        else:
            x -= 1
        

        # when x hits 5 switch directions, when x hits 0 switch directions again
        if x == 5.0 :
            direction = 0
            index +=0.5
        
        elif x == 0.0:
            direction = 1
            index += 0.5

        # print(x)
        # print("index: ", index)
        

        # publish message
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x/10
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        msg.pose.covariance = [
            0.1, 0.01, 0.02, 0.00, 0.00, 0.00,   # Row 1: variance of x and covariances with y, z, roll, pitch, yaw
            0.01, 0.1, 0.01, 0.00, 0.00, 0.00,   # Row 2: covariance with x, variance of y, covariances with z, roll, pitch, yaw
            0.02, 0.01, 0.1, 0.00, 0.00, 0.00,   # Row 3: covariance with x, y, variance of z, covariances with roll, pitch, yaw
            0.00, 0.00, 0.00, 0.1, 0.01, 0.02,   # Row 4: covariance with x, y, z, variance of roll, covariances with pitch, yaw
            0.00, 0.00, 0.00, 0.01, 0.1, 0.01,   # Row 5: covariance with x, y, z, roll, variance of pitch, covariance with yaw
            0.00, 0.00, 0.00, 0.02, 0.01, 0.1    # Row 6: covariance with x, y, z, roll, pitch, variance of yaw
        ]

        pub.publish(msg)
        rate.sleep()