#!/usr/bin/env python3
import rospy
import numpy as np
# import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped,PoseArray, TwistStamped, Pose, Twist
from tf.transformations import euler_from_quaternion
from mavros_msgs.msg import OverrideRCIn
from pymavlink import mavutil
from std_msgs.msg import Bool, Int8, Float32MultiArray



class velocity:
    def __init__(self):
        self.velocity_setpoint = Twist()
        self.velocity = Twist()
        self.frequency = 10
        self.rate =rospy.Rate(self.frequency)

        # publishingg controller setpoints
        self.pub1 = rospy.Publisher('velocity_setpoint', Twist, queue_size=10)
        self.pub2 = rospy.Publisher('/dvl/twist', Twist, queue_size=10)
        # self.pub2 = rospy.Publisher('pwm_setpoint', Twist, queue_size=10)
        
    def publish_velocity_setpoint(self):

        self.velocity_setpoint.linear.x = 10
        self.velocity_setpoint.linear.y = 0
        self.velocity_setpoint.linear.z = 0
        self.velocity_setpoint.angular.x= 0
        self.velocity_setpoint.angular.y = 0
        self.velocity_setpoint.angular.z = 0


        self.pub1.publish(self.velocity_setpoint)
    

    def publish_velocity(self):

        self.velocity.linear.x = 5
        self.velocity.linear.y = 0
        self.velocity.linear.z = 0
        self.velocity.angular.x= 0
        self.velocity.angular.y = 0
        self.velocity.angular.z = 0


        self.pub2.publish(self.velocity)



def main(): 
    # Initialize the ROS node
    
    rospy.init_node('simulate_velocity_data')
    rospy.loginfo("initialized velocity_data node")
    data = velocity()
    

    while not rospy.is_shutdown():


        data.publish_velocity_setpoint()
        data.publish_velocity()

        data.rate.sleep()    

    
if __name__ == "__main__":

    main()