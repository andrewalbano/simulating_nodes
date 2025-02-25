#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistStamped

def twist_stamped_publisher():
    # Initialize the node
    rospy.init_node('sim_velocity_output')
    
    # Create a publisher object
    pub = rospy.Publisher('velocity_command', TwistStamped, queue_size=10)
    
    # Set the publishing rate
    rate = rospy.Rate(10)  # 10 Hz

    # Create a TwistStamped message
    twist_stamped_msg = TwistStamped()
    twist_stamped_msg.header.frame_id = "base_link"  # Example frame id

    while not rospy.is_shutdown():
        # Populate the TwistStamped message here
        twist_stamped_msg.twist.linear.x = 0.5  # Example velocity
        twist_stamped_msg.twist.linear.y = 0.0  # Example velocity
        twist_stamped_msg.twist.linear.z = 0.0  # Example velocity
        twist_stamped_msg.twist.angular.z = 0.1  # Example angular rate
        twist_stamped_msg.twist.angular.y = 0.0  # Example angular rate
        twist_stamped_msg.twist.angular.x = 0.0  # Example angular rate

        # Publish the message
        pub.publish(twist_stamped_msg)

        # Sleep for the remaining time to enforce the rate
        rate.sleep()

if __name__ == '__main__':
    try:
        twist_stamped_publisher()
    except rospy.ROSInterruptException:
        pass
