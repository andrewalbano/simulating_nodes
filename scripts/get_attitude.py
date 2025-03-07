#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from pymavlink import mavutil


if __name__ == '__main__':
    rospy.init_node('get_attitude_test')
    rate =rospy.Rate(10)

    pub_position_point = rospy.Publisher('sitl_attitude',Point, queue_size=10)
    pub_velocity_point = rospy.Publisher('sitl_attitude_omega',Point, queue_size=10)
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14553')
    master.wait_heartbeat()    
    point = Point()     #in ned frame
    velocity_point = Point()

    while not rospy.is_shutdown():
        # gettting position from simulation
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,  # Command
            0,  # Confirmation
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,  # Request specific message ID (BATTERY_STATUS)
            0, 0, 0, 0, 0, 0  # Unused parameters
        )  
        msg = master.recv_match(type='ATTITUDE', blocking=True)
        if not msg:
            continue

        
        point.x = msg.roll
        point.y = msg.pitch
        point.z = msg.yaw
        
        velocity_point.x = msg.rollspeed
        velocity_point.y = msg.pitchspeed
        velocity_point.z = msg.yawspeed

        pub_position_point.publish(point)
        pub_velocity_point.publish(velocity_point)
        
        # rospy.loginfo_throttle(5, f"roll,pitch,yaw: {point.x}, {point.y}, {point.z}")
        # rospy.sleep(0.1)
        rate.sleep()