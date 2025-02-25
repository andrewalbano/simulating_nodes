#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from pymavlink import mavutil


if __name__ == '__main__':
    rospy.init_node('get_ned_test')
    pub_pos_point = rospy.Publisher('sitl_xyz',Point, queue_size=10)
    pub_vel_point = rospy.Publisher('sitl_velocity_xyz',Point, queue_size=10)

    master = mavutil.mavlink_connection('udpin:0.0.0.0:14552')
    # master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    master.wait_heartbeat()    
    pos_point = Point()     #in ned frame
    velocity_point = Point()


    while not rospy.is_shutdown():
        # gettting position from simulation
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,  # Command
            0,  # Confirmation
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,  # Request specific message ID (BATTERY_STATUS)
            0, 0, 0, 0, 0, 0  # Unused parameters
        )  
        msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        if not msg:
            continue

        
        pos_point.x= msg.x
        pos_point.y = msg.y
        pos_point.z= msg.z

        velocity_point.x = msg.vx
        velocity_point.y = msg.vy
        velocity_point.z = msg.vz
        # rospy.loginfo(f"raw vx = {msg.vx}")

        pub_pos_point.publish(pos_point)

        pub_vel_point.publish(velocity_point)
        
        # rospy.loginfo_throttle(5, f"x,y,z: {point.x}, {point.y}, {point.z}")
        rospy.sleep(0.1)