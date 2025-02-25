#!/usr/bin/env python3

import rospy
# import tf
from tf.transformations import quaternion_from_euler, translation_matrix, euler_matrix, concatenate_matrices, translation_from_matrix, euler_from_matrix, quaternion_matrix
# from math import fmod
# import time
import numpy as np

# from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped, Point, PoseStamped, Twist
# from nav_msgs.msg import Path



# class kinematic_simulator:
#     def __init__(self):
#         rospy.init_node('simulator_node')
#         self.br = tf.TransformBroadcaster()    

#         # Linear velocities in meters per second
#         # self.velocity = TwistStamped()
#         self.velocity = Twist()
#         # self.velocity_x = 0.1
#         # self.velocity_y = 0.1
#         # self.velocity_z = 0.0

#         # Angular velocity in radians per second
#         # self.angular_velocity_yaw = math.radians(10)  # 10 degrees per second
#         self.frequency = 10
#         self.rate = rospy.Rate(self.frequency)  # 10 Hz

#         # Initial position and yaw
#         self.pos_x = 0.0
#         self.pos_y = 0.0
#         self.pos_z = 0.0  
#         self.yaw = 0.0
#         self.roll =0.0
#         self.pitch=0.0

#         # Initial velocity 
#         self.vx =0
#         self.vy = 0
#         self.vz = 0
#         self.vyaw = 0


#         self.path = Path()
#         self.path.header.frame_id='NED'

#         self.start_time = time.time()

#         # subscribe to twist 
#         self.sub1 = rospy.Subscriber('sitl_xyz', Point, self.position_callback)

#         self.sub2 = rospy.Subscriber('sitl_attitude', Point, self.attitude_callback)

#         self.sub3 = rospy.Subscriber('sitl_velocity_xyz', Point, self.lin_velocity_callback)

#         self.sub4 = rospy.Subscriber('sitl_attitude_omega', Point, self.angular_velocity_callback)


#         self.pub1 = rospy.Publisher('/state',PoseWithCovarianceStamped, queue_size=10)
#         self.pub2 = rospy.Publisher('actual_path',Path, queue_size=10)
#         self.pub3 = rospy.Publisher('/dvl/twist',Twist, queue_size=10)

#         # self.pub3 = rospy.Publisher('sitl_current_velocity',Twist, queue_size=10)

#     def publish_pose(self):
#         msg = PoseWithCovarianceStamped()

#         pose_stamped_msg = PoseStamped()
#         pose_stamped_msg.header.frame_id = 'NED'
#         self.path.header.frame_id = 'NED'
        
#         msg.header.frame_id = 'NED'
#         msg.pose.pose.position.x = self.pos_x
#         msg.pose.pose.position.y = self.pos_y
#         msg.pose.pose.position.z = self.pos_z

#         q = quaternion_from_euler(self.roll,self.pitch,self.yaw)

#         msg.pose.pose.orientation.x = q[0]
#         msg.pose.pose.orientation.y = q[1]
#         msg.pose.pose.orientation.z = q[2]
#         msg.pose.pose.orientation.w = q[3]

#         msg.pose.covariance = [
#             0.1, 0.01, 0.02, 0.00, 0.00, 0.00,   # Row 1: variance of x and covariances with y, z, roll, pitch, yaw
#             0.01, 0.1, 0.01, 0.00, 0.00, 0.00,   # Row 2: covariance with x, variance of y, covariances with z, roll, pitch, yaw
#             0.02, 0.01, 0.1, 0.00, 0.00, 0.00,   # Row 3: covariance with x, y, variance of z, covariances with roll, pitch, yaw
#             0.00, 0.00, 0.00, 0.1, 0.01, 0.02,   # Row 4: covariance with x, y, z, variance of roll, covariances with pitch, yaw
#             0.00, 0.00, 0.00, 0.01, 0.1, 0.01,   # Row 5: covariance with x, y, z, roll, variance of pitch, covariance with yaw
#             0.00, 0.00, 0.00, 0.02, 0.01, 0.1    # Row 6: covariance with x, y, z, roll, pitch, variance of yaw
#         ]
#         self.pub1.publish(msg)

#         pose_stamped_msg.pose = msg.pose.pose
#         self.path.poses.append(pose_stamped_msg)
#         self.pub2.publish(self.path)


#     def position_callback(self,msg:Point):
#         self.pos_x = msg.x
#         self.pos_y = msg.y
#         self.pos_z = msg.z

#     def attitude_callback(self,msg:Point):
#         self.yaw = msg.z
          

#     def lin_velocity_callback(self,msg:Point):
#         self.vx = msg.x
#         self.vy = msg.y
#         self.vz = msg.z
        
#     def angular_velocity_callback(self,msg:Point):
#         self.vyaw = msg.z
    


#     def update_position(self):
#         # Calculate elapsed time
#         # elapsed = time.time() - self.start_time

#         # extracting the current pose as a rotation matrix and converting to 3 by 3
#         current_rotation = euler_matrix(self.roll, self.pitch, self.yaw)
#         # current_rotation = current_rotation[:3,:3]

#         #current position
#         # p_current= np.array([[self.pos_x ],[self.pos_y],[self.pos_z]])

#         # linear displacement in the body frame
#         x_rel = self.velocity.twist.linear.x * (1.0 / self.frequency)
#         y_rel = self.velocity.twist.linear.y * (1.0 / self.frequency)
#         z_rel = self.velocity.twist.linear.z * (1.0 / self.frequency)
#         p_rel = np.array([[x_rel],[y_rel],[z_rel]])

#         # converting the displacement due to linear velocity to be relative to the inertial frame (NED)
#         # delta_p = current_rotation @ p_rel

#         self.pos_x += current_rotation[0,0:3]@ p_rel
#         self.pos_y += current_rotation[1,0:3]@ p_rel
#         self.pos_z += current_rotation[2,0:3]@ p_rel


#         self.yaw += self.velocity.twist.angular.z * (1.0 / self.frequency)
    

#     def publish_velocity(self):
#         # self.velocity.header.frame_id="body_link"
#         # self.velocity.twist.linear.x = self.vx
#         # self.velocity.twist.linear.y = self.vy
#         # self.velocity.twist.linear.z = self.vz
#         # self.velocity.twist.angular.x = 0
#         # self.velocity.twist.angular.y = 0
#         # self.velocity.twist.angular.z = self.vyaw

 
#         self.velocity.linear.x = self.vx
#         self.velocity.linear.y = self.vy
#         self.velocity.linear.z = self.vz
#         self.velocity.angular.x = 0
#         self.velocity.angular.y = 0
#         self.velocity.angular.z = self.vyaw
#         self.pub3.publish(self.velocity)

#     def ned2body(self):
#         R = euler_matrix(self.roll,self.pitch, self.yaw)
#         v = [self.vx, self.vy, self.vz]
#         velocity = 


#     def broadcast_transform(self):
#         # Send the updated transform
#         self.br.sendTransform(
#             (self.pos_x, self.pos_y, self.pos_z),
#             quaternion_from_euler(self.roll, self.pitch, self.yaw),
#             rospy.Time.now(),
#             'base_link',   # child frame
#             'NED'        # parent frame, typically 'world' or 'map'
#         )

#     def run(self):
#         # self.update_position()
#         self.broadcast_transform()
        


#         self.publish_pose()
#         self.publish_velocity()

        
#         self.rate.sleep()

# if __name__ == '__main__':


#     try:
#         sim = kinematic_simulator()
#         while not rospy.is_shutdown():
#             sim.run()
#     except rospy.ROSInterruptException:
#         pass


roll = 0
pitch = 0
yaw = np.pi/2

vx = 1
vy = 2
vz = 3


def ned2body(roll,pitch,yaw,vx,vy,vz):
        R = euler_matrix(roll,pitch,yaw)
        
        v = np.array(([vx],[vy],[vz],[1]))
        print(R.shape)
        print(v.shape)
        velocity = np.linalg.inv(R)@v
        print(velocity)
        print(velocity[0][0])
        vx = velocity[2,0]

        print(vx)

ned2body(roll,pitch,yaw,vx,vy,vz)
