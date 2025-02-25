#!/usr/bin/env python3

import rospy
import tf
from tf.transformations import quaternion_from_euler, translation_matrix, euler_matrix, concatenate_matrices, translation_from_matrix, euler_from_matrix
from math import fmod
import time
import numpy as np

from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped



class kinematic_simulator:
    def __init__(self):
        rospy.init_node('simulator_node')
        self.br = tf.TransformBroadcaster()    

        # Linear velocities in meters per second
        self.velocity = TwistStamped()
        # self.velocity_x = 0.1
        # self.velocity_y = 0.1
        # self.velocity_z = 0.0

        # Angular velocity in radians per second
        # self.angular_velocity_yaw = math.radians(10)  # 10 degrees per second
        self.frequency = 10
        self.rate = rospy.Rate(self.frequency)  # 10 Hz

        # Initial position and yaw
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0  
        self.yaw = 0.0
        self.roll =0.0
        self.pitch=0.0

        self.start_time = time.time()

        # subscribe to twist 
        self.sub1 = rospy.Subscriber('velocity_command', TwistStamped, self.velocity_callback)

        self.pub1 = rospy.Publisher('/dvl/local_position',PoseWithCovarianceStamped, queue_size=10)


    def publish_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'NED'
        msg.pose.pose.position.x = self.pos_x
        msg.pose.pose.position.y = self.pos_y
        msg.pose.pose.position.z = self.pos_z

        q = quaternion_from_euler(self.roll,self.pitch,self.yaw)

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
        self.pub1.publish(msg)

    def velocity_callback(self,msg:TwistStamped):
        self.velocity = msg

    
    def update_position(self):
        # Calculate elapsed time
        # elapsed = time.time() - self.start_time

        # extracting the current pose as a rotation matrix and converting to 3 by 3
        current_rotation = euler_matrix(self.roll, self.pitch, self.yaw)
        # current_rotation = current_rotation[:3,:3]

        #current position
        # p_current= np.array([[self.pos_x ],[self.pos_y],[self.pos_z]])

        # linear displacement in the body frame
        x_rel = self.velocity.twist.linear.x * (1.0 / self.frequency)
        y_rel = self.velocity.twist.linear.y * (1.0 / self.frequency)
        z_rel = self.velocity.twist.linear.z * (1.0 / self.frequency)
        p_rel = np.array([[x_rel],[y_rel],[z_rel]])

        # converting the displacement due to linear velocity to be relative to the inertial frame (NED)
        # delta_p = current_rotation @ p_rel

        self.pos_x += current_rotation[0,0:3]@ p_rel
        self.pos_y += current_rotation[1,0:3]@ p_rel
        self.pos_z += current_rotation[2,0:3]@ p_rel


        self.yaw += self.velocity.twist.angular.z * (1.0 / self.frequency)
        '''
        # # skew symmetric matrix of angular velocity (4 by 4)
        # angular_velocity_skew = np.array([[0, -1* self.velocity.twist.angular.z, self.velocity.twist.angular.y],
        #                                   [self.velocity.twist.angular.z, 0, -1*self.velocity.twist.angular.x],
        #                                   [-1*self.velocity.twist.angular.y, self.velocity.twist.angular.x,0],
        #                                   [0,0,0,1]
        #                                   ])
        
        # # compute the rotation matrix for the time interval
        # rotation_matrix = (angular_velocity_skew * (1.0 / self.frequency))@current_rotation
        
        # # get euler angles
        # self.roll = 0
        # self.pitch = 0
        # _,_,self.yaw  = euler_from_matrix(rotation_matrix) 
        




        
        # new attitude is given by the following     
        # new_rotation = delta_rotation@current_rotation 
        
        # p_new = p_current + delta_p




        # T_new = np.eye(4)
        # T_new[:3,:3] = new_rotation
        # T_new[0,3] = p_new[0]
        # T_new[1,3] = p_new[1]
        # T_new[2,3] = p_new[2]

        # self.pos_x = float(p_new[0])
        # self.pos_y = float(p_new[1])
        # self.pos_z = float(p_new[2])

        # euler = euler_from_matrix(T_new)

        # self.roll, self.pitch, self.yaw = float(euler[0]),float(euler[1]),float(euler[2])
        



        # self.velocity.twist.linear.x * (1.0 / self.frequency)
        # # self.pos_y += self.velocity.twist.linear.y * (1.0 / self.frequency)
        # # self.pos_z 
        


        # # Update yaw based on angular velocity
        # self.yaw += self.velocity.twist.angular.z * (1.0 / self.frequency)

        # # get relative transform in base_frame and then convert it to ned _FrAME
        # # Update position based on linear velocity in base_link  frame
        # x_rel = self.velocity.twist.linear.x * (1.0 / self.frequency)
        # y_rel = self.velocity.twist.linear.y * (1.0 / self.frequency)
        # z_rel = self.velocity.twist.linear.z * (1.0 / self.frequency)

        # # Update yaw based on angular velocity in base_link frame
        # roll_rel = 0 
        # pitch_rel = 0
        # yaw_rel= self.velocity.twist.angular.z * (1.0 / self.frequency)
        
        # T = translation_matrix([x_rel,y_rel,z_rel])
        # body_transform = T
        # # R = euler_matrix(roll_rel,pitch_rel,yaw_rel)
        # # body_transform = concatenate_matrices(R,T)


        # # generate transform current position transform in ned frame
        # T = translation_matrix([self.pos_x,self.pos_y,self.pos_z])
        # R = euler_matrix(self.roll, self.pitch, self.yaw)
        # current_transform = concatenate_matrices(R,T)

        # # updated transform of new position
        # updated_transform = concatenate_matrices(current_transform, body_transform)
        # # updated_transform = concatenate_matrices(body_transform, current_transform, )
        
        # # getting translations and rotation
        # translation = translation_from_matrix(updated_transform)
        # euler = euler_from_matrix(updated_transform)

        # updating pose

        # self.pos_x += x_rel
        
        # angular_velocity_skew = np.array([[0, -1* self.velocity.twist.angular.z, self.velocity.twist.angular.y],
        #                                   [self.velocity.twist.angular.z, 0, -1*self.velocity.twist.angular.x],
        #                                   [-1*self.velocity.twist.angular.y, self.velocity.twist.angular.x,0],
        #                                   ])


        # =,self.pos_y,self.pos_z = translation[0], translation[1], translation[2]
        
        # # self.roll, self.pitch, self.yaw = euler[0],euler[1],euler[2]

        # # Update yaw based on angular velocity
        # self.yaw += self.velocity.twist.angular.z * (1.0 / self.frequency)

        # causing issue when frame is rotated
        # # Update position based on linear velocity
        # self.pos_x += self.velocity.twist.linear.x * (1.0 / self.frequency)
        # self.pos_y += self.velocity.twist.linear.y * (1.0 / self.frequency)
        # self.pos_z += self.velocity.twist.linear.z * (1.0 / self.frequency)
        # # Update yaw based on angular velocity
        # self.yaw += self.velocity.twist.angular.z * (1.0 / self.frequency)
        '''

    def broadcast_transform(self):
        # Send the updated transform
        self.br.sendTransform(
            (self.pos_x, self.pos_y, self.pos_z),
            quaternion_from_euler(self.roll, self.pitch, self.yaw),
            rospy.Time.now(),
            'base_link',   # child frame
            'NED'        # parent frame, typically 'world' or 'map'
        )

    def run(self):
        self.update_position()
        self.broadcast_transform()
        self.publish_pose()
        self.rate.sleep()

if __name__ == '__main__':
    try:
        sim = kinematic_simulator()
        while not rospy.is_shutdown():
            sim.run()
    except rospy.ROSInterruptException:
        pass
