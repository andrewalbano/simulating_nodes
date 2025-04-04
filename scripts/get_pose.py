#!/usr/bin/env python3

import rospy
import tf
from tf.transformations import quaternion_from_euler, translation_matrix, euler_matrix, concatenate_matrices, translation_from_matrix, euler_from_matrix, quaternion_matrix
from math import fmod
import time
import numpy as np

from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped, Point, PoseStamped, Twist
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float32

import time

# start_time = time()
# ...

# end_time = time()
# seconds_elapsed = end_time - start_time

class kinematic_simulator:
    def __init__(self):

        n = 1 #number of data points to consider
        self.alpha = 2 / (n + 1)
        self.alpha =0.9


        queue_size = 1
        rospy.init_node('simulator_node')
        self.br = tf.TransformBroadcaster()    

        # Linear velocities in meters per second
        # self.velocity = TwistStamped()
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

        self.last_x = 0.0
        self.last_y= 0.0
        self.last_z = 0.0
        self.last_yaw = 0.0

        # Initial velocity 
        self.vx =0
        self.vy = 0
        self.vz = 0
        self.vyaw = 0

        self.vx_data = []
        


        self.path = Path()
        self.path.header.frame_id='NED'

        self.start_time = time.time()


        self.time_step = 1/self.frequency

        self.odom = Odometry() 


        # subscribe to twist 
        self.sub1 = rospy.Subscriber('sitl_xyz', Point, self.position_callback)

        self.sub2 = rospy.Subscriber('sitl_attitude', Point, self.attitude_callback)
        

        self.sub3 = rospy.Subscriber('sitl_velocity_xyz', Point, self.lin_velocity_callback)

        self.sub4 = rospy.Subscriber('sitl_attitude_omega', Point, self.angular_velocity_callback)


        self.pub1 = rospy.Publisher('/state',PoseWithCovarianceStamped, queue_size=queue_size )
        self.pub2 = rospy.Publisher('actual_path',Path, queue_size= queue_size)
        self.pub3 = rospy.Publisher('/dvl/twist',TwistStamped, queue_size=queue_size)
        self.pub4 = rospy.Publisher('/odom',Odometry, queue_size=queue_size)
        self.pub5 = rospy.Publisher('/plot_vx',Float32, queue_size=queue_size)
        self.pub6 = rospy.Publisher('/plot_vy',Float32, queue_size=queue_size)
        self.pub7 = rospy.Publisher('/plot_vz',Float32, queue_size=queue_size)
        self.pub8 = rospy.Publisher('/plot_vyaw',Float32, queue_size=queue_size)


        

    def publish_pose(self):
        msg = PoseWithCovarianceStamped()

        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.frame_id = 'NED'
        self.path.header.frame_id = 'NED'
        
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

        pose_stamped_msg.pose = msg.pose.pose
        self.path.poses.append(pose_stamped_msg)
        self.pub2.publish(self.path)


    def position_callback(self,msg:Point):
        self.last_x = self.pos_x
        self.last_y = self.pos_y
        self.last_z = self.pos_z
    

        self.pos_x = msg.x
        self.pos_y = msg.y
        self.pos_z = msg.z

        # self.time_step = self.start_time-time.time()
        # self.start_time = time.time()

    def attitude_callback(self,msg:Point):

        self.last_yaw = self.yaw


        self.roll = msg.x
        self.pitch = msg.y
        self.yaw = msg.z


    def lin_velocity_callback(self,msg:Point):

        self.vx = msg.x
        self.vy = msg.y
        self.vz = msg.z

        # exponential filtering
       
        # self.vx = self.alpha * msg.x + (1-self.alpha)*self.vx
        # self.vy = self.alpha * msg.y + (1-self.alpha)*self.vy
        # self.vz = self.alpha * msg.z + (1-self.alpha)*self.vz
        
    def angular_velocity_callback(self,msg:Point):
        self.vyaw = msg.z

        # exponential filtering
     
        # self.vyaw = self.alpha * msg.z + (1-self.alpha)*self.vyaw
    


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
    

    def publish_velocity(self):
        # self.velocity.header.frame_id="body_link"
        # self.velocity.twist.linear.x = self.vx
        # self.velocity.twist.linear.y = self.vy
        # self.velocity.twist.linear.z = self.vz
        # self.velocity.twist.angular.x = 0
        # self.velocity.twist.angular.y = 0
        # self.velocity.twist.angular.z = self.vyaw

        self.velocity.header.frame_id = "base_link"
        self.velocity.twist.linear.x = self.vx
        self.velocity.twist.linear.y = self.vy
        self.velocity.twist.linear.z = self.vz
        self.velocity.twist.angular.x = 0
        self.velocity.twist.angular.y = 0
        self.velocity.twist.angular.z = self.vyaw
        self.pub3.publish(self.velocity)

        self.pub5.publish(self.vx)
        self.pub6.publish(self.vy)
        self.pub7.publish(self.vz)
        self.pub8.publish(self.vyaw*180/np.pi)

        

    def ned2body(self):
        R = np.linalg.inv(euler_matrix(self.roll,self.pitch, self.yaw))
        # R = np.linalg.inv(euler_matrix(0,0, self.yaw))
        v = np.matrix([[self.vx],[self.vy],[self.vz],[1]])

        # rospy.loginfo(f"NED VELOCITY: {self.vx},{self.vy}")
        velocity = R@v
        
        self.vx = velocity[0,0]
        self.vy = velocity[1,0]
        self.vz = velocity[2,0]

    # NO LONGER USED
    def calc_ned_vel(self):
        
        self.vx = (self.pos_x - self.last_x)/self.time_step
        self.vy = (self.pos_y - self.last_y)/self.time_step
        self.vz = (self.pos_z - self.last_z)/self.time_step
        self.vyaw = (self.yaw- self.last_yaw)/self.time_step


    def broadcast_transform(self):
        # Send the updated transform
        self.br.sendTransform(
            (self.pos_x, self.pos_y, self.pos_z),
            quaternion_from_euler(self.roll, self.pitch, self.yaw),
            rospy.Time.now(),
            'base_link',   # child frame
            'NED'        # parent frame, typically 'world' or 'map'
        )

    # NO LONGER USED
    def publish_odom(self):
        self.odom.header.stamp = time.time()
        self.odom.header.frame_id = "NED"
        self.odom.pose.pose.position.x = self.pos_x
        self.odom.pose.pose.position.y = self.pos_y
        self.odom.pose.pose.position.z = self.pos_z
        q =  quaternion_from_euler(self.roll,self.pitch,self.yaw)

        
        self.odom.pose.pose.orientation.x = q[0]
        self.odom.pose.pose.orientation.y = q[1]
        self.odom.pose.pose.orientation.z = q[2]
        self.odom.pose.pose.orientation.w = q[3]
        
        self.odom.child_frame_id = "base_link"

        self.odom.twist.twist.linear.x = self.vx
        self.odom.twist.twist.linear.y = self.vy
        self.odom.twist.twist.linear.z = self.vz
        self.odom.twist.twist.angular.x = 0
        self.odom.twist.twist.angular.y = 0
        self.odom.twist.twist.angular.z = self.vyaw
        
        self.pub4.publish(self.odom)
        





    def run(self):
        # self.update_position()
        self.broadcast_transform()
        
        # self.calc_ned_vel()
        # note: cuirrently using velocity published in ned format from q ground control 

        

        # convert the linear velocity in ned to body frame 
        # the angular velocity is already in body frame 
        self.ned2body() 
       

        self.publish_pose()
        
        self.publish_velocity()

        # self.publish_odom()
        

        
        self.rate.sleep()

if __name__ == '__main__':


    try:
        sim = kinematic_simulator()
        while not rospy.is_shutdown():
            sim.run()
    except rospy.ROSInterruptException:
        pass
