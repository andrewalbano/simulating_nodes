#!/usr/bin/env python3

import rospy
import tf
from tf.transformations import quaternion_from_euler, translation_matrix, euler_matrix, concatenate_matrices, translation_from_matrix, euler_from_matrix
from math import fmod
import time
import numpy as np

from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped, Twist, PoseStamped
from nav_msgs.msg import Path



class kinematic_simulator:
    def __init__(self):
        
        self.br = tf.TransformBroadcaster()  


        self.apply_acc_limits = True

        self.max_linear_acceleration = 0.05
        self.max_linear_deceleration = 0.1

        # deg/s
        self.max_angular_acceleration = 1*np.pi/180
        self.max_angular_deceleration = 3*np.pi/180


        # self.path.poses
        # self.velocity_setpoint = Twist
        

        # Linear velocities in meters per second
        # self.velocity = TwistStamped()
        self.velocity = Twist()
        self.velocity.linear.x =0
        self.velocity.linear.y =0
        self.velocity.linear.z =0

        self.velocity.angular.x=0
        self.velocity.angular.y=0
        self.velocity.angular.z=0

 
        # Initial position and yaw
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0  
        self.yaw = 0.0
        self.roll = 0.0
        self.pitch = 0.0

        self.current_pose = PoseWithCovarianceStamped()
        self.current_pose.header.frame_id = 'NED'
        self.current_pose.pose.pose.position.x =0
        self.current_pose.pose.pose.position.y =0
        self.current_pose.pose.pose.position.z =0
        

        q = quaternion_from_euler(self.roll,self.pitch,self.yaw)
        self.current_pose.pose.pose.orientation.x = q[0]
        self.current_pose.pose.pose.orientation.y = q[1]
        self.current_pose.pose.pose.orientation.z = q[2]
        self.current_pose.pose.pose.orientation.w = q[3]
        self.current_pose.pose.covariance = [
            0.1, 0.01, 0.02, 0.00, 0.00, 0.00,   # Row 1: variance of x and covariances with y, z, roll, pitch, yaw
            0.01, 0.1, 0.01, 0.00, 0.00, 0.00,   # Row 2: covariance with x, variance of y, covariances with z, roll, pitch, yaw
            0.02, 0.01, 0.1, 0.00, 0.00, 0.00,   # Row 3: covariance with x, y, variance of z, covariances with roll, pitch, yaw
            0.00, 0.00, 0.00, 0.1, 0.01, 0.02,   # Row 4: covariance with x, y, z, variance of roll, covariances with pitch, yaw
            0.00, 0.00, 0.00, 0.01, 0.1, 0.01,   # Row 5: covariance with x, y, z, roll, variance of pitch, covariance with yaw
            0.00, 0.00, 0.00, 0.02, 0.01, 0.1    # Row 6: covariance with x, y, z, roll, pitch, variance of yaw
        ]
        self.state = PoseStamped()
        self.state.header.frame_id = "NED"
        self.state.pose.position.x = 0
        self.state.pose.position.y = 0
        self.state.pose.position.z = 0
        self.state.pose.orientation = self.current_pose.pose.pose.orientation
   
        self.path = Path()
        self.path.header.frame_id ='NED'
        # self.path.poses.append(self.state)

        # new_pose = PoseStamped()
        # new_pose = self.state
        # new_pose.pose.position.x = 10
        # self.state.pose.position.x = 20
        # self.state.pose.position.y = 0
        # self.state.pose.position.z = 0
        # self.state.pose.orientation = self.current_pose.pose.pose.orientation
        # self.path.poses.append(new_pose)
        


        # self.velocity_x = 0.1
        # self.velocity_y = 0.1
        # self.velocity_z = 0.0

        # Angular velocity in radians per second
        # self.angular_velocity_yaw = math.radians(10)  # 10 degrees per second
        self.frequency = 10
        self.rate = rospy.Rate(self.frequency)  # 10 Hz

  
        self.start_time = time.time()

        # subscribe to twist 
        self.sub1 = rospy.Subscriber('velocity_setpoint', Twist, self.velocity_callback)
       
        self.pub1 = rospy.Publisher('/state',PoseWithCovarianceStamped, queue_size=1)
        self.pub2 = rospy.Publisher('/dvl/twist', TwistStamped, queue_size=1)
       
        self.pub4 = rospy.Publisher('robot_path',Path, queue_size= 1)

    def publish_pose(self):
        # msg = PoseWithCovarianceStamped()        
        # msg.header.frame_id = "NED"
        # msg.pose.pose.position.x = self.pos_x
        # msg.pose.pose.position.y = self.pos_y
        # msg.pose.pose.position.z = self.pos_z

        # q = quaternion_from_euler(self.roll,self.pitch,self.yaw)

        # msg.pose.pose.orientation.x = q[0]
        # msg.pose.pose.orientation.y = q[1]
        # msg.pose.pose.orientation.z = q[2]
        # msg.pose.pose.orientation.w = q[3]

        # msg.pose.covariance = [
        #     0.1, 0.01, 0.02, 0.00, 0.00, 0.00,   # Row 1: variance of x and covariances with y, z, roll, pitch, yaw
        #     0.01, 0.1, 0.01, 0.00, 0.00, 0.00,   # Row 2: covariance with x, variance of y, covariances with z, roll, pitch, yaw
        #     0.02, 0.01, 0.1, 0.00, 0.00, 0.00,   # Row 3: covariance with x, y, variance of z, covariances with roll, pitch, yaw
        #     0.00, 0.00, 0.00, 0.1, 0.01, 0.02,   # Row 4: covariance with x, y, z, variance of roll, covariances with pitch, yaw
        #     0.00, 0.00, 0.00, 0.01, 0.1, 0.01,   # Row 5: covariance with x, y, z, roll, variance of pitch, covariance with yaw
        #     0.00, 0.00, 0.00, 0.02, 0.01, 0.1    # Row 6: covariance with x, y, z, roll, pitch, variance of yaw
        # ]

        # self.current_pose.header.frame_id = "NED"
        # self.current_pose.pose.pose.position.x


        # self.pub1.publish(msg)
        self.pub1.publish(self.current_pose)

        new_pose = PoseStamped()
        new_pose.header.frame_id = self.current_pose.header.frame_id
        new_pose.pose = self.current_pose.pose.pose
        rospy.loginfo(new_pose)
        
        self.path.poses.append(new_pose)

        self.pub1.publish(self.current_pose)
        self.pub3.publish(self.path)

# 

    def linear_velocity_controller(self, v, v_goal):
        if not v == v_goal:
            v_error = v_goal - v 
            # rospy.loginfo(f"v_error : {v_error}")
    
            if v_error > 0:
                # positive to higher positive velocity works 
                if v >= 0 and v_goal >= 0:
                    v += self.max_linear_acceleration*(1/self.frequency)
                    
                    # prevent overshoot 
                    if v > v_goal:
                        v = v_goal

                # negative velocity to lower negative velocity 
                # elif v <= 0 and v_goal <= 0:
                elif v <= 0:
                    
                    v += self.max_linear_deceleration*(1/self.frequency)
                    
                    # prevent overshoot 
                    if v > v_goal:
                        v = v_goal


            if v_error < 0:
                # Positive velocity to lower positive velocity works 
                if v >= 0:
                    v -= self.max_linear_deceleration*(1/self.frequency)
                    # prevent overshoot 
                    if v < v_goal:
                        v = v_goal

                # negative velocity to higher negative velocity 
                elif v <= 0 and v_goal <= 0:
                    v -= self.max_linear_acceleration*(1/self.frequency)
                    
                    # prevent overshoot 
                    if v < v_goal:
                        v = v_goal


        return v            
            
    def angular_velocity_controller(self, v, v_goal):
        if not v == v_goal:
            v_error = v_goal - v 
            # rospy.loginfo(f"v_error : {v_error}")
    
            if v_error > 0:
                # positive to higher positive velocity works 
                if v >= 0 and v_goal >= 0:
                    v += self.max_angular_acceleration*(1/self.frequency)
                    
                    
                    # prevent overshoot 
                    if v > v_goal:
                        v = v_goal

                # negative velocity to lower negative velocity 
                # elif v <= 0 and v_goal <= 0:
                elif v <= 0:
                    
                    v += self.max_angular_deceleration*(1/self.frequency)
                    
                    # prevent overshoot 
                    if v > v_goal:
                        v = v_goal


            if v_error < 0:
                # Positive velocity to lower positive velocity works 
                if v >= 0:
                    v -= self.max_angular_deceleration*(1/self.frequency)
                    # prevent overshoot 
                    if v < v_goal:
                        v = v_goal

                # negative velocity to higher negative velocity 
                elif v <= 0 and v_goal <= 0:
                    v -= self.max_angular_acceleration*(1/self.frequency)
                    
                    # prevent overshoot 
                    if v < v_goal:
                        v = v_goal

        # rospy.loginfo(f"v ={v}")

        return v
                
                
    def velocity_callback(self,msg:Twist):

        if self.apply_acc_limits:

            velocity_setpoint = Twist()
            velocity_setpoint = msg
      
            self.velocity.linear.x = self.linear_velocity_controller(self.velocity.linear.x, velocity_setpoint.linear.x)
            self.velocity.linear.y = self.linear_velocity_controller(self.velocity.linear.y, velocity_setpoint.linear.y)
            self.velocity.linear.z = self.linear_velocity_controller(self.velocity.linear.z, velocity_setpoint.linear.z)
            self.velocity.angular.z = self.angular_velocity_controller(self.velocity.angular.z, velocity_setpoint.angular.z)
        else:
            self.velocity = msg
        
        # rospy.loginfo('got velocity setpoint')

    def publish_velocity(self):
        msg = TwistStamped()
        msg.header.frame_id = "base_link"
        msg.twist = self.velocity
        self.pub2.publish(msg)

    def update_position(self):
        # extracting the current pose as a rotation matrix and converting to 3 by 3
        current_rotation = euler_matrix(self.roll, self.pitch, self.yaw)
       
        # linear displacement in the body frame
        x_rel = self.velocity.linear.x * (1.0 / self.frequency)
        y_rel = self.velocity.linear.y * (1.0 / self.frequency)
        z_rel = self.velocity.linear.z * (1.0 / self.frequency)
        p_rel = np.array([[x_rel],[y_rel],[z_rel]])

        # converting the displacement due to linear velocity to be relative to the inertial frame (NED)
        # self.pos_x += current_rotation[0,0:3]@ p_rel
        # self.pos_y += current_rotation[1,0:3]@ p_rel
        # self.pos_z += current_rotation[2,0:3]@ p_rel

        # self.yaw += self.velocity.angular.z* (1.0 / self.frequency)

        x = current_rotation[0,0:3]@ p_rel
        y = current_rotation[1,0:3]@ p_rel
        z = current_rotation[2,0:3]@ p_rel

        self.current_pose.pose.pose.position.x += x[0]
        self.current_pose.pose.pose.position.y += y[0]
        self.current_pose.pose.pose.position.z += z[0]

        self.yaw += self.velocity.angular.z* (1.0 / self.frequency)

        q = quaternion_from_euler(self.roll,self.pitch,self.yaw)
        self.current_pose.pose.pose.orientation.x = q[0]
        self.current_pose.pose.pose.orientation.y = q[1]
        self.current_pose.pose.pose.orientation.z = q[2]
        self.current_pose.pose.pose.orientation.w = q[3]
        self.current_pose.pose.covariance = [
            0.1, 0.01, 0.02, 0.00, 0.00, 0.00,   # Row 1: variance of x and covariances with y, z, roll, pitch, yaw
            0.01, 0.1, 0.01, 0.00, 0.00, 0.00,   # Row 2: covariance with x, variance of y, covariances with z, roll, pitch, yaw
            0.02, 0.01, 0.1, 0.00, 0.00, 0.00,   # Row 3: covariance with x, y, variance of z, covariances with roll, pitch, yaw
            0.00, 0.00, 0.00, 0.1, 0.01, 0.02,   # Row 4: covariance with x, y, z, variance of roll, covariances with pitch, yaw
            0.00, 0.00, 0.00, 0.01, 0.1, 0.01,   # Row 5: covariance with x, y, z, roll, variance of pitch, covariance with yaw
            0.00, 0.00, 0.00, 0.02, 0.01, 0.1    # Row 6: covariance with x, y, z, roll, pitch, variance of yaw
        ]

        self.pub1.publish(self.current_pose)

    def broadcast_transform(self):
        # Send the updated transform
        # self.br.sendTransform(
        #     (self.pos_x, self.pos_y, self.pos_z),
        #     quaternion_from_euler(self.roll, self.pitch, self.yaw),
        #     rospy.Time.now(),
        #     'base_link',   # child frame
        #     'NED'        # parent frame, typically 'world' or 'map'
        # )
        self.br.sendTransform(
            (self.current_pose.pose.pose.position.x, self.current_pose.pose.pose.position.y, self.current_pose.pose.pose.position.z),
            quaternion_from_euler(self.roll, self.pitch, self.yaw),
            rospy.Time.now(),
            'base_link',   # child frame
            'NED'        # parent frame, typically 'world' or 'map'
        )

    def publish_path(self):
        # self.new_pose = PoseStamped()
        # self.new_pose.header.frame_id = self.current_pose.header.frame_id
        
        # self.new_pose.pose.position.x = self.current_pose.pose.pose.position.x
        # self.new_pose.pose.position.y = self.current_pose.pose.pose.position.y
        # self.new_pose.pose.position.z = self.current_pose.pose.pose.position.z
        # self.new_pose.pose.orientation.x = self.current_pose.pose.pose.orientation.x
        # self.new_pose.pose.orientation.y = self.current_pose.pose.pose.orientation.y
        # self.new_pose.pose.orientation.z = self.current_pose.pose.pose.orientation.z
        # self.new_pose.pose.orientation.w = self.current_pose.pose.pose.orientation.w       
        # self.state.header.frame_id = "NED"
        # self.state.pose = self.current_pose.pose.pose
        new_pose = PoseStamped()
        new_pose.header.frame_id = 'NED'
        new_pose.pose.position.x = 0 + self.current_pose.pose.pose.position.x
        new_pose.pose.position.y = 0 + self.current_pose.pose.pose.position.y
        new_pose.pose.position.z = 0 + self.current_pose.pose.pose.position.z
        new_pose.pose.orientation.x = 0 + self.current_pose.pose.pose.orientation.x
        new_pose.pose.orientation.y = 0 + self.current_pose.pose.pose.orientation.y
        new_pose.pose.orientation.z = 0 + self.current_pose.pose.pose.orientation.z
        new_pose.pose.orientation.w = 0 + self.current_pose.pose.pose.orientation.w  
       
     

        self.path.poses.append(new_pose)

        self.pub4.publish(self.path)

       


    def run(self):
        self.publish_velocity()
        self.update_position()
        self.publish_path()
        self.broadcast_transform()
        # self.publish_pose()
        self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('kinematic_simulation')
    rospy.loginfo("kinematic simulator node running")
    sim = kinematic_simulator()
    
    while not rospy.is_shutdown():
        sim.run()
        # sim.publish_velocity()
        # sim.update_position()
        # sim.broadcast_transform()
        # sim.publish_pose()
        # sim.publish_path()

            
        # path.poses.append(sim.get_pose())
        # sim.pub3.publish(path)
        
        
        
        # rospy.loginfo_throttle(5,sim.path)
        # sim.rate.sleep()

    # except rospy.ROSInterruptException:
    #     pass
