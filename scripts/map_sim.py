#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
# from geometry_msgs.msg import Pose



class world_map:
    def __init__(self): 
        queue_size = 1
        self.rate = rospy.Rate(10)
        self.pub_map = rospy.Publisher('occupancy_map', OccupancyGrid, queue_size=queue_size)
        
        # initializing the occupancy grid
        self.occupancy = OccupancyGrid()
        self.occupancy.header.frame_id = "NED"
        self.occupancy.info.resolution = 0.25
        self.occupancy.info.width = int(100/0.25)
        self.occupancy.info.height = int(100/0.25)
        self.occupancy.info.origin.position.x = -10.0
        self.occupancy.info.origin.position.y = -10.0
        self.occupancy.info.origin.position.z = 0.0
        self.occupancy.info.origin.orientation.x = 0.0
        self.occupancy.info.origin.orientation.y = 0.0
        self.occupancy.info.origin.orientation.z = 0.0
        self.occupancy.info.origin.orientation.w = 0.0
    

    def publish_map(self):
        self.occupancy.data = self.grid_data.flatten().tolist()
        self.pub_map.publish(self.occupancy)

    def init_map(self):
        self.occupancy.header.frame_id = "NED"
        self.occupancy.info.resolution = 0.25
        self.occupancy.info.width = 1000/0.25
        self.occupancy.info.height = 1000/0.25
        self.occupancy.info.origin.position.x = 0.0
        self.occupancy.info.origin.position.y = 0.0
        self.occupancy.info.origin.position.z = 0.0
        self.occupancy.info.origin.orientation.x = 0.0
        self.occupancy.info.origin.orientation.y = 0.0
        self.occupancy.info.origin.orientation.z = 0.0
        self.occupancy.info.origin.orientation.w = 0.0

    def create_grid(self):
        self.grid_data = np.zeros((self.occupancy.info.height, self.occupancy.info.width), dtype=np.int8)

    def fill_area(self,row_start, row_end, column_start, column_end, fill_value):
        """
        Fill a specified rectangular area of the grid with a given fill value.
        """
        self.grid_data[row_start:row_end, column_start:column_end] = fill_value
   
    def populate_random_obstacles(self, num_obstacles=20, obstacle_size=10):
        """
        Randomly fill the grid with a specified number of obstacles.
        Each obstacle is a square of 'obstacle_size' x 'obstacle_size'.
        """
        for _ in range(num_obstacles):
            row_start = np.random.randint(15, int(self.occupancy.info.height) - obstacle_size)
            column_start = np.random.randint(15, int(self.occupancy.info.width) - obstacle_size)
            self.fill_area(row_start, row_start + obstacle_size, column_start, column_start + obstacle_size, 100)

    def populate_occupancy(self):
        self.occupancy.data = self.grid_data.flatten().tolist()



if __name__ == '__main__':
    rospy.init_node('map_sim')
    rospy.loginfo_once("publishing map")
    world= world_map()
    world.create_grid()
    world.populate_random_obstacles()
    


    # world.publish_map()


    while not rospy.is_shutdown():    

        world.publish_map()
        world.rate.sleep()