import rospy
import random
import numpy as np
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped
import psutil
import rosnode
import tf
import math
import os


class MOveBaseClient:
    def __init__(self):

        self.client = actionlib.SimpleActionClient('/summit_xl/move_base', MoveBaseAction)
        self.pode_sub = rospy.Subscriber('/summit_xl/amcl_pose', PoseStamped, self.pose_callback)
        self.grid_sub = rospy.Subscriber('/summit_xl/map', OccupancyGrid, self.grid_callback)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(timeout=rospy.Duration(300.0)) # takes a while for sim environment to bring up
        if not wait:
            rospy.logerr("Action server not available!")
        rospy.loginfo("Connected to move base server")
        self.move_goal = MoveBaseGoal()
        self.rate = rospy.Rate(2)
        self.pose = PoseStamped()
        self.grid = OccupancyGrid()
        self.grid_res = 0
        

    def pose_callback(self, msg):
            self.pose = msg

    def grid_callback(self, msg):
        self.grid = msg
        self.grid_res = msg.info.resolution


    def run(self):
        # The method would fing the extreme points from the generated map and define a random goal position from those points
        while not rospy.is_shutdown():
            unexplored_goals = []
            map_width = self.grid.info.width
            map_height = self.grid.info.height
            map_origin_x = self.grid.info.origin.position.x
            map_origin_y = self.grid.info.origin.position.y
            map_data = self.grid.data

            for i in range(map_height):
                for j in range(map_width):
                    if map_data[i*map_width+j] == -1:
                        x = map_origin_x + (j + 0.5) * self.grid_res
                        y = map_origin_y + (j + 0.5) * self.grid_res
                        unexplored_goals.append((x,y))

            if unexplored_goals:
                x1, y1 = random.choice(unexplored_goals)
                angle = math.atan2(y1 - self.pose.pose.position.y, x1- self.pose.pose.position.x)
                quat = tf.transformations.quaternion_from_euler(0, 0, angle)
                print(x1, y1)
                self.move_goal.target_pose.header.frame_id = "summit_xl_map"
                self.move_goal.target_pose.header.stamp = rospy.Time.now()
                self.move_goal.target_pose.pose.position.x = x1/5
                self.move_goal.target_pose.pose.position.y = y1/5
                self.move_goal.target_pose.pose.orientation.x = quat[0]
                self.move_goal.target_pose.pose.orientation.y = quat[1]
                self.move_goal.target_pose.pose.orientation.z = quat[2]
                self.move_goal.target_pose.pose.orientation.w = quat[3]       
                rospy.loginfo("Sending goal pose to Action Server")
                self.client.send_goal(self.move_goal)
                self.client.wait_for_result()
            self.rate.sleep()



if __name__ == '__main__':
    rospy.init_node('movebase_client')
    mbc = MOveBaseClient()
    mbc.run()
    
