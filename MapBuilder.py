# Import Libraries
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
        rospy.Subscriber('/summit_xl/amcl_pose', PoseStamped, self.pose_callback)

        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(timeout=rospy.Duration(300.0)) # takes a while for sim environment to bring up
        if not wait:
            rospy.logerr("Action server not available!")
        rospy.loginfo("Connected to move base server")
        self.move_goal = MoveBaseGoal()
        self.rate = rospy.Rate(1)
        self.pose = PoseStamped()
        

    def pose_callback(self, msg):
            self.pose = msg

    def run(self):
        # The method generates random goal position with respect to the current position of the robot
        while not rospy.is_shutdown():
            angle = random.uniform(0, 2*np.pi)
            x_offset = random.uniform(-10,10)
            y_offset = random.uniform(-10, 10)
            self.move_goal.target_pose.header.frame_id = "summit_xl_map"
            self.move_goal.target_pose.header.stamp = rospy.Time.now()
            self.move_goal.target_pose.pose.position.x = self.pose.pose.position.x + x_offset
            self.move_goal.target_pose.pose.position.y = self.pose.pose.position.y + y_offset
            print(self.pose.pose.position.x + x_offset, self.pose.pose.position.y + y_offset)
            quaternion = tf.transformations.quaternion_from_euler(0, 0, math.atan2(y_offset, x_offset))
            self.move_goal.target_pose.pose.orientation.x = quaternion[0]
            self.move_goal.target_pose.pose.orientation.y = quaternion[1]
            self.move_goal.target_pose.pose.orientation.z = quaternion[2]
            self.move_goal.target_pose.pose.orientation.w = quaternion[3]

            rospy.loginfo("Sending goal pose to Action Server")
            self.client.send_goal(self.move_goal)
            self.client.wait_for_result()
            self.rate.sleep()

        self.stop_mapping()

    def stop_mapping(self):
        nodes = rosnode.get_node_name()
        for node in nodes:
            if 'gmapping' in node:
                rospy.loginfo("Stopping %s......", node)
                os.system('rosnode kill'  + node)
    


if __name__ == '__main__':
    rospy.init_node('movebase_client')
    mbc = MOveBaseClient()
    mbc.run()
    
