import rospy
import random
import numpy as np
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
import psutil
import rosnode
import tf
import math

class MoveBaseClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('/summit_xl/move_base', MoveBaseAction)
        rospy.Subscriber('/summit_xl/amcl_pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(timeout=rospy.Duration(300.0)) # takes a while for sim environment to bring up
        if not wait:
            rospy.logerr("Action server not available!")
        rospy.loginfo("Connected to move base server")
        self.move_goal = MoveBaseGoal()
        self.rate = rospy.Rate(2)
        self.pose = PoseStamped()
        self.map = OccupancyGrid()

    def pose_callback(self, msg):
        self.pose = msg

    def map_callback(self, msg):
        self.map = msg

    def run(self):
        while not rospy.is_shutdown():
            goal = self.generate_random_goal()
            while not self.is_goal_in_unexplored_area(goal):
                goal = self.generate_random_goal()
            self.move_goal.target_pose = goal
            rospy.loginfo("Sending goal pose to Action Server")
            self.client.send_goal(self.move_goal)
            self.client.wait_for_result()
            self.rate.sleep()

    def is_goal_in_unexplored_area(self, goal):
        x, y = self.get_grid_cell(goal.pose.position.x, goal.pose.position.y)
        if x >= self.map.info.width or x < 0 or y >= self.map.info.height or y < 0:
            return False
        cell_value = self.map.data[y * self.map.info.width + x]
        return cell_value == -1

    def get_grid_cell(self, x, y):
        grid_x = int((x - self.map.info.origin.position.x) / self.map.info.resolution)
        grid_y = int((y - self.map.info.origin.position.y) / self.map.info.resolution)
        return grid_x, grid_y

    def generate_random_goal(self):
        angle = random.uniform(0, 2*np.pi)
        distance = random.uniform(10, 20)
        x_offset = distance*np.cos(angle)
        y_offset = distance*np.sin(angle)
        goal = PoseStamped()
        goal.header.frame_id = "summit_xl_map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = self.pose.pose.position.x + x_offset
        goal.pose.position.y = self.pose.pose.position.y + y_offset
        quaternion = tf.transformations.quaternion_from_euler(0, 0, math.atan2(y_offset, x_offset))
        goal.pose.orientation.x = quaternion[0]
        goal.pose.orientation.y = quaternion[1]
        goal.pose.orientation.z = quaternion[2]
        goal.pose.orientation.w = quaternion[3]
        return goal

if __name__ == '__main__':
    rospy.init_node('movebase_client')
    mbc = MoveBaseClient()
    mbc.run()
