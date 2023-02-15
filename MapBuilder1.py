import rospy
import random
import numpy as np
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
import tf
import math

class MoveBaseClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.grid_sub = rospy.Subscriber('/map', OccupancyGrid, self.grid_callback)
        self.pose_sub = rospy.Subscriber('/amcl_pose', PoseStamped, self.pose_callback)
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
        while not rospy.is_shutdown():
            unexplored_goals = self.get_unexplored_goals()
            if unexplored_goals:
                goal = random.choice(unexplored_goals)
                rospy.loginfo("Sending goal pose to Action Server")
                self.client.send_goal(goal)
                self.client.wait_for_result()
            self.rate.sleep()

    def get_unexplored_goals(self):
        unexplored_goals = []
        map_width = self.grid.info.width
        map_height = self.grid.info.height
        map_origin_x = self.grid.info.origin.position.x
        map_origin_y = self.grid.info.origin.position.y
        map_data = self.grid.data

        for i in range(map_height):
            for j in range(map_width):
                if map_data[i*map_width+j] == -1:  # unexplored cell
                    x = map_origin_x + (j + 0.5) * self.grid_res
                    y = map_origin_y + (i + 0.5) * self.grid_res
                    angle = math.atan2(y - self.pose.pose.position.y, x - self.pose.pose.position.x)
                    quat = tf.transformations.quaternion_from_euler(0, 0, angle)
                    goal = MoveBaseGoal()
                    goal.target_pose.header.frame_id = "map"
                    goal.target_pose.pose.position.x = x
                    goal.target_pose.pose.position.y = y
                    goal.target_pose.pose.orientation.x = quat[0]
                    goal.target_pose.pose.orientation.y = quat[1]
                    goal.target_pose.pose.orientation.z = quat[2]
                    goal.target_pose.pose.orientation.w = quat[3]
                    unexplored_goals.append(goal)

        return unexplored_goals


if __name__ == '__main__':
    rospy.init_node('movebase_client')
    mbc = MoveBaseClient()
    mbc.run()

