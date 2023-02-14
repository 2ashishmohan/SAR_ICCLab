import rospy
import random
import numpy as np
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import psutil
import rosnode
import tf
import math

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
        self.rate = rospy.Rate(2)
        self.pose = PoseStamped()

    def pose_callback(self, msg):
            self.pose = msg

    def run(self):
        while not rospy.is_shutdown():
            #goal = self.generate_random_goal()
            
            angle = random.uniform(0, 2*np.pi)
            distance = random.uniform(10, 20)
            #distance = 20
            x_offset = distance*np.cos(angle)
            y_offset = distance*np.sin(angle)
            self.move_goal.target_pose.header.frame_id = "summit_xl_map"
            self.move_goal.target_pose.header.stamp = rospy.Time.now()
            self.move_goal.target_pose.pose.position.x = self.pose.pose.position.x + x_offset
            self.move_goal.target_pose.pose.position.y = self.pose.pose.position.y + y_offset
            
            quaternion = tf.transformations.quaternion_from_euler(0, 0, math.atan2(y_offset, x_offset))
            self.move_goal.target_pose.pose.orientation.x = quaternion[0]
            self.move_goal.target_pose.pose.orientation.y = quaternion[1]
            self.move_goal.target_pose.pose.orientation.z = quaternion[2]
            self.move_goal.target_pose.pose.orientation.w = quaternion[3]

            #self.move_goal.target_pose = goal
            rospy.loginfo("Sending goal pose to Action Server")
            self.client.send_goal(self.move_goal)
            self.client.wait_for_result()
            self.rate.sleep()
    '''
    def generate_random_goal(self):
            angle = random.uniform(0, 2*np.pi)
            distance = random.uniform(0, 1)
            x_offset = distance*np.cos(angle)
            y_offset = distance*np.sin(angle)
            goal = PoseStamped()
            goal.header = self.pose.header
            goal.pose.position.x = self.pose.pose.position.x + x_offset
            goal.pose.position.y = self.pose.pose.position.y + y_offset
            goal.pose.orientation = self.pose.pose.orientation
            return goal   
  
    '''


if __name__ == '__main__':
    rospy.init_node('movebase_client')
    mbc = MOveBaseClient()
    mbc.run()
    
