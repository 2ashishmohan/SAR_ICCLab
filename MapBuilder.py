import rospy
import random
import numpy as np
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import psutil
import rosnode


def move_client():
    client = actionlib.SimpleActionClient('/summit_xl/move_base', MoveBaseAction)
    rospy.Subscriber('/summit_xl/amcl_pose', PoseStamped, pose_callback)
    rospy.loginfo("Waiting for move_base action server...")
    wait = client.wait_for_server(timeout=rospy.Duration(300.0)) # takes a while for sim environment to bring up
    if not wait:
        rospy.logerr("Action server not available!")
    rospy.loginfo("Connected to move base server")
    move_goal = MoveBaseGoal()
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():

        goal = generate_random_goal()
        move_goal.target_pose = goal
        rospy.loginfo("Sending goal pose to Action Server")
        client.send_goal(move_goal)
        client.wait_for_result()
        rate.sleep()

def pose_callback(msg):
        global pose 
        pose = msg
    
def generate_random_goal():
        angle = random.uniform(0, 2*np.pi)
        distance = random.uniform(0, 10)
        x_offset = distance*np.cos(angle)
        y_offset = distance*np.sin(angle)
        goal = PoseStamped()
        goal.header = pose.header
        goal.pose.position.x = pose.pose.position.x + x_offset
        goal.pose.position.y = pose.pose.position.y + y_offset
        goal.pose.orientation = pose.pose.orientation
        return goal   
   
    


if __name__ == '__main__':
    rospy.init_node('movebase_client')
    move_client()
    
