#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped

def move_robot():
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1

    client.send_goal(goal)

    client.wait_for_result()

rospy.init_node('move_robot')


initial_pose = PoseWithCovarianceStamped()
initial_pose.header.frame_id = "map"
initial_pose.pose.pose.position.x = 0
initial_pose.pose.pose.position.y = 0
initial_pose.pose.pose.orientation.w = 1
initial_pose_publisher = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
initial_pose_publisher.publish(initial_pose)

while not rospy.is_shutdown():
    x = 3
    y = 3
    move_robot()


