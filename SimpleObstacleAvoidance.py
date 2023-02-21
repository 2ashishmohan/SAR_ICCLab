#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

rospy.init_node('ObstacleDetector')
cmd_vel_pub = rospy.Publisher('/summit_xl/move/cmd_vel', Twist, queue_size=10)
visited_points = []
def move_robot(linear_speed, angular_speed):
    twist = Twist()
    twist.linear.x = linear_speed
    twist.angular.z = angular_speed
    cmd_vel_pub.publish(twist)


def laser_callback(msg):

    ranges = msg.ranges
    min_range = min(ranges)

    if min_range < 0.5:
        move_robot(0.0, 0.0)
    else:
    
        move_robot(0.5, 0.0)


laser_sub = rospy.Subscriber('/summit_xl/scan_front', LaserScan, laser_callback)

rospy.spin()  
