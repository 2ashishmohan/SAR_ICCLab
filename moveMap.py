#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

rospy.init_node('move_robot')
cmd_vel_pub = rospy.Publisher('/summit_xl/move/cmd_vel', Twist, queue_size=10)
grid = None

def map_callback(data):
    global grid
    grid = data

def move_robot(linear_speed, angular_speed):
    twist = Twist()
    twist.linear.x = linear_speed
    twist.angular.z = angular_speed
    cmd_vel_pub.publish(twist) 

def laser_callback(data):
    scan = data.ranges

    min_left = min(scan[0:30])
    min_right = min(scan[-30:])
    min_front = min(scan[150:210])
    print(min_front)
    if min_front < 0.5:
        move_robot(0.0,0.0)
        
        if min_left < min_right:
            move_robot(0.0,-0.2)
        else:
            move_robot(0.0,0.2)
            
    else:
        move_robot(0.5,0.0)

            

laser_sub = rospy.Subscriber('/summit_xl/scan_front', LaserScan, laser_callback)

map_sub = rospy.Subscriber('/summit_xl/map', OccupancyGrid, map_callback)

rospy.spin()  