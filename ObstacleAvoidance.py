#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class ObstAvoid:
    def __init__(self):
        rospy.init_node('obstacle_avoidance')
        self.odom_sub = rospy.Subscriber('summit_xl/amcl_pose', Odometry, self.odom_callback)
        self.laser_sub = rospy.Subscriber('/summit_xl/scan_front', LaserScan, self.laser_callback)
        self.cmd_vel_pub = rospy.Publisher('/summit_xl/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.min_distance = 0.5

    def odom_callback(self, msg):
        self.pose = msg.pose.pose

    def laser_callback(self, msg):
        self.ranges = msg.ranges

    def move(self):
        while not rospy.is_shutdown():
            min_range = min(self.ranges)
            if min_range < self.min_distance:
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
            else:
                twist = Twist()
                twist.linear.x = 0.5
                self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = ObstAvoid()
        node.move()
    except rospy.ROSInterruptException:
        pass

        


