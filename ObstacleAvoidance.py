import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class ObstacleAvoidance:
    def __init__(self):
        self.visited_points = []
        rospy.Subscriber("/summit_xl/scan_front", LaserScan, self.laser_callback)
        rospy.Subscriber("/summit_xl/robotnik_base_control/odom", Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher("/summit_xl/move/cmd_vel", Twist, queue_size=10)
        self.current_position = None
        self.min_distance = None
    
    def laser_callback(self,scan):

        self.min_distance = min(scan.ranges)
        #print(self.min_distance)

    def odom_callback(self, odom):
        
        position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        angle = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        current_position = (round(position.x, 2), round(position.y, 2), round(angle[2], 2))
        
        # Check if the obstacle is close enough to the robot
        if current_position not in self.visited_points and self.min_distance is not None:
            self.visited_points.append(current_position)
            print(self.visited_points)
            #print(self.visited_points)

            if self.min_distance < 2:   
            # Create a twist message to avoid the obstacle
                twist = Twist()
                twist.linear.x = 0
                twist.angular.z = -1
                self.cmd_vel_pub.publish(twist)
            else:   
                twist = Twist()
                twist.linear.x = 1
                twist.angular.z = 0
                self.cmd_vel_pub.publish(twist)

        else:   
                twist = Twist()
                twist.linear.x = 0
                twist.angular.z = 1
                self.cmd_vel_pub.publish(twist)

if __name__ == "__main__":
    rospy.init_node("obstacle_avoidance")
    avoidance = ObstacleAvoidance()
    rospy.spin()
