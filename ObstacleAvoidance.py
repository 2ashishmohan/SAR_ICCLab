import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidance:
    def __init__(self):
        self.visited_points = []
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def laser_callback(self, scan):
        # Get the distance of the closest obstacle
        min_distance = min(scan.ranges)

        # Check if the obstacle is close enough to the robot
        if min_distance < 0.5:
            # Create a twist message to avoid the obstacle
            twist = Twist()
            twist.linear.x = 0
            twist.angular.z = 1
            self.cmd_vel_pub.publish(twist)
        else:
            # Check if the robot has already been at this location
            current_position = (round(scan.ranges[0], 2), round(scan.ranges[len(scan.ranges)//2], 2))
            if current_position not in self.visited_points:
                # If not, add it to the list of visited points
                self.visited_points.append(current_position)
                # Create a twist message to move the robot forward
                twist = Twist()
                twist.linear.x = 0.5
                twist.angular.z = 0
                self.cmd_vel_pub.publish(twist)

if __name__ == "__main__":
    rospy.init_node("obstacle_avoidance")
    avoidance = ObstacleAvoidance()
    rospy.spin()
