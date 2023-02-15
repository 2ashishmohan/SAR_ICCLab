import rospy
import numpy as np
import random
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
import tf

class RandomExplorer:
    def __init__(self):
        rospy.init_node('random_explorer')
        self.rate = rospy.Rate(2)
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()

        self.map = None
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

    def map_callback(self, msg):
        self.map = msg

    def run(self):
        while not rospy.is_shutdown():
            # Wait until we have a map
            while self.map is None and not rospy.is_shutdown():
                rospy.loginfo("Waiting for occupancy grid")
                self.rate.sleep()

            # Get the shape of the map and the resolution
            width = self.map.info.width
            height = self.map.info.height
            resolution = self.map.info.resolution

            # Convert the occupancy grid to a numpy array
            occupancy = np.array(self.map.data).reshape((height, width))

            # Create a mask of the unexplored region (i.e. cells with unknown occupancy)
            unexplored_mask = (occupancy == -1)

            # Check if there are any unexplored cells
            if np.any(unexplored_mask):
                # Get a list of unexplored cells
                unexplored_cells = np.argwhere(unexplored_mask)

                # Select a random unexplored cell
                random_cell = unexplored_cells[random.randint(0, len(unexplored_cells) - 1)]

                # Calculate the position of the cell in meters
                x = random_cell[1] * resolution + self.map.info.origin.position.x
                y = random_cell[0] * resolution + self.map.info.origin.position.y

                # Create a move_base goal at the selected position
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = x
                goal.target_pose.pose.position.y = y

                # Calculate the orientation of the goal
                quaternion = tf.transformations.quaternion_from_euler(0, 0, random.uniform(0, 2*np.pi))
                goal.target_pose.pose.orientation.x = quaternion[0]
                goal.target_pose.pose.orientation.y = quaternion[1]
                goal.target_pose.pose.orientation.z = quaternion[2]
                goal.target_pose.pose.orientation.w = quaternion[3]

                # Send the goal to move_base
                rospy.loginfo("Sending goal to move_base")
                self.client.send_goal(goal)
                self.client.wait_for_result()

            self.rate.sleep()

if __name__ == '__main__':
    explorer = RandomExplorer()
    explorer.run()
