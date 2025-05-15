import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped
import random
import math
import tf_transformations
import numpy as np

class SearchController(Node):
    def __init__(self):
        super().__init__('search_controller')

        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        self.subscription_costmap = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            10)

        self.subscription_detection = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            10)

        self.search_target = 'bottle'  # Target to search for
        self.found = False
        self.visited = set()
        self.failed_goals = set()
        self.generated_goals = []
        self.history_radius = 0.5  # Meters
        self.goal_attempt_limit = 20

        self.latest_costmap = None
        self.timer = self.create_timer(1.0, self.search_loop)
        self.refresh_timer = self.create_timer(15.0, self.refresh_map)

    def costmap_callback(self, msg):
        self.latest_costmap = msg
        if not self.generated_goals:
            self.generated_goals = self.generate_waypoints(msg)

    def refresh_map(self):
        if self.latest_costmap:
            self.generated_goals = self.generate_waypoints(self.latest_costmap)
            self.get_logger().info("Waypoints updated using latest costmap.")

    def generate_waypoints(self, costmap_msg):
        resolution = costmap_msg.info.resolution
        width = costmap_msg.info.width
        height = costmap_msg.info.height
        origin = costmap_msg.info.origin

        data = np.array(costmap_msg.data).reshape((height, width))
        waypoints = []

        for _ in range(self.goal_attempt_limit * 3):  # Try generating more than needed
            x_pix = random.randint(0, width - 1)
            y_pix = random.randint(0, height - 1)

            if data[y_pix, x_pix] != 0:
                continue  # Skip unknown or occupied

            x = origin.position.x + (x_pix + 0.5) * resolution
            y = origin.position.y + (y_pix + 0.5) * resolution

            if self.is_near_visited(x, y):
                continue  # Avoid close to previously visited

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation = self.random_orientation()

            waypoints.append(pose)

            if len(waypoints) >= self.goal_attempt_limit:
                break

        self.get_logger().info(f"Generated {len(waypoints)} filtered waypoints.")
        return waypoints

    def is_near_visited(self, x, y):
        for vx, vy in self.visited:
            dist = math.hypot(vx - x, vy - y)
            if dist < self.history_radius:
                return True
        return False

    def random_orientation(self):
        yaw = random.uniform(-math.pi, math.pi)
        q = tf_transformations.quaternion_from_euler(0, 0, yaw)
        from geometry_msgs.msg import Quaternion
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    def detection_callback(self, msg):
        for det in msg.detections:
            for hypo in det.results:
                if hypo.hypothesis.class_id == self.search_target:
                    self.get_logger().info(f"Target '{self.search_target}' detected!")
                    self.found = True
                    self.navigator.cancelTask()
                    return

    def search_loop(self):
        if self.found or not self.generated_goals:
            return

        if not self.navigator.isTaskComplete():
            return  # Still moving to goal

        if self.navigator.getResult() == TaskResult.SUCCEEDED:
            last_goal = self.navigator.getGoalPoses()[0].pose.position
            self.visited.add((last_goal.x, last_goal.y))
        else:
            last_goal = self.navigator.getGoalPoses()[0].pose.position
            self.failed_goals.add((last_goal.x, last_goal.y))

        for i, goal in enumerate(self.generated_goals):
            pos = (goal.pose.position.x, goal.pose.position.y)
            if pos not in self.visited and pos not in self.failed_goals:
                self.get_logger().info(f"Navigating to waypoint {i} at {pos}...")
                self.navigator.goToPose(goal)
                return

        self.get_logger().warn("No more waypoints to search. Item not found.")
        self.timer.cancel()
        self.refresh_timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = SearchController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
