import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from vision_msgs.msg import Detection2DArray
import math
import tf_transformations
import numpy as np
import time

class SearchController(Node):
    def __init__(self):
        super().__init__('search_controller')
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        # Subscriptions
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.costmap_callback, 10)
        self.create_subscription(Detection2DArray, '/detections', self.detection_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)

        # Parameters
        self.search_target_id = 39
        self.hypo_threshold = 0.7
        self.history_radius = 0.15
        self.goal_attempt_limit = 20
        self.max_retries = 3
        self.reach_tolerance = 0.1
        self.view_fov = math.radians(60.0)
        self.view_range = 0.3

        # State
        self.visited = set()
        self.failed_goals = set()
        self.generated_goals = []
        self.retry_counts = {}
        self.current_goal = None
        self.found = False

        # Map / robot
        self.latest_costmap = None
        self.robot_pose_stamped = None
        self.map_min_x = self.map_min_y = None
        self.map_max_x = self.map_max_y = None

        # Timers
        self.create_timer(0.5, self.search_loop)
        self.create_timer(15.0, self.refresh_map)

    def pose_callback(self, msg):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.robot_pose_stamped = ps

    def costmap_callback(self, msg):
        self.latest_costmap = msg
        origin = msg.info.origin.position
        res = msg.info.resolution
        w, h = msg.info.width, msg.info.height
        self.map_min_x = origin.x
        self.map_min_y = origin.y
        self.map_max_x = origin.x + w * res
        self.map_max_y = origin.y + h * res
        if not self.generated_goals and self.robot_pose_stamped:
            self.generated_goals = self.generate_waypoints(msg)

    def refresh_map(self):
        pending = [g for g in self.generated_goals if (g.pose.position.x, g.pose.position.y) not in self.visited | self.failed_goals]
        if not pending and self.latest_costmap and not self.found and self.robot_pose_stamped:
            self.generated_goals = self.generate_waypoints(self.latest_costmap)
            self.get_logger().info('Waypoints refreshed')

    def generate_waypoints(self, msg):
        span = self.view_range * math.tan(self.view_fov / 2)
        x_min, y_min = self.map_min_x, self.map_min_y
        x_max, y_max = self.map_max_x, self.map_max_y
        cols = max(1, int(math.ceil((x_max-x_min)/(2*span))))
        rows = max(1, int(math.ceil((y_max-y_min)/(2*span))))

        data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        res = msg.info.resolution
        waypoints = []

        for i in range(cols):
            for j in range(rows):
                x = x_min + (i+0.5)*2*span
                y = y_min + (j+0.5)*2*span
                if not (x_min<=x<=x_max and y_min<=y<=y_max): continue
                xi = int((x-x_min)/res)
                yi = int((y-y_min)/res)
                if xi<0 or xi>=msg.info.width or yi<0 or yi>=msg.info.height: continue
                if data[yi, xi]!=0: continue
                if any(math.hypot(vx-x, vy-y)<self.history_radius for vx,vy in self.visited|self.failed_goals): continue
                pose = PoseStamped()
                pose.header.frame_id='map'
                pose.header.stamp=self.get_clock().now().to_msg()
                pose.pose.position.x = x
                pose.pose.position.y = y
                q = tf_transformations.quaternion_from_euler(0,0,0)
                from geometry_msgs.msg import Quaternion
                pose.pose.orientation = Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])
                try:
                    path = self.navigator.getPath(self.robot_pose_stamped, pose)
                except Exception:
                    path = None
                if not path:
                    self.failed_goals.add((x,y))
                    continue
                waypoints.append(pose)
                self.retry_counts[(x,y)] = 0
                if len(waypoints)>=self.goal_attempt_limit:
                    break
            if len(waypoints)>=self.goal_attempt_limit:
                break

        if self.robot_pose_stamped:
            rx = self.robot_pose_stamped.pose.position.x
            ry = self.robot_pose_stamped.pose.position.y
            waypoints.sort(key=lambda p: math.hypot(p.pose.position.x-rx, p.pose.position.y-ry))
        self.get_logger().info(f'Generated {len(waypoints)} waypoints')
        return waypoints

    def detection_callback(self, msg):
        for det in msg.detections:
            for hypo in det.results:
                try:
                    cid = int(float(hypo.hypothesis.class_id))
                except:
                    continue
                scr = float(hypo.hypothesis.score)
                if cid==self.search_target_id and scr>=self.hypo_threshold:
                    self.get_logger().info(f'Found target {cid}')
                    self.found=True
                    if self.current_goal:
                        self.navigator.cancelTask()
                    return

    def search_loop(self):
        if self.found or not self.generated_goals or not self.robot_pose_stamped:
            return
        if not self.navigator.isTaskComplete():
            return

        # Evaluate last goal
        if self.current_goal:
            gx, gy = self.current_goal.pose.position.x, self.current_goal.pose.position.y
            rx, ry = self.robot_pose_stamped.pose.position.x, self.robot_pose_stamped.pose.position.y
            dist = math.hypot(gx-rx, gy-ry)
            pos = (gx,gy)
            if dist<=self.reach_tolerance and self.navigator.getResult()==TaskResult.SUCCEEDED:
                self.visited.add(pos)
                self.get_logger().info(f'Reached {pos}, scanning')
                spins = int(2*math.pi/self.view_fov)
                for _ in range(spins):
                    self.navigator.spin(self.view_fov)
                    time.sleep(0.5)
                self.current_goal=None
            else:
                retries = self.retry_counts.get(pos,0)
                if retries<self.max_retries:
                    self.retry_counts[pos]=retries+1
                    self.get_logger().warn(f'Retry {self.retry_counts[pos]} for {pos}')
                    self.navigator.goToPose(self.current_goal)
                    return
                else:
                    self.failed_goals.add(pos)
                    self.generated_goals=[g for g in self.generated_goals if (g.pose.position.x,g.pose.position.y)!=pos]
                    self.current_goal=None

        # Resort remaining
        rx=self.robot_pose_stamped.pose.position.x; ry=self.robot_pose_stamped.pose.position.y
        self.generated_goals.sort(key=lambda p:math.hypot(p.pose.position.x-rx,p.pose.position.y-ry))

        # Next goal
        for g in self.generated_goals:
            pos=(g.pose.position.x,g.pose.position.y)
            if pos not in self.visited and pos not in self.failed_goals:
                self.current_goal=g
                self.get_logger().info(f'Navigating to {pos}')
                self.navigator.goToPose(g)
                break
        else:
            self.get_logger().warn('Search complete')


def main():
    rclpy.init()
    node=SearchController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()
