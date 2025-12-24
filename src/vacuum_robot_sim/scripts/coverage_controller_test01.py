#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
import math

class CoverageController(Node):
    def __init__(self):
        super().__init__('coverage_controller')

        self.map_msg = None
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_cb, 10
        )

        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )

        self.waypoints = []
        self.current_index = 0

    def map_cb(self, msg):
        if self.map_msg is None:
            self.get_logger().info("Map received.")
        self.map_msg = msg

    def plan_dummy_coverage(self):
        # 临时：手写几个点验证执行链路
        self.waypoints = [
            (0.0, 0.0, 0.0),
            (0.5, 0.0, 0.0),
            (0.5, 0.5, math.pi/2),
        ]
        self.current_index = 0

    def start(self):
        self.plan_dummy_coverage()
        self.send_next_goal()

    def send_next_goal(self):
        if self.current_index >= len(self.waypoints):
            self.get_logger().info("Coverage finished.")
            return

        x, y, yaw = self.waypoints[self.current_index]
        self.current_index += 1

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw/2)
        goal.pose.pose.orientation.w = math.cos(yaw/2)

        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=None
        )
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Reached waypoint.")
            self.send_next_goal()
        else:
            self.get_logger().warn(f"Navigation failed: {status}")

def main():
    rclpy.init()
    node = CoverageController()
    node.start()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
