#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

import fields2cover as f2c


class Fields2CoverPlanner(Node):
    """
    订阅 SLAM 的 /map，用 Fields2Cover 做全覆盖规划，
    并把路径点依次发给 Nav2 的 navigate_to_pose。
    """

    def __init__(self):
        super().__init__('fields2cover_planner')

        # 参数：全局坐标系（要和 Nav2 / AMCL 一致，默认 map）
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('goal_step', 20)  # 每隔多少个点发一个导航目标

        self.global_frame = self.get_parameter('global_frame').get_parameter_value().string_value
        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.goal_step = self.get_parameter('goal_step').get_parameter_value().integer_value

        # 订阅 /map
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            10
        )

        # 用于在 RViz 中查看规划路径
        self.path_pub = self.create_publisher(Path, '/coverage_path', 10)

        # Nav2 动作客户端
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Fields2Cover 里的机器人模型（宽度根据你真实机器人改）
        self.robot = f2c.Robot(0.5, 0.5)
        self.robot.setCruiseVel(0.5)

        self.already_planned = False

        self.get_logger().info('Fields2CoverPlanner 启动，等待 /map ...')

    # =================== map 回调 ===================
    def map_callback(self, msg: OccupancyGrid):
        # 只规划一次；如果你希望重规划，可以改成条件触发
        if self.already_planned:
            return

        self.get_logger().info('收到 /map，开始构造场地并进行覆盖规划 ...')

        cells = self.occupancy_grid_to_cells(msg)
        path = self.plan_coverage(cells)

        ros_path = self.f2c_path_to_ros_path(path, msg.header.frame_id)
        self.path_pub.publish(ros_path)
        self.get_logger().info(f'覆盖路径已发布，共 {len(ros_path.poses)} 个点 (/coverage_path)')

        # 等 Nav2 服务器准备好
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 navigate_to_pose 动作服务器不可用！')
            return

        self.get_logger().info('Nav2 已连接，开始沿覆盖路径依次发送目标点 ...')
        self.send_goals_along_path(ros_path, step=self.goal_step)

        self.already_planned = True

    # =================== OccupancyGrid → Cells ===================
    def occupancy_grid_to_cells(self, map_msg: OccupancyGrid) -> f2c.Cells:
        """
        简单版本：用整张 map 的外接矩形做场地。
        先跑通流程，之后你可以把障碍物也转成 polygon。
        """
        res = map_msg.info.resolution
        w = map_msg.info.width
        h = map_msg.info.height
        origin = map_msg.info.origin.position

        x_min = origin.x
        y_min = origin.y
        x_max = origin.x + w * res
        y_max = origin.y + h * res

        ring = f2c.LinearRing()
        ring.addPoint(f2c.Point(x_min, y_min))
        ring.addPoint(f2c.Point(x_max, y_min))
        ring.addPoint(f2c.Point(x_max, y_max))
        ring.addPoint(f2c.Point(x_min, y_max))
        ring.addPoint(f2c.Point(x_min, y_min))  # 闭合

        cell = f2c.Cell()
        cell.addRing(ring)
        cells = f2c.Cells(cell)

        self.get_logger().info(
            f'场地外接矩形: '
            f'({x_min:.2f},{y_min:.2f}) -> ({x_max:.2f},{y_max:.2f})'
        )
        return cells

    # =================== Fields2Cover 覆盖规划 ===================
    def plan_coverage(self, cells: f2c.Cells):
        hg = f2c.HG_Const_gen()
        no_hl = hg.generateHeadlands(cells, 3.0 * self.robot.getWidth())

        sg = f2c.SG_BruteForce()
        swaths = sg.generateSwaths(
            math.pi,  # 参考角度：可以根据 map 形状智能调整
            self.robot.getCovWidth(),
            no_hl.getGeometry(0)
        )
        self.get_logger().info(f'生成条带 swaths: {swaths.size()}')

        rp = f2c.RP_Boustrophedon()
        route = rp.genSortedSwaths(swaths)

        pp = f2c.PP_PathPlanning()
        dubins = f2c.PP_DubinsCurves()
        path = pp.planPath(self.robot, route, dubins)
        self.get_logger().info(f'规划出的路径状态点数量: {path.size()}')

        return path

    # =================== F2C Path → nav_msgs/Path ===================
    def f2c_path_to_ros_path(self, path: f2c.PP_Path, frame_id: str) -> Path:
        ros_path = Path()
        ros_path.header.frame_id = frame_id
        ros_path.header.stamp = self.get_clock().now().to_msg()

        for i in range(path.size()):
            state = path.getState(i)
            p = state.point

            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.header.stamp = ros_path.header.stamp
            pose.pose.position.x = p.getX()
            pose.pose.position.y = p.getY()
            pose.pose.position.z = 0.0

            # 这里暂时不给 yaw，Nav2 会自己调整；你后面也可以从 state 中取方向转成四元数
            pose.pose.orientation.w = 1.0

            ros_path.poses.append(pose)

        return ros_path

    # =================== 按路径给 Nav2 发目标 ===================
    def send_goals_along_path(self, path: Path, step: int = 20):
        """
        每隔 step 个路径点发一个 NavigateToPose 目标。
        简单串行版本：等待一个 goal 结束后再发下一个。
        """
        total = len(path.poses)
        if total == 0:
            self.get_logger().warn('路径为空，不能发送目标')
            return

        for idx in range(0, total, step):
            pose = path.poses[idx]

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = pose
            goal_msg.pose.header.frame_id = self.global_frame
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

            self.get_logger().info(
                f'发送导航目标 [{idx}/{total}]: '
                f'({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})'
            )

            send_future = self.nav_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_future)
            goal_handle = send_future.result()

            if not goal_handle.accepted:
                self.get_logger().warn(f'目标 {idx} 被 Nav2 拒绝，跳过')
                continue

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result()
            self.get_logger().info(f'目标 {idx} 完成，状态：{result.status}')

        self.get_logger().info('全部覆盖目标发送完成')

def main(args=None):
    rclpy.init(args=args)
    node = Fields2CoverPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
