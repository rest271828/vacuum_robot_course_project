#!/usr/bin/env python3
"""
全图覆盖路径规划与执行节点

该节点实现了仿真环境下的全图覆盖并跟随算法（Lawnmower Coverage Algorithm）。
主要功能：
1. 订阅 /map 话题获取占用栅格地图
2. 基于地图生成割草机式覆盖路径（来回扫掠）
3. 通过 Nav2 串行执行航点，实现全图覆盖

算法特点：
- 保守策略：将未知区域(-1)视为障碍物
- 障碍物膨胀：考虑机器人半径和安全边距
- 来回扫掠：交替方向扫掠，提高覆盖效率
- 自动跟随：每个航点的朝向指向下一个航点

@file coverage_controller_test02.py
@brief 全图覆盖路径规划与执行节点
@author vacuum_robot_sim
"""

import math
from typing import List, Tuple

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class CoverageController(Node):
    """
    全图覆盖控制器节点
    
    实现割草机式（Lawnmower）覆盖路径规划算法，通过 Nav2 执行覆盖任务。
    
    订阅话题：
        /map (nav_msgs/OccupancyGrid) - 占用栅格地图
    
    使用服务：
        /navigate_to_pose (nav2_msgs/action/NavigateToPose) - Nav2 导航动作服务
    
    算法流程：
        1. 等待地图数据
        2. 障碍物膨胀（考虑机器人半径和安全边距）
        3. 生成来回扫掠路径
        4. 串行执行航点导航
    """

    def __init__(self):
        """
        初始化覆盖控制器节点
        
        设置地图订阅、Nav2 动作客户端、路径规划参数等。
        """
        super().__init__("coverage_controller")

        # --- 地图订阅 ---
        self.map_msg = None
        # 使用 TRANSIENT_LOCAL 持久化 QoS，确保能接收到已发布的地图
        qos_map = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self._map_cb, qos_map
        )

        # --- Nav2 动作客户端 ---
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # --- 航点执行状态 ---
        self.waypoints: List[Tuple[float, float, float]] = []  # 航点列表 (x, y, yaw)
        self.current_index: int = 0  # 当前执行的航点索引
        self._started: bool = False  # 是否已开始覆盖任务
        self._start_timer = None  # 等待地图的定时器

        # --- 路径规划参数（根据机器人尺寸调整） ---
        self.robot_radius_m = 0.25  # 机器人半径（米）
        self.margin_m = 0.05  # 安全边距（米）
        self.step_m = 0.40  # 扫掠线间距（米），控制覆盖密度
        self.wp_spacing_m = 0.50  # 航点间距（米），沿扫掠线的航点间隔

        self.get_logger().info("CoverageController ready. Waiting for /map ...")

    def _map_cb(self, msg: OccupancyGrid):
        """
        地图回调函数
        
        接收并存储占用栅格地图数据。
        
        @param {OccupancyGrid} msg - 占用栅格地图消息
        """
        if self.map_msg is None:
            self.get_logger().info("Map received.")
        self.map_msg = msg

    # ---------------------------
    # 入口点
    # ---------------------------
    def start(self):
        """
        启动覆盖任务
        
        如果地图已接收，立即开始规划；否则等待地图到达后自动开始。
        """
        if self.map_msg is None:
            self.get_logger().warn("Waiting for /map before starting coverage ...")
            # 轮询直到地图到达
            self._start_timer = self.create_timer(0.5, self._try_start_once)
            return
        self._start_coverage_once()

    def _try_start_once(self):
        """
        尝试启动覆盖任务（定时器回调）
        
        检查地图是否已到达，如果到达则启动覆盖规划。
        """
        if self._started:
            return
        if self.map_msg is None:
            return
        self._start_coverage_once()

    def _start_coverage_once(self):
        """
        执行覆盖路径规划并开始导航
        
        调用路径规划算法生成航点，然后开始串行执行。
        """
        if self._started:
            return
        self._started = True
        if self._start_timer is not None:
            self._start_timer.cancel()

        # 生成割草机式覆盖路径
        self.waypoints = self.plan_coverage_lawnmower(
            robot_radius_m=self.robot_radius_m,
            margin_m=self.margin_m,
            step_m=self.step_m,
            wp_spacing_m=self.wp_spacing_m,
            use_unknown_as_obstacle=True,  # 保守策略：未知区域视为障碍物
        )
        self.current_index = 0

        if not self.waypoints:
            self.get_logger().error("No waypoints planned. Not starting navigation.")
            return

        self.get_logger().info(f"Start executing {len(self.waypoints)} waypoints ...")
        self.send_next_goal()

    # ---------------------------
    # 覆盖路径规划器（割草机算法）
    # ---------------------------
    def plan_coverage_lawnmower(
        self,
        robot_radius_m: float,
        margin_m: float,
        step_m: float,
        wp_spacing_m: float,
        use_unknown_as_obstacle: bool = True,
    ) -> List[Tuple[float, float, float]]:
        """
        生成割草机式覆盖路径
        
        基于占用栅格地图生成来回扫掠的覆盖路径。算法流程：
        1. 障碍物膨胀（考虑机器人半径和安全边距）
        2. 沿 Y 方向按步长扫掠
        3. 每行内沿 X 方向生成航点
        4. 交替方向扫掠（来回模式）
        5. 计算每个航点的朝向（指向下一个航点）
        
        占用值处理（保守策略）：
            - free: value == 0 （自由空间）
            - obstacle: value == 100 （障碍物）
            - unknown: value == -1 （未知区域，视为障碍物）
        
        @param {float} robot_radius_m - 机器人半径（米）
        @param {float} margin_m - 安全边距（米）
        @param {float} step_m - 扫掠线间距（米）
        @param {float} wp_spacing_m - 航点间距（米）
        @param {bool} use_unknown_as_obstacle - 是否将未知区域视为障碍物
        @returns {List[Tuple[float, float, float]]} 航点列表，每个航点为 (x, y, yaw)
        """
        if self.map_msg is None:
            self.get_logger().error("No /map received yet.")
            return []

        m = self.map_msg
        w, h = int(m.info.width), int(m.info.height)  # 地图宽度和高度（像素）
        res = float(m.info.resolution)  # 地图分辨率（米/像素）
        ox = float(m.info.origin.position.x)  # 地图原点 X 坐标
        oy = float(m.info.origin.position.y)  # 地图原点 Y 坐标

        # 将地图数据转换为 numpy 数组
        grid = np.array(m.data, dtype=np.int16).reshape((h, w))

        # 确定自由空间：根据策略处理未知区域
        if use_unknown_as_obstacle:
            free = (grid == 0)  # 仅 0 为自由空间
        else:
            free = (grid == 0) | (grid == -1)  # 0 和 -1 都为自由空间

        # 障碍物膨胀：通过 4 邻域膨胀，考虑机器人半径和安全边距
        inflate_cells = int(np.ceil((robot_radius_m + margin_m) / res))
        occ = ~free

        # 重复膨胀操作，扩大障碍物区域
        for _ in range(inflate_cells):
            occ2 = occ.copy()
            # 4 邻域膨胀：上、下、左、右
            occ2[1:, :] |= occ[:-1, :]  # 向上扩展
            occ2[:-1, :] |= occ[1:, :]  # 向下扩展
            occ2[:, 1:] |= occ[:, :-1]  # 向右扩展
            occ2[:, :-1] |= occ[:, 1:]  # 向左扩展
            occ = occ2

        free2 = ~occ  # 膨胀后的自由空间
        if not np.any(free2):
            self.get_logger().error(
                "No free space after inflation. Reduce radius/margin or check map."
            )
            return []

        # 将米转换为像素
        step_cells = max(1, int(np.round(step_m / res)))  # 扫掠线间距（像素）
        wp_cells = max(1, int(np.round(wp_spacing_m / res)))  # 航点间距（像素）

        waypoints: List[Tuple[float, float, float]] = []
        reverse = False  # 是否反向扫掠（来回模式）

        # 沿 Y 方向从下到上扫掠
        for j in range(0, h, step_cells):
            row = free2[j, :]  # 当前行的自由空间
            idx = np.where(row)[0]  # 自由空间的 X 坐标索引
            if idx.size == 0:
                continue  # 该行无自由空间，跳过

            # 将连续的自由空间分段（处理中间有障碍物的情况）
            splits = np.where(np.diff(idx) > 1)[0]  # 找到不连续点
            segments = np.split(idx, splits + 1)  # 分割成连续段

            line_points: List[Tuple[float, float]] = []
            for seg in segments:
                if seg.size < 3:  # 段太短，跳过
                    continue
                x0, x1 = int(seg[0]), int(seg[-1])  # 段的起始和结束 X 坐标

                # 沿 X 方向生成航点
                xs = list(range(x0, x1 + 1, wp_cells))
                if xs[-1] != x1:  # 确保包含终点
                    xs.append(x1)
                if reverse:  # 反向扫掠
                    xs = xs[::-1]

                # 将像素坐标转换为世界坐标
                for i in xs:
                    x = ox + (i + 0.5) * res  # 像素中心点坐标
                    y = oy + (j + 0.5) * res
                    line_points.append((x, y))

            if len(line_points) < 2:  # 点太少，跳过
                continue

            # 转换为 (x, y, yaw) 格式：yaw 指向下一个航点
            for k in range(len(line_points)):
                x, y = line_points[k]
                if k < len(line_points) - 1:
                    # 有下一个点：朝向指向下一个点
                    nx, ny = line_points[k + 1]
                    yaw = float(np.arctan2(ny - y, nx - x))
                else:
                    # 最后一个点：保持前一个点的方向
                    px, py = line_points[k - 1]
                    yaw = float(np.arctan2(y - py, x - px))
                waypoints.append((x, y, yaw))

            reverse = not reverse  # 切换扫掠方向（来回模式）

        self.get_logger().info(
            f"Planned lawnmower coverage: {len(waypoints)} waypoints "
            f"(inflate_cells={inflate_cells}, step={step_m:.2f}m, spacing={wp_spacing_m:.2f}m)"
        )
        return waypoints

    # ---------------------------
    # 执行器（串行 NavigateToPose）
    # ---------------------------
    def send_next_goal(self):
        """
        发送下一个导航目标
        
        从航点列表中取出下一个航点，通过 Nav2 发送导航目标。
        如果所有航点已完成，则结束覆盖任务。
        """
        if self.current_index >= len(self.waypoints):
            self.get_logger().info("Coverage finished.")
            return

        x, y, yaw = self.waypoints[self.current_index]
        self.current_index += 1

        # 构建导航目标
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)

        # 将平面 yaw 角转换为四元数（仅 z 和 w 分量）
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # 等待 Nav2 动作服务器就绪
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server 'navigate_to_pose' not available.")
            return

        self.get_logger().info(
            f"Sending waypoint {self.current_index}/{len(self.waypoints)}: "
            f"x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}rad"
        )

        # 异步发送目标
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        """
        目标响应回调函数
        
        处理 Nav2 对导航目标的响应。
        
        @param {Future} future - 异步操作的结果 Future
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected.")
            return

        # 获取导航结果
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        """
        导航结果回调函数
        
        处理导航完成后的结果，根据状态决定是否继续下一个航点。
        
        @param {Future} future - 异步操作的结果 Future
        """
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Reached waypoint.")
            self.send_next_goal()  # 成功到达，继续下一个航点
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Navigation aborted. Stopping coverage.")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Navigation canceled. Stopping coverage.")
        else:
            self.get_logger().warn(f"Navigation finished with status={status}. Stopping coverage.")


def main():
    """
    主函数
    
    初始化 ROS2 节点并启动覆盖控制器。
    """
    rclpy.init()
    node = CoverageController()
    node.start()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
