#!/usr/bin/env python3
"""
Nav2控制节点 (修复版)
功能：
1. 读取当前机器人坐标
2. 通过终端输入坐标，控制机器人运动到目标点
3. 实时显示导航反馈（剩余距离）
4. 正确处理导航成功、取消或失败的状态
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus  # [新增] 引入目标状态枚举
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformListener, Buffer
import math
import threading
from rclpy.qos import qos_profile_sensor_data

class Nav2Controller(Node):
    """
    Nav2控制节点类
    """
    
    def __init__(self):
        """初始化节点"""
        super().__init__('nav2_controller')
        
        # 声明参数
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('global_frame', 'map')
        
        # 获取参数
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.global_frame = self.get_parameter('global_frame').get_parameter_value().string_value
        
        # TF监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 当前位姿信息
        self.current_pose = None
        self.current_odom = None
        self.pose_lock = threading.Lock()
        
        # 订阅里程计话题
        self.odom_subscriber = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            qos_profile_sensor_data
        )
        
        # Nav2导航动作客户端
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        
        # 等待动作服务器
        self.get_logger().info('等待Nav2动作服务器...')
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2动作服务器未响应！请确保Nav2已启动。')
        else:
            self.get_logger().info('Nav2动作服务器已连接')
        
        # 创建定时器定期更新位姿
        self.create_timer(0.1, self.update_pose)
        
        # 启动交互式命令行界面
        self.get_logger().info('=' * 50)
        self.get_logger().info('Nav2控制节点已启动')
        self.get_logger().info('可用命令：')
        self.get_logger().info('  get_pose    - 获取当前坐标')
        self.get_logger().info('  navigate x y [yaw] - 导航到目标点')
        self.get_logger().info('  quit        - 退出')
        self.get_logger().info('=' * 50)
        
        # 启动命令行输入线程
        self.input_thread = threading.Thread(target=self.command_loop, daemon=True)
        self.input_thread.start()
    
    def odom_callback(self, msg):
        with self.pose_lock:
            self.current_odom = msg
    
    def update_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                rclpy.time.Time()
            )
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.global_frame
            pose_stamped.header.stamp = transform.header.stamp
            pose_stamped.pose.position.x = transform.transform.translation.x
            pose_stamped.pose.position.y = transform.transform.translation.y
            pose_stamped.pose.position.z = transform.transform.translation.z
            pose_stamped.pose.orientation = transform.transform.rotation
            with self.pose_lock:
                self.current_pose = pose_stamped
        except Exception:
            with self.pose_lock:
                if self.current_odom is not None:
                    pose_stamped = PoseStamped()
                    pose_stamped.header = self.current_odom.header
                    pose_stamped.pose = self.current_odom.pose.pose
                    self.current_pose = pose_stamped
    
    def get_current_pose(self):
        with self.pose_lock:
            if self.current_pose is None:
                return None
            pose = self.current_pose.pose
            x = pose.position.x
            y = pose.position.y
            qx = pose.orientation.x
            qy = pose.orientation.y
            qz = pose.orientation.z
            qw = pose.orientation.w
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return (x, y, yaw)
    
    def print_current_pose(self):
        pose = self.get_current_pose()
        if pose is None:
            self.get_logger().warn('当前位姿不可用，请等待TF变换...')
            return
        x, y, yaw = pose
        self.get_logger().info(f'当前坐标: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.2f}°')
    
    def navigate_to_pose(self, x, y, yaw=None):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.global_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        
        if yaw is not None:
            qz = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)
            goal_msg.pose.pose.orientation.z = qz
            goal_msg.pose.pose.orientation.w = qw
        else:
            goal_msg.pose.pose.orientation.w = 1.0
        
        self.get_logger().info(f'正在发送导航目标: x={x:.3f}, y={y:.3f}...')
        
        # 异步发送目标
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)
    
    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('导航目标被服务器拒绝！')
            return
        
        self.get_logger().info('目标已接受，开始移动...')
        
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._get_result_callback)
    
    def _get_result_callback(self, future):
        """
        [修改] 根据状态码判断真正的结果
        """
        status = future.result().status
        result = future.result().result
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('>>> 导航成功到达目的地！ <<<')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('>>> 导航被中断（遇到障碍物或无法规划路径） <<<')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('>>> 导航任务被取消 <<<')
        else:
            self.get_logger().warn(f'>>> 导航结束，状态码: {status} <<<')

    def navigation_feedback_callback(self, feedback_msg):
        """
        [修改] 打印剩余距离
        """
        feedback = feedback_msg.feedback
        # distance_remaining 是 feedback 中的标准字段
        self.get_logger().info(f'导航中... 剩余距离: {feedback.distance_remaining:.2f} 米', throttle_duration_sec=2.0)
    
    def command_loop(self):
        while rclpy.ok():
            try:
                command = input('\n> ').strip().split()
                if not command: continue
                cmd = command[0].lower()
                
                if cmd in ['quit', 'exit', 'q']:
                    self.get_logger().info('正在退出...')
                    rclpy.shutdown()
                    break
                elif cmd in ['get_pose', 'pose']:
                    self.print_current_pose()
                elif cmd in ['navigate', 'nav', 'go']:
                    if len(command) < 3:
                        self.get_logger().warn('用法: navigate x y [yaw]')
                        continue
                    try:
                        x = float(command[1])
                        y = float(command[2])
                        yaw = float(command[3]) if len(command) >= 4 else None
                        self.navigate_to_pose(x, y, yaw)
                    except ValueError:
                        self.get_logger().error('坐标必须是数字！')
                else:
                    self.get_logger().warn(f'未知命令: {cmd}')
            except Exception as e:
                pass # 忽略输入错误，防止线程崩溃

def main(args=None):
    rclpy.init(args=args)
    try:
        controller = Nav2Controller()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()