#!/usr/bin/env python3
"""
Nav2控制节点
功能：
1. 读取当前机器人坐标
2. 通过终端输入坐标，控制机器人运动到目标点
3. 读取当前机器人姿态（方向）
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
import math
import threading
import sys
from rclpy.qos import qos_profile_sensor_data


class Nav2Controller(Node):
    """
    Nav2控制节点类
    提供坐标读取、导航控制和姿态查询功能
    """
    
    def __init__(self):
        """初始化节点"""
        super().__init__('nav2_controller')
        
        # 声明参数
        #self.declare_parameter('use_sim_time', True)
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
        self.get_logger().info('Nav2控制节点已启动')
        self.get_logger().info('=' * 50)
        self.get_logger().info('可用命令：')
        self.get_logger().info('  get_pose    - 获取当前坐标和姿态')
        self.get_logger().info('  navigate x y [yaw] - 导航到指定坐标 (yaw可选，单位：弧度)')
        self.get_logger().info('  quit        - 退出程序')
        self.get_logger().info('=' * 50)
        
        # 启动命令行输入线程
        self.input_thread = threading.Thread(target=self.command_loop, daemon=True)
        self.input_thread.start()
    
    def odom_callback(self, msg):
        """
        里程计回调函数
        
        @param msg: Odometry消息
        """
        with self.pose_lock:
            self.current_odom = msg
    
    def update_pose(self):
        """
        更新当前位姿（从TF树获取）
        """
        try:
            # 尝试从TF获取base_link到map的变换
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                rclpy.time.Time()
            )
            
            # 创建PoseStamped
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.global_frame
            pose_stamped.header.stamp = transform.header.stamp
            pose_stamped.pose.position.x = transform.transform.translation.x
            pose_stamped.pose.position.y = transform.transform.translation.y
            pose_stamped.pose.position.z = transform.transform.translation.z
            pose_stamped.pose.orientation = transform.transform.rotation
            
            with self.pose_lock:
                self.current_pose = pose_stamped
                
        except Exception as e:
            # TF变换可能暂时不可用，使用里程计数据作为备选
            with self.pose_lock:
                if self.current_odom is not None:
                    # 从里程计创建位姿（在odom坐标系中）
                    pose_stamped = PoseStamped()
                    pose_stamped.header = self.current_odom.header
                    pose_stamped.pose = self.current_odom.pose.pose
                    self.current_pose = pose_stamped
    
    def get_current_pose(self):
        """
        获取当前位姿
        
        @return: (x, y, yaw) 或 None
        """
        with self.pose_lock:
            if self.current_pose is None:
                return None
            
            pose = self.current_pose.pose
            x = pose.position.x
            y = pose.position.y
            
            # 从四元数计算yaw角
            qx = pose.orientation.x
            qy = pose.orientation.y
            qz = pose.orientation.z
            qw = pose.orientation.w
            
            # 转换为欧拉角（yaw）
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            return (x, y, yaw)
    
    def print_current_pose(self):
        """打印当前位姿信息"""
        pose = self.get_current_pose()
        if pose is None:
            self.get_logger().warn('当前位姿不可用，请等待TF变换...')
            return
        
        x, y, yaw = pose
        yaw_deg = math.degrees(yaw)
        
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'当前坐标: x = {x:.3f} m, y = {y:.3f} m')
        self.get_logger().info(f'当前姿态: yaw = {yaw:.3f} rad ({yaw_deg:.2f}°)')
        self.get_logger().info('=' * 50)
    
    def navigate_to_pose(self, x, y, yaw=None):
        """
        导航到指定位姿（非阻塞版本）
        
        @param x: 目标x坐标
        @param y: 目标y坐标
        @param yaw: 目标yaw角（可选，单位：弧度）
        """
        # 创建目标位姿
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.global_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        
        # 设置方向
        if yaw is not None:
            # 从yaw角创建四元数
            qz = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)
            goal_msg.pose.pose.orientation.z = qz
            goal_msg.pose.pose.orientation.w = qw
            goal_msg.pose.pose.orientation.x = 0.0
            goal_msg.pose.pose.orientation.y = 0.0
        else:
            # 不指定方向，使用当前方向
            current_pose = self.get_current_pose()
            if current_pose is not None:
                _, _, current_yaw = current_pose
                qz = math.sin(current_yaw / 2.0)
                qw = math.cos(current_yaw / 2.0)
                goal_msg.pose.pose.orientation.z = qz
                goal_msg.pose.pose.orientation.w = qw
                goal_msg.pose.pose.orientation.x = 0.0
                goal_msg.pose.pose.orientation.y = 0.0
            else:
                # 默认方向（0度）
                goal_msg.pose.pose.orientation.w = 1.0
        
        # 发送目标
        yaw_str = f'{math.degrees(yaw):.2f}°' if yaw is not None else 'current'
        self.get_logger().info(f'发送导航目标: x={x:.3f}, y={y:.3f}, yaw={yaw_str}')
        
        # 异步发送目标，不阻塞
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback
        )
        
        # 设置回调处理结果
        send_goal_future.add_done_callback(self._goal_response_callback)
    
    def _goal_response_callback(self, future):
        """
        目标响应回调函数
        
        @param future: send_goal_async返回的Future对象
        """
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('导航目标被拒绝')
            return
        
        self.get_logger().info('导航目标已接受，机器人开始移动...')
        
        # 获取结果
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._get_result_callback)
    
    def _get_result_callback(self, future):
        """
        获取结果回调函数
        
        @param future: get_result_async返回的Future对象
        """
        result = future.result().result
        if result:
            self.get_logger().info('导航完成！')
        else:
            self.get_logger().warn('导航可能未完成')
    
    def navigation_feedback_callback(self, feedback_msg):
        """
        导航反馈回调函数
        
        @param feedback_msg: NavigateToPose.Feedback消息
        """
        # 可以在这里处理导航过程中的反馈信息
        pass
    
    def command_loop(self):
        """
        命令行输入循环（在独立线程中运行）
        """
        while rclpy.ok():
            try:
                # 从标准输入读取命令
                command = input('\n> ').strip().split()
                
                if not command:
                    continue
                
                cmd = command[0].lower()
                
                if cmd == 'quit' or cmd == 'exit' or cmd == 'q':
                    self.get_logger().info('正在退出...')
                    rclpy.shutdown()
                    break
                
                elif cmd == 'get_pose' or cmd == 'pose':
                    self.print_current_pose()
                
                elif cmd == 'navigate' or cmd == 'nav' or cmd == 'go':
                    if len(command) < 3:
                        self.get_logger().error('用法: navigate x y [yaw]')
                        self.get_logger().info('示例: navigate 2.0 1.5 1.57')
                        continue
                    
                    try:
                        x = float(command[1])
                        y = float(command[2])
                        yaw = None
                        
                        if len(command) >= 4:
                            yaw = float(command[3])
                        
                        # 直接调用导航（非阻塞）
                        self.navigate_to_pose(x, y, yaw)
                        
                    except ValueError as e:
                        self.get_logger().error(f'无效的坐标值: {e}')
                
                elif cmd == 'help' or cmd == 'h':
                    self.get_logger().info('=' * 50)
                    self.get_logger().info('可用命令：')
                    self.get_logger().info('  get_pose              - 获取当前坐标和姿态')
                    self.get_logger().info('  navigate x y [yaw]    - 导航到指定坐标')
                    self.get_logger().info('                        - x, y: 坐标（米）')
                    self.get_logger().info('                        - yaw: 可选，方向角（弧度）')
                    self.get_logger().info('  quit                  - 退出程序')
                    self.get_logger().info('=' * 50)
                
                else:
                    self.get_logger().warn(f'未知命令: {cmd}。输入 "help" 查看帮助')
            
            except EOFError:
                # 标准输入关闭（例如Ctrl+D）
                break
            except Exception as e:
                self.get_logger().error(f'命令处理错误: {e}')
    


def main(args=None):
    """
    主函数
    """
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

