#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import serial
import struct
import math


class WheelSerialBridge(Node):
    def __init__(self):
        super().__init__('robot_serial_driver')

        # 参数
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('wheel_base', 0.20)  # 轮距 L，自己改
        self.declare_parameter('pose_scale', 1000.0)  # uint32位姿转换系数，默认1000（表示千分之一度）

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.pose_scale = self.get_parameter('pose_scale').get_parameter_value().double_value

        # 串口初始化
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1  # 100ms超时，确保能读取完整帧（12字节在9600波特率下需要约12.5ms）
            )
            self.ser.reset_input_buffer()  # 清空输入缓冲区
            self.ser.reset_output_buffer()  # 清空输出缓冲区
            self.get_logger().info(f'串口打开成功: {port}, {baudrate}')
        except Exception as e:
            self.ser = None
            self.get_logger().error(f'串口打开失败: {e}')

        # 订阅 cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

        # 发布 odom
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # 保存最近一次 cmd_vel
        self.last_cmd_vel = Twist()
        self.last_cmd_time = self.get_clock().now()

        # 串口接收缓冲区
        self.rx_buffer = bytearray()
        self.max_buffer_size = 1024  # 最大缓冲区大小，防止溢出
        
        # 调试统计
        self.frame_count = 0
        self.last_frame_time = None
        self.last_stats_frame_count = 0

        # 定时器：周期性发期望轮速 + 读串口（每2秒发送一次）
        self.timer = self.create_timer(0.5, self.timer_callback)  # 2Hz，每0.5秒发送一次速度指令
        
        # 统计定时器：每5秒打印一次接收统计
        self.stats_timer = self.create_timer(5.0, self.stats_callback)

    # -------------------------
    # cmd_vel 回调：只存，不立即发
    # -------------------------
    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_vel = msg
        self.last_cmd_time = self.get_clock().now()

    # -------------------------
    # 定时器：发期望轮速 + 读串口
    # -------------------------
    def timer_callback(self):
        # 1. 发送期望轮速到下位机
        self.send_desired_wheel_speed()

        # 2. 读取并解析下位机的里程计帧
        self.read_odom_from_mcu()
    
    # -------------------------
    # 统计回调：定期打印接收统计
    # -------------------------
    def stats_callback(self):
        if self.ser is None or not self.ser.is_open:
            return
        
        frames_received = self.frame_count - self.last_stats_frame_count
        buffer_size = len(self.rx_buffer)
        in_waiting = self.ser.in_waiting
        
        if frames_received > 0:
            self.get_logger().info(
                f'接收统计: 5秒内收到 {frames_received} 帧, '
                f'缓冲区: {buffer_size} 字节, 串口待读: {in_waiting} 字节'
            )
        elif buffer_size > 0 or in_waiting > 0:
            # 打印缓冲区内容用于调试
            if buffer_size > 0:
                buffer_hex = ' '.join([f'{b:02X}' for b in self.rx_buffer[:20]])
                self.get_logger().warn(
                    f'接收异常: 5秒内未收到完整帧, '
                    f'缓冲区: {buffer_size} 字节, 串口待读: {in_waiting} 字节, '
                    f'缓冲区内容: {buffer_hex}'
                )
            else:
                self.get_logger().warn(
                    f'接收异常: 5秒内未收到完整帧, '
                    f'缓冲区: {buffer_size} 字节, 串口待读: {in_waiting} 字节'
                )
        
        self.last_stats_frame_count = self.frame_count

    # -------------------------
    # 上位机 → 下位机：发送左右轮期望速度
    # 帧格式：
    #   0: 0xAA (帧头)
    #   1-2: left_speed (float16, little-endian)
    #   3-4: right_speed (float16, little-endian)
    #   5: 0x55 (帧尾)
    # -------------------------
    def send_desired_wheel_speed(self):
        if self.ser is None or not self.ser.is_open:
            return

        v = self.last_cmd_vel.linear.x
        w = self.last_cmd_vel.angular.z

        L = self.wheel_base

        # 差速解算：假定单位 m/s
        v_left = v - w * L / 2.0
        v_right = v + w * L / 2.0

        # 实际速度是计算速度的1/100
        v_left = v_left / 100.0
        v_right = v_right / 100.0

        # 打包成 float16
        try:
            frame = bytearray()
            frame.append(0xAA)  # 帧头
            frame += struct.pack('<e', float(v_left))
            frame += struct.pack('<e', float(v_right))
            frame.append(0x55)  # 帧尾

            self.ser.write(frame)
            # 可按需要注释掉日志
            # self.get_logger().info(
            #     f'发送轮速: v_l={v_left:.3f}, v_r={v_right:.3f}'
            # )
        except Exception as e:
            self.get_logger().error(f'串口发送失败: {e}')

    # -------------------------
    # 下位机 → 上位机：接收里程计数据
    # 帧格式：
    #   [0]      0x55        帧头字节1
    #   [1]      0x00        帧头字节2
    #   [2..5]   x (float32, 小端)
    #   [6..9]   y (float32, 小端)
    #   [10..13] 位姿 (uint32, 小端)
    #   [14]     0x00        帧尾字节1
    #   [15]     0xAA        帧尾字节2
    # 总共 16 字节，无校验
    # -------------------------
    def read_odom_from_mcu(self):
        if self.ser is None or not self.ser.is_open:
            return

        # 读取串口数据
        try:
            # 检查是否有数据可读
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting)  # 读取所有可用数据
                if data:
                    self.rx_buffer.extend(data)
                    # 防止缓冲区溢出
                    if len(self.rx_buffer) > self.max_buffer_size:
                        self.get_logger().warn(f'接收缓冲区溢出（{len(self.rx_buffer)}字节），清空缓冲区')
                        self.rx_buffer.clear()
        except Exception as e:
            self.get_logger().error(f'串口读取失败: {e}')
            return

        FRAME_LEN = 16  # 帧头(2) + x(4) + y(4) + 位姿(4) + 帧尾(2) = 16字节
        FRAME_HEAD_BYTE1 = 0x55  # 帧头第一个字节
        FRAME_HEAD_BYTE2 = 0x00  # 帧头第二个字节
        FRAME_TAIL_BYTE1 = 0x00  # 帧尾第一个字节
        FRAME_TAIL_BYTE2 = 0xAA  # 帧尾第二个字节

        # 尝试从缓冲区中解析完整帧
        frames_found = 0
        max_frames_per_call = 10  # 限制每次最多处理10帧，避免阻塞
        
        # 如果缓冲区有数据但不够完整帧，打印调试信息
        if 0 < len(self.rx_buffer) < FRAME_LEN:
            # 打印缓冲区内容（十六进制）
            buffer_hex = ' '.join([f'{b:02X}' for b in self.rx_buffer])
            self.get_logger().info(
                f'缓冲区不完整: {len(self.rx_buffer)}/{FRAME_LEN} 字节, '
                f'数据: {buffer_hex}'
            )
        
        while len(self.rx_buffer) >= FRAME_LEN and frames_found < max_frames_per_call:
            # 寻找帧头 0x55 0x00
            head_index = -1
            for i in range(len(self.rx_buffer) - FRAME_LEN + 1):
                if (self.rx_buffer[i] == FRAME_HEAD_BYTE1 and 
                    i + 1 < len(self.rx_buffer) and 
                    self.rx_buffer[i + 1] == FRAME_HEAD_BYTE2):
                    head_index = i
                    break
            
            # 如果没找到帧头，清空缓冲区（可能数据损坏）
            if head_index == -1:
                # 打印调试信息
                buffer_hex = ' '.join([f'{b:02X}' for b in self.rx_buffer[:20]])  # 只打印前20字节
                self.get_logger().warn(
                    f'未找到帧头 0x55 0x00，缓冲区: {len(self.rx_buffer)} 字节, '
                    f'前20字节: {buffer_hex}'
                )
                if len(self.rx_buffer) > FRAME_LEN:
                    # 保留最后几个字节，可能是不完整的帧
                    self.rx_buffer = self.rx_buffer[-(FRAME_LEN-1):]
                else:
                    self.rx_buffer.clear()
                break
            
            # 丢弃帧头之前的无效数据
            if head_index > 0:
                self.rx_buffer = self.rx_buffer[head_index:]
            
            # 检查是否有足够的数据（包括帧尾）
            if len(self.rx_buffer) < FRAME_LEN:
                break  # 数据不完整，等待下次读取
            
            # 检查帧尾 0x00 0xAA
            if (self.rx_buffer[FRAME_LEN - 2] != FRAME_TAIL_BYTE1 or 
                self.rx_buffer[FRAME_LEN - 1] != FRAME_TAIL_BYTE2):
                # 帧尾不匹配，打印调试信息
                frame_hex = ' '.join([f'{b:02X}' for b in self.rx_buffer[:FRAME_LEN]])
                self.get_logger().warn(
                    f'帧尾不匹配: 期望 0x{FRAME_TAIL_BYTE1:02X} 0x{FRAME_TAIL_BYTE2:02X}, '
                    f'实际 0x{self.rx_buffer[FRAME_LEN-2]:02X} 0x{self.rx_buffer[FRAME_LEN-1]:02X}, '
                    f'帧数据: {frame_hex}'
                )
                # 帧尾不匹配，丢弃第一个字节，继续寻找
                self.rx_buffer.pop(0)
                continue
            
            # 找到完整帧，提取并解析
            frame = self.rx_buffer[:FRAME_LEN]
            self.rx_buffer = self.rx_buffer[FRAME_LEN:]
            frames_found += 1

            # 解析数据：2个float32 (x, y) + 1个uint32 (位姿)
            try:
                # frame[2:14] 是 12 字节数据区（跳过2字节帧头）
                payload = frame[2:14]

                # 解析 x, y (float32)
                x_bytes = payload[0:4]
                y_bytes = payload[4:8]
                
                # 解析位姿 (uint32)
                pose_uint32 = payload[8:12]

                x = struct.unpack('<f', x_bytes)[0]  # float32
                y = struct.unpack('<f', y_bytes)[0]  # float32
                pose_raw = struct.unpack('<I', pose_uint32)[0]  # uint32
                
                # 将uint32位姿转换为yaw角度（弧度）
                # 使用参数配置的转换系数
                # 默认假设uint32表示千分之一度，需要转换为弧度
                # 例如：pose_scale=1000 表示 uint32值/1000 = 度，然后转换为弧度
                # 如果下位机使用其他格式，可以通过pose_scale参数调整
                yaw = math.radians(pose_raw / self.pose_scale)

                # 更新统计
                self.frame_count += 1
                current_time = self.get_clock().now()
                if self.last_frame_time is not None:
                    dt = (current_time - self.last_frame_time).nanoseconds / 1e9
                    if dt > 2.0:  # 如果超过2秒没收到数据，打印警告
                        self.get_logger().warn(f'收到数据帧（间隔 {dt:.2f}秒），x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.3f}°')
                self.last_frame_time = current_time

                # 发布里程计
                self.publish_odom(x, y, yaw)

            except Exception as e:
                self.get_logger().error(f'解析里程计帧失败: {e}, 帧数据: {frame.hex()}')
                continue

    # -------------------------
    # 发布 Odometry
    # -------------------------
    def publish_odom(self, x, y, yaw):
        """
        发布里程计数据
        @param x: x坐标 (float16, 米)
        @param y: y坐标 (float16, 米)
        @param yaw: 位姿角度 (弧度)
        """
        odom_msg = Odometry()

        now = self.get_clock().now().to_msg()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # 位置
        odom_msg.pose.pose.position.x = float(x)
        odom_msg.pose.pose.position.y = float(y)
        odom_msg.pose.pose.position.z = 0.0

        # yaw -> 四元数（绕z轴旋转）
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        # 由于没有速度数据，将twist设为0
        # 如果需要速度信息，可以通过位置差分计算
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0

        # 设置协方差矩阵（对定位和导航很重要）
        # pose 协方差：位置和姿态的不确定性
        # 36个元素，6x6矩阵（x, y, z, roll, pitch, yaw）
        odom_msg.pose.covariance[0] = 0.01  # x 方差
        odom_msg.pose.covariance[7] = 0.01  # y 方差
        odom_msg.pose.covariance[35] = 0.03  # yaw 方差
        # 其他元素保持默认值 0

        # twist 协方差：速度的不确定性
        # 36个元素，6x6矩阵（vx, vy, vz, vroll, vpitch, vyaw）
        odom_msg.twist.covariance[0] = 0.01  # vx 方差
        odom_msg.twist.covariance[35] = 0.03  # vyaw 方差
        # 其他元素保持默认值 0

        self.odom_pub.publish(odom_msg)

        # 可选：调试打印
        # self.get_logger().info(
        #     f'ODOM x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}, '
        #     f'vl={v_left:.3f}, vr={v_right:.3f}'
        # )


def main(args=None):
    rclpy.init(args=args)
    node = WheelSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser is not None and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
