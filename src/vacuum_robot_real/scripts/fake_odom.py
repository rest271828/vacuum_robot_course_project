#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from tf2_ros import TransformBroadcaster
import math


class FakeOdomPublisher(Node):
    def __init__(self):
        super().__init__('fake_odom_publisher')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 创建 odom 发布者
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)

        # TF 广播器
        self.tf_broadcaster = TransformBroadcaster(self)

        # 发布频率 50Hz
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("Fake Odom Publisher started (static odom @ 0,0,0)")

    def timer_callback(self):
        now = self.get_clock().now().to_msg()

        # === 构建 odom 消息 =====
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # 你可以保持位置恒定，也可以用简单的正弦运动，看 RViz 是否更新
        t = self.get_clock().now().nanoseconds / 1e9
        x = 0.0 + 0.2 * math.sin(t)   # 正弦测试
        y = 0.0
        yaw = 0.2 * math.sin(t)

        # 填充 pose
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0

        # yaw -> quaternion
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # 填充简单速度
        odom.twist.twist.linear.x = 0.2 * math.cos(t)
        odom.twist.twist.angular.z = 0.2 * math.cos(t)

        # 发布 odom
        self.odom_pub.publish(odom)

        # === 广播 TF (odom -> base_link) =====
        tfs = TransformStamped()
        tfs.header.stamp = now
        tfs.header.frame_id = 'odom'
        tfs.child_frame_id = 'base_link'
        tfs.transform.translation.x = x
        tfs.transform.translation.y = y
        tfs.transform.translation.z = 0.0
        tfs.transform.rotation.z = qz
        tfs.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tfs)


def main(args=None):
    rclpy.init(args=args)
    node = FakeOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
