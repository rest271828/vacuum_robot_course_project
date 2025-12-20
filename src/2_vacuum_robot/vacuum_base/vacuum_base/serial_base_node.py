#!/usr/bin/env python3
from __future__ import annotations

import math
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

from .protocol import VacuumSerialProtocol, WheelFeedback


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class VacuumSerialBase(Node):
    """
    Minimal serial base:
    - Sub /cmd_vel
    - TX wheel cmd (mm/s) at send_rate_hz
    - RX wheel feedback + yaw
    - Integrate x,y in odom frame
    - Publish /odom and TF odom->base_link
    """

    def __init__(self) -> None:
        super().__init__("vacuum_serial_base")

        # ---- params ----
        self.declare_parameter("port", "/dev/ttyUSB1")
        self.declare_parameter("baudrate", 9600)
        self.declare_parameter("wheel_base", 0.20)           # meters
        self.declare_parameter("send_rate_hz", 50.0)
        self.declare_parameter("read_rate_hz", 200.0)
        self.declare_parameter("odom_rate_hz", 50.0)
        self.declare_parameter("cmd_timeout_sec", 0.5)

        # If your MCU expects different cmd unit, set this (default mm/s)
        self.declare_parameter("cmd_scale_mps_to_unit", 1000.0)  # m/s -> mm/s

        self.port = str(self.get_parameter("port").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.L = float(self.get_parameter("wheel_base").value)
        self.send_rate_hz = float(self.get_parameter("send_rate_hz").value)
        self.read_rate_hz = float(self.get_parameter("read_rate_hz").value)
        self.odom_rate_hz = float(self.get_parameter("odom_rate_hz").value)
        self.cmd_timeout_sec = float(self.get_parameter("cmd_timeout_sec").value)
        self.cmd_scale = float(self.get_parameter("cmd_scale_mps_to_unit").value)

        # ---- ros io ----
        self.cmd_sub = self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_pub = TransformBroadcaster(self)

        # ---- serial ----
        self.ser: serial.Serial | None = None
        self.rx_buf = bytearray()
        self.open_serial()

        # ---- state ----
        self.last_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()

        # integrated odom
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_integ_time = self.get_clock().now()
        self.latest_fb: WheelFeedback | None = None

        # ---- timers ----
        self.send_timer = self.create_timer(1.0 / self.send_rate_hz, self.send_cmd_timer)
        self.read_timer = self.create_timer(1.0 / self.read_rate_hz, self.read_serial_timer)
        self.odom_timer = self.create_timer(1.0 / self.odom_rate_hz, self.publish_odom_timer)

        self.get_logger().info(
            f"vacuum_serial_base started. port={self.port} baud={self.baudrate} "
            f"wheel_base={self.L}m send={self.send_rate_hz}Hz read={self.read_rate_hz}Hz odom={self.odom_rate_hz}Hz"
        )

    def open_serial(self) -> None:
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.0,  # non-blocking
            )
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.get_logger().info("Serial opened successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial: {e}")
            self.ser = None

    def on_cmd(self, msg: Twist) -> None:
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    # ---------------- TX ----------------
    def send_cmd_timer(self) -> None:
        if self.ser is None or not self.ser.is_open:
            return

        now = self.get_clock().now()
        age = (now - self.last_cmd_time).nanoseconds / 1e9
        if age > self.cmd_timeout_sec:
            v = 0.0
            w = 0.0
        else:
            v = float(self.last_cmd.linear.x)   # m/s
            w = float(self.last_cmd.angular.z)  # rad/s

        # diff kinematics in m/s
        v_l_mps = v - w * self.L * 0.5
        v_r_mps = v + w * self.L * 0.5

        # convert to MCU unit (default mm/s)
        v_l_unit = v_l_mps * self.cmd_scale
        v_r_unit = v_r_mps * self.cmd_scale

        try:
            frame = VacuumSerialProtocol.pack_tx_cmd(v_l_unit, v_r_unit)
            self.ser.write(frame)
            # 调试日志：打印发送的速度值
            self.get_logger().debug(
                f"TX: v={v:.3f} m/s, w={w:.3f} rad/s -> "
                f"v_l={v_l_unit:.1f} mm/s, v_r={v_r_unit:.1f} mm/s"
            )
        except Exception as e:
            self.get_logger().warn(f"Serial write failed: {e}")

    # ---------------- RX ----------------
    def read_serial_timer(self) -> None:
        if self.ser is None or not self.ser.is_open:
            return

        try:
            n = self.ser.in_waiting
            if n <= 0:
                return
            data = self.ser.read(n)
            if data:
                self.rx_buf.extend(data)
        except Exception as e:
            self.get_logger().warn(f"Serial read failed: {e}")
            return

        frames = VacuumSerialProtocol.parse_rx_frames(self.rx_buf)
        if not frames:
            return

        fb = frames[-1]
        self.latest_fb = fb

    # -------------- ODOM --------------
    def integrate(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_integ_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        self.last_integ_time = now

        if self.latest_fb is None:
            return

        # wheel speeds: mm/s -> m/s
        v_l = self.latest_fb.v_l_mm_s / 1000.0
        v_r = self.latest_fb.v_r_mm_s / 1000.0

        v = 0.5 * (v_l + v_r)        # m/s
        # yaw from MCU: degrees -> radians
        yaw = math.radians(self.latest_fb.yaw_deg)
        # use yaw from MCU as absolute heading (more stable than integrating w)
        self.yaw = wrap_pi(yaw)

        # integrate x,y using yaw
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt

    def publish_odom_timer(self) -> None:
        self.integrate()

        now_msg = self.get_clock().now().to_msg()
        qx, qy, qz, qw = quat_from_yaw(self.yaw)

        odom = Odometry()
        odom.header.stamp = now_msg
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Provide twist (Nav2 really benefits from this)
        if self.latest_fb is not None:
            v_l = self.latest_fb.v_l_mm_s / 1000.0
            v_r = self.latest_fb.v_r_mm_s / 1000.0
            v = 0.5 * (v_l + v_r)
            w = (v_r - v_l) / self.L
            odom.twist.twist.linear.x = float(v)
            odom.twist.twist.angular.z = float(w)

        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = now_msg
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = float(self.x)
        t.transform.translation.y = float(self.y)
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_pub.sendTransform(t)


def main() -> None:
    rclpy.init()
    node = VacuumSerialBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser is not None and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
