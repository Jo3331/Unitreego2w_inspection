#!/usr/bin/env python3

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray


class StandstillDiagnosticNode(Node):
    def __init__(self):
        super().__init__("standstill_diagnostic_node")

        self.imu_topic = self.declare_parameter("imu_topic", "/SMX/Go2IMU").value
        self.joint_topic = self.declare_parameter("joint_topic", "/SMX/Go2Joint").value
        self.odom_topic = self.declare_parameter("odom_topic", "/SMX/Odom").value
        self.report_period = float(self.declare_parameter("report_period", 1.0).value)

        self.latest_imu = None
        self.latest_joint = None
        self.latest_odom = None

        self.create_subscription(Imu, self.imu_topic, self.imu_callback, 20)
        self.create_subscription(Float64MultiArray, self.joint_topic, self.joint_callback, 20)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        self.create_timer(self.report_period, self.report_once)

        self.get_logger().info(
            f"Standstill diagnostic started: imu={self.imu_topic}, joint={self.joint_topic}, odom={self.odom_topic}"
        )

    def imu_callback(self, msg: Imu):
        self.latest_imu = msg

    def joint_callback(self, msg: Float64MultiArray):
        self.latest_joint = list(msg.data)

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    def report_once(self):
        if self.latest_imu is None or self.latest_joint is None or self.latest_odom is None:
            self.get_logger().warn("Waiting for IMU, joint, and odom messages...")
            return

        imu = self.latest_imu
        joint = self.latest_joint
        odom = self.latest_odom

        accel_x = float(imu.linear_acceleration.x)
        accel_y = float(imu.linear_acceleration.y)
        accel_z = float(imu.linear_acceleration.z)
        accel_xy = math.hypot(accel_x, accel_y)
        gyro_norm = math.sqrt(
            float(imu.angular_velocity.x) ** 2
            + float(imu.angular_velocity.y) ** 2
            + float(imu.angular_velocity.z) ** 2
        )

        joint_dq = joint[16:32] if len(joint) >= 32 else []
        dq_rms = math.sqrt(sum(value * value for value in joint_dq) / len(joint_dq)) if joint_dq else 0.0

        wheel_q = [joint[idx] for idx in (3, 7, 11, 15)] if len(joint) >= 16 else [0.0] * 4
        wheel_dq = [joint[idx] for idx in (19, 23, 27, 31)] if len(joint) >= 32 else [0.0] * 4
        wheel_tau = [joint[idx] for idx in (35, 39, 43, 47)] if len(joint) >= 48 else [0.0] * 4
        wheel_dq_rms = math.sqrt(sum(value * value for value in wheel_dq) / len(wheel_dq))

        linear = odom.twist.twist.linear
        speed_norm = math.sqrt(float(linear.x) ** 2 + float(linear.y) ** 2 + float(linear.z) ** 2)
        position = odom.pose.pose.position

        suspects = []
        if accel_xy > 0.2:
            suspects.append(f"imu_acc_xy={accel_xy:.3f}m/s^2")
        if gyro_norm > 0.02:
            suspects.append(f"imu_gyro_norm={gyro_norm:.3f}rad/s")
        if dq_rms > 0.02:
            suspects.append(f"joint_dq_rms={dq_rms:.3f}rad/s")
        if wheel_dq_rms > 0.01:
            suspects.append(f"wheel_dq_rms={wheel_dq_rms:.3f}rad/s")
        if speed_norm > 0.1:
            suspects.append(f"odom_speed={speed_norm:.3f}m/s")

        suspect_text = ", ".join(suspects) if suspects else "no obvious issue"
        self.get_logger().info(
            "standstill diag | "
            f"acc=({accel_x:.3f}, {accel_y:.3f}, {accel_z:.3f}) xy={accel_xy:.3f} | "
            f"gyro_norm={gyro_norm:.3f} | dq_rms={dq_rms:.3f} | wheel_q={['%.3f' % v for v in wheel_q]} | "
            f"wheel_dq={['%.3f' % v for v in wheel_dq]} | wheel_tau={['%.3f' % v for v in wheel_tau]} | "
            f"odom_pos=({position.x:.3f}, {position.y:.3f}, {position.z:.3f}) | "
            f"odom_speed={speed_norm:.3f} | suspects: {suspect_text}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = StandstillDiagnosticNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()