#!/usr/bin/env python3

import math
import sys
from pathlib import Path
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None

try:
    from unitree_go.msg import LowState
except ImportError as exc:
    LowState = None
    LOWSTATE_IMPORT_ERROR = exc
else:
    LOWSTATE_IMPORT_ERROR = None


def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qw, qx, qy, qz


def quaternion_conjugate(qw: float, qx: float, qy: float, qz: float):
    return qw, -qx, -qy, -qz


def quaternion_multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def rotate_vector(qw: float, qx: float, qy: float, qz: float, vector):
    pure = (0.0, vector[0], vector[1], vector[2])
    rotated = quaternion_multiply(quaternion_multiply((qw, qx, qy, qz), pure), quaternion_conjugate(qw, qx, qy, qz))
    return [rotated[1], rotated[2], rotated[3]]


class UnitreeLowstateAdapterNode(Node):
    def __init__(self):
        super().__init__("unitree_lowstate_adapter_node")

        self.sub_lowstate_topic = self.declare_parameter(
            "sub_lowstate_topic", "lowstate").value
        self.pub_imu_topic = self.declare_parameter(
            "pub_imu_topic", "SMX/Go2IMU").value
        self.pub_joint_topic = self.declare_parameter(
            "pub_joint_topic", "SMX/Go2Joint").value
        self.imu_frame = self.declare_parameter(
            "imu_frame", "base_imu").value
        self.orientation_source = self.declare_parameter(
            "orientation_source", "rpy").value
        self.contact_source = self.declare_parameter(
            "contact_source", "motor_tau").value
        self.foot_force_scale = float(self.declare_parameter(
            "foot_force_scale", 1.0).value)
        self.motor_index_map = [
            int(value) for value in self.declare_parameter(
                "motor_index_map", list(range(16))).value
        ]
        self.enable_standstill_deadzone = bool(self.declare_parameter(
            "enable_standstill_deadzone", True).value)
        self.standstill_gyro_norm_threshold = float(self.declare_parameter(
            "standstill_gyro_norm_threshold", 0.05).value)
        self.standstill_joint_dq_rms_threshold = float(self.declare_parameter(
            "standstill_joint_dq_rms_threshold", 0.08).value)
        self.standstill_accel_norm_target = float(self.declare_parameter(
            "standstill_accel_norm_target", 9.81).value)
        self.standstill_accel_norm_tolerance = float(self.declare_parameter(
            "standstill_accel_norm_tolerance", 0.8).value)
        self.standstill_required_frames = int(self.declare_parameter(
            "standstill_required_frames", 5).value)
        self.gyro_deadzone = float(self.declare_parameter(
            "gyro_deadzone", 0.03).value)
        self.accel_world_xy_deadzone = float(self.declare_parameter(
            "accel_world_xy_deadzone", 0.15).value)
        self.accel_world_z_deadzone = float(self.declare_parameter(
            "accel_world_z_deadzone", 0.25).value)
        self.joint_dq_deadzone = float(self.declare_parameter(
            "joint_dq_deadzone", 0.05).value)
        self.joint_tau_deadzone = float(self.declare_parameter(
            "joint_tau_deadzone", 0.10).value)

        self._validate_parameters()

        self.last_stamp = None
        self.standstill_counter = 0

        self.imu_publisher = self.create_publisher(Imu, self.pub_imu_topic, 20)
        self.joint_publisher = self.create_publisher(Float64MultiArray, self.pub_joint_topic, 20)
        self.lowstate_subscriber = self.create_subscription(
            LowState,
            self.sub_lowstate_topic,
            self.lowstate_callback,
            20,
        )

        self.get_logger().info(
            "Unitree lowstate adapter started: %s -> (%s, %s)"
            % (self.sub_lowstate_topic, self.pub_imu_topic, self.pub_joint_topic)
        )

    def _validate_parameters(self):
        if len(self.motor_index_map) != 16:
            raise ValueError("motor_index_map must contain exactly 16 indices for wheel mode")

        if any(index < 0 or index >= 20 for index in self.motor_index_map):
            raise ValueError("motor_index_map values must stay in LowState.motor_state[0..19]")

        if self.orientation_source not in {"rpy", "quaternion"}:
            raise ValueError("orientation_source must be 'rpy' or 'quaternion'")

        if self.contact_source not in {"motor_tau", "foot_force", "foot_force_est"}:
            raise ValueError("contact_source must be 'motor_tau', 'foot_force', or 'foot_force_est'")

    def _next_stamp(self):
        now = self.get_clock().now()
        if self.last_stamp is not None and now <= self.last_stamp:
            now = Time(nanoseconds=self.last_stamp.nanoseconds + 1)
        self.last_stamp = now
        return now.to_msg()

    @staticmethod
    def _apply_deadzone(value: float, threshold: float) -> float:
        return 0.0 if abs(value) < threshold else value

    def _standstill_detected(self, low_state: LowState) -> bool:
        gyro = [float(value) for value in low_state.imu_state.gyroscope]
        gyro_norm = math.sqrt(sum(value * value for value in gyro))

        mapped_dq = [
            float(low_state.motor_state[src_index].dq)
            for src_index in self.motor_index_map
        ]
        joint_dq_rms = math.sqrt(sum(value * value for value in mapped_dq) / len(mapped_dq))

        accel = [float(value) for value in low_state.imu_state.accelerometer]
        accel_norm = math.sqrt(sum(value * value for value in accel))

        static_now = (
            gyro_norm <= self.standstill_gyro_norm_threshold
            and joint_dq_rms <= self.standstill_joint_dq_rms_threshold
            and abs(accel_norm - self.standstill_accel_norm_target) <= self.standstill_accel_norm_tolerance
        )

        if static_now:
            self.standstill_counter += 1
        else:
            self.standstill_counter = 0

        return self.standstill_counter >= self.standstill_required_frames

    def _fill_orientation(self, imu_msg: Imu, low_state: LowState):
        if self.orientation_source == "quaternion":
            quat = low_state.imu_state.quaternion
            imu_msg.orientation.w = float(quat[0])
            imu_msg.orientation.x = float(quat[1])
            imu_msg.orientation.y = float(quat[2])
            imu_msg.orientation.z = float(quat[3])
            return

        roll = float(low_state.imu_state.rpy[0])
        pitch = float(low_state.imu_state.rpy[1])
        yaw = float(low_state.imu_state.rpy[2])
        qw, qx, qy, qz = euler_to_quaternion(roll, pitch, yaw)
        imu_msg.orientation.w = qw
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz

    def _get_orientation_quaternion(self, low_state: LowState):
        if self.orientation_source == "quaternion":
            quat = low_state.imu_state.quaternion
            return float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])

        roll = float(low_state.imu_state.rpy[0])
        pitch = float(low_state.imu_state.rpy[1])
        yaw = float(low_state.imu_state.rpy[2])
        return euler_to_quaternion(roll, pitch, yaw)

    def _filter_linear_acceleration(self, low_state: LowState, standstill_active: bool):
        accel_body = [float(value) for value in low_state.imu_state.accelerometer]
        if not standstill_active:
            return accel_body

        qw, qx, qy, qz = self._get_orientation_quaternion(low_state)
        accel_world = rotate_vector(qw, qx, qy, qz, accel_body)

        accel_world[0] = self._apply_deadzone(accel_world[0], self.accel_world_xy_deadzone)
        accel_world[1] = self._apply_deadzone(accel_world[1], self.accel_world_xy_deadzone)

        if abs(accel_world[2] - self.standstill_accel_norm_target) < self.accel_world_z_deadzone:
            accel_world[2] = self.standstill_accel_norm_target

        qw_inv, qx_inv, qy_inv, qz_inv = quaternion_conjugate(qw, qx, qy, qz)
        return rotate_vector(qw_inv, qx_inv, qy_inv, qz_inv, accel_world)

    def _build_joint_array(self, low_state: LowState, standstill_active: bool) -> List[float]:
        joint_data = [0.0] * 48

        for dst_index, src_index in enumerate(self.motor_index_map):
            motor_state = low_state.motor_state[src_index]
            is_wheel_joint = (dst_index % 4 == 3)
            joint_data[dst_index] = float(motor_state.q)
            joint_data[16 + dst_index] = self._apply_deadzone(
                float(motor_state.dq),
                0.0 if is_wheel_joint else (self.joint_dq_deadzone if standstill_active else 0.0),
            )

        if self.contact_source == "motor_tau":
            for dst_index, src_index in enumerate(self.motor_index_map):
                joint_data[32 + dst_index] = self._apply_deadzone(
                    float(low_state.motor_state[src_index].tau_est),
                    self.joint_tau_deadzone if standstill_active else 0.0,
                )
            return joint_data

        foot_force = low_state.foot_force if self.contact_source == "foot_force" else low_state.foot_force_est
        for leg_index in range(4):
            joint_data[32 + leg_index * 4 + 2] = float(foot_force[leg_index]) * self.foot_force_scale

        return joint_data

    def lowstate_callback(self, low_state: LowState):
        stamp = self._next_stamp()
        standstill_active = self.enable_standstill_deadzone and self._standstill_detected(low_state)

        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self.imu_frame
        self._fill_orientation(imu_msg, low_state)

        imu_msg.angular_velocity.x = self._apply_deadzone(
            float(low_state.imu_state.gyroscope[0]),
            self.gyro_deadzone if standstill_active else 0.0,
        )
        imu_msg.angular_velocity.y = self._apply_deadzone(
            float(low_state.imu_state.gyroscope[1]),
            self.gyro_deadzone if standstill_active else 0.0,
        )
        imu_msg.angular_velocity.z = self._apply_deadzone(
            float(low_state.imu_state.gyroscope[2]),
            self.gyro_deadzone if standstill_active else 0.0,
        )

        filtered_accel = self._filter_linear_acceleration(low_state, standstill_active)
        imu_msg.linear_acceleration.x = filtered_accel[0]
        imu_msg.linear_acceleration.y = filtered_accel[1]
        imu_msg.linear_acceleration.z = filtered_accel[2]

        joint_msg = Float64MultiArray()
        joint_msg.data = self._build_joint_array(low_state, standstill_active)

        self.imu_publisher.publish(imu_msg)
        self.joint_publisher.publish(joint_msg)


def default_param_file() -> str:
    if get_package_share_directory is None:
        return ""

    try:
        share_dir = Path(get_package_share_directory("fusion_estimator"))
    except Exception:
        return ""

    config_path = share_dir / "config.yaml"
    return str(config_path) if config_path.exists() else ""


def main(args=None):
    if LowState is None:
        print(
            "Failed to import unitree_go.msg.LowState. Source your Unitree ROS2 workspace first: %s"
            % LOWSTATE_IMPORT_ERROR,
            file=sys.stderr,
        )
        return 1

    cli_args = list(args) if args is not None else sys.argv[1:]
    if not cli_args:
        config_path = default_param_file()
        if config_path:
            cli_args = ["--ros-args", "--params-file", config_path]

    rclpy.init(args=cli_args)
    node = UnitreeLowstateAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())