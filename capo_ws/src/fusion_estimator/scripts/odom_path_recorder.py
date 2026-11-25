#!/usr/bin/env python3
"""
Odom Path Recorder — 订阅里程计，累计轨迹并以 Path 消息发布，在 RViz 中显示。
用于观察回环误差：起点 (0,0) 和终点之间的偏移即漂移量。

用法:
  ros2 run fusion_estimator odom_path_recorder.py  # 使用默认参数
  ros2 run fusion_estimator odom_path_recorder.py --ros-args -p odom_topic:=/SMX/Odom_2D

RViz 配置: 添加 Path display, topic 选 /odom_path
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class OdomPathRecorder(Node):
    def __init__(self):
        super().__init__("odom_path_recorder")

        # ---- 参数 ----
        self.odom_topic = self.declare_parameter(
            "odom_topic", "/SMX/Odom").value
        self.odom2d_topic = self.declare_parameter(
            "odom2d_topic", "/SMX/Odom_2D").value
        self.world_frame = self.declare_parameter(
            "world_frame", "odom").value
        self.max_points = self.declare_parameter(
            "max_points", 0).value  # 0 = 不限
        self.line_thickness = self.declare_parameter(
            "line_thickness", 0.05).value
        self.loop_marker_scale = self.declare_parameter(
            "loop_marker_scale", 0.3).value

        # ---- 内部状态 ----
        self.path_3d = Path()
        self.path_3d.header.frame_id = self.world_frame
        self.path_2d = Path()
        self.path_2d.header.frame_id = self.world_frame

        # 记录第一个点和最新点，用于画回环误差标记
        self.first_pose = None       # 3D
        self.first_pose_2d = None    # 2D
        self.latest_pose = None
        self.latest_pose_2d = None
        self.distance_traveled_3d = 0.0
        self.distance_traveled_2d = 0.0
        self.last_pose_3d = None
        self.last_pose_2d = None

        # 起始位置（用第一个点做原点）
        self.origin_x = None
        self.origin_y = None
        self.origin_z = None
        self.origin_2d_x = None
        self.origin_2d_y = None

        # ---- 订阅器 ----
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10)
        self.odom2d_sub = self.create_subscription(
            Odometry, self.odom2d_topic, self.odom2d_callback, 10)

        # ---- 发布器 ----
        self.path3d_pub = self.create_publisher(Path, "/odom_path_3d", 10)
        self.path2d_pub = self.create_publisher(Path, "/odom_path_2d", 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/odom_path_markers", 10)

        # ---- 定时器：定期发布 path ----
        self.timer = self.create_timer(0.1, self.publish_path)

        self.get_logger().info(
            f"OdomPathRecorder started\n"
            f"  3D odom: {self.odom_topic}\n"
            f"  2D odom: {self.odom2d_topic}\n"
            f"  Path topics: /odom_path_3d, /odom_path_2d\n"
            f"  Marker topic: /odom_path_markers"
        )

    # ------------------------------------------------------------------
    def odom_callback(self, msg: Odometry):
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self.world_frame
        pose.pose = msg.pose.pose

        # 以第一个点为原点
        if self.origin_x is None:
            self.origin_x = pose.pose.position.x
            self.origin_y = pose.pose.position.y
            self.origin_z = pose.pose.position.z
            self.first_pose = pose

        # 累计距离
        if self.last_pose_3d is not None:
            dx = pose.pose.position.x - self.last_pose_3d.pose.position.x
            dy = pose.pose.position.y - self.last_pose_3d.pose.position.y
            dz = pose.pose.position.z - self.last_pose_3d.pose.position.z
            self.distance_traveled_3d += math.sqrt(dx*dx + dy*dy + dz*dz)

        self.last_pose_3d = pose
        self.latest_pose = pose

        self.path_3d.poses.append(pose)
        self.path_3d.header.stamp = msg.header.stamp
        self._trim(self.path_3d)

    # ------------------------------------------------------------------
    def odom2d_callback(self, msg: Odometry):
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self.world_frame
        pose.pose = msg.pose.pose

        if self.origin_2d_x is None:
            self.origin_2d_x = pose.pose.position.x
            self.origin_2d_y = pose.pose.position.y
            self.first_pose_2d = pose

        if self.last_pose_2d is not None:
            dx = pose.pose.position.x - self.last_pose_2d.pose.position.x
            dy = pose.pose.position.y - self.last_pose_2d.pose.position.y
            self.distance_traveled_2d += math.sqrt(dx*dx + dy*dy)

        self.last_pose_2d = pose
        self.latest_pose_2d = pose

        self.path_2d.poses.append(pose)
        self.path_2d.header.stamp = msg.header.stamp
        self._trim(self.path_2d)

    # ------------------------------------------------------------------
    def _trim(self, path: Path):
        if self.max_points > 0 and len(path.poses) > self.max_points:
            path.poses = path.poses[-self.max_points:]

    # ------------------------------------------------------------------
    def publish_path(self):
        """定时发布 path + 起终点标记"""
        now = self.get_clock().now().to_msg()

        # 更新时间戳
        self.path_3d.header.stamp = now
        self.path_2d.header.stamp = now

        # 发布 path
        self.path3d_pub.publish(self.path_3d)
        self.path2d_pub.publish(self.path_2d)

        # 发布起终点标记（用于看回环误差）
        markers = MarkerArray()

        # ---- 3D：起点 (绿色球) + 当前点 (红色球) + 连线 (误差线) ----
        if self.first_pose is not None and self.latest_pose is not None:
            # 起点 — 绿色
            markers.markers.append(self._make_sphere(
                ns="start_3d", id=0,
                pose=self.first_pose.pose,
                r=0.0, g=1.0, b=0.0, a=0.8,
                scale=self.loop_marker_scale))

            # 当前位 — 红色
            markers.markers.append(self._make_sphere(
                ns="current_3d", id=1,
                pose=self.latest_pose.pose,
                r=1.0, g=0.0, b=0.0, a=0.8,
                scale=self.loop_marker_scale))

            # 起点到当前点的误差线 — 黄色
            markers.markers.append(self._make_line(
                ns="drift_3d", id=2,
                p1=self.first_pose.pose.position,
                p2=self.latest_pose.pose.position,
                r=1.0, g=1.0, b=0.0, a=0.7))

            # 漂移量文本 — 白色文字标记在终点上方
            dx = self.latest_pose.pose.position.x - self.first_pose.pose.position.x
            dy = self.latest_pose.pose.position.y - self.first_pose.pose.position.y
            drift = math.sqrt(dx*dx + dy*dy)
            markers.markers.append(self._make_text(
                ns="drift_text_3d", id=3,
                x=self.latest_pose.pose.position.x,
                y=self.latest_pose.pose.position.y,
                z=self.latest_pose.pose.position.z + 0.3,
                text=f"Drift: {drift:.3f}m\nDist: {self.distance_traveled_3d:.1f}m"))

        # ---- 2D：起点 (青色球) + 当前点 (橙色球) + 误差线 ----
        if self.first_pose_2d is not None and self.latest_pose_2d is not None:
            markers.markers.append(self._make_sphere(
                ns="start_2d", id=10,
                pose=self.first_pose_2d.pose,
                r=0.0, g=1.0, b=1.0, a=0.8,
                scale=self.loop_marker_scale))

            markers.markers.append(self._make_sphere(
                ns="current_2d", id=11,
                pose=self.latest_pose_2d.pose,
                r=1.0, g=0.5, b=0.0, a=0.8,
                scale=self.loop_marker_scale))

            markers.markers.append(self._make_line(
                ns="drift_2d", id=12,
                p1=self.first_pose_2d.pose.position,
                p2=self.latest_pose_2d.pose.position,
                r=1.0, g=0.5, b=0.0, a=0.7))

            dx2 = self.latest_pose_2d.pose.position.x - self.first_pose_2d.pose.position.x
            dy2 = self.latest_pose_2d.pose.position.y - self.first_pose_2d.pose.position.y
            drift2 = math.sqrt(dx2*dx2 + dy2*dy2)
            markers.markers.append(self._make_text(
                ns="drift_text_2d", id=13,
                x=self.latest_pose_2d.pose.position.x,
                y=self.latest_pose_2d.pose.position.y,
                z=self.latest_pose_2d.pose.position.z + 0.6,
                text=f"Drift2D: {drift2:.3f}m\nDist: {self.distance_traveled_2d:.1f}m"))

        self.marker_pub.publish(markers)

    # ------------------------------------------------------------------
    def _make_sphere(self, ns: str, id: int, pose, r, g, b, a, scale):
        m = Marker()
        m.header.frame_id = self.world_frame
        m.ns = ns
        m.id = id
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose = pose
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color = ColorRGBA(r=r, g=g, b=b, a=a)
        return m

    def _make_line(self, ns: str, id: int, p1, p2, r, g, b, a):
        m = Marker()
        m.header.frame_id = self.world_frame
        m.ns = ns
        m.id = id
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.points = [p1, p2]
        m.scale.x = self.line_thickness
        m.color = ColorRGBA(r=r, g=g, b=b, a=a)
        return m

    def _make_text(self, ns: str, id: int, x, y, z, text):
        m = Marker()
        m.header.frame_id = self.world_frame
        m.ns = ns
        m.id = id
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.scale.z = 0.15
        m.text = text
        m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
        return m


def main():
    rclpy.init()
    node = OdomPathRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
