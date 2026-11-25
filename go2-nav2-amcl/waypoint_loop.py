#!/usr/bin/env python3
"""定点循环巡航节点：按 route.txt 里的航点顺序无限循环 NavigateToPose。

用法:
    python3 waypoint_loop.py [route.txt]

route.txt 每行一个航点:  x y yaw(弧度)，# 开头为注释，空行忽略。
"""
import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

# NavigateToPose 结果状态码
STATUS_SUCCEEDED = 4  # action_msgs/GoalStatus.STATUS_SUCCEEDED


class WaypointLoop(Node):
    def __init__(self, route_file):
        super().__init__('waypoint_loop')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._waypoints = self._load(route_file)
        if not self._waypoints:
            self.get_logger().error('路由文件 %s 里没有有效航点' % route_file)
            raise SystemExit(1)
        self._idx = 0
        self.get_logger().info('加载 %d 个航点，准备循环导航' % len(self._waypoints))

    def _load(self, path):
        waypoints = []
        with open(path, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.split()
                if len(parts) < 2:
                    continue
                x = float(parts[0])
                y = float(parts[1])
                yaw = float(parts[2]) if len(parts) > 2 else 0.0

                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = x
                pose.pose.position.y = y
                half = yaw / 2.0
                pose.pose.orientation.z = math.sin(half)
                pose.pose.orientation.w = math.cos(half)
                waypoints.append(pose)
        return waypoints

    def start(self):
        # 启动时同步等待 action 服务器（在 spin 前阻塞等待，只做一次）
        self.get_logger().info('等待 navigate_to_pose action 服务器...')
        while rclpy.ok() and not self._client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('仍未连接 navigate_to_pose，继续等待...',
                                   throttle_duration_sec=5.0)
        self._send_next()

    def _send_next(self):
        goal = NavigateToPose.Goal()
        goal.pose = self._waypoints[self._idx]
        p = goal.pose.pose.position
        self.get_logger().info('Going to waypoint %d (%.2f, %.2f)' %
                               (self._idx, p.x, p.y))
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._goal_response)

    def _goal_response(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('waypoint %d 目标被拒，跳到下一个' % self._idx)
            self._advance()
            return
        goal_handle.get_result_async().add_done_callback(self._result)

    def _result(self, future):
        result = future.result()
        status = getattr(result, 'status', -1)
        if status == STATUS_SUCCEEDED:
            self.get_logger().info('Reached waypoint %d' % self._idx)
        else:
            self.get_logger().warn('waypoint %d 结束但 status=%d（可能失败/中止），继续下一个'
                                   % (self._idx, status))
        self._advance()

    def _advance(self):
        self._idx = (self._idx + 1) % len(self._waypoints)
        self._send_next()


def main():
    rclpy.init()
    route_file = sys.argv[1] if len(sys.argv) > 1 else 'route.txt'
    node = WaypointLoop(route_file)
    try:
        node.start()
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
