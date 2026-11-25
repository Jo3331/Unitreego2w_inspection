#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from unitree_go.msg import LowState


class LowStateMotorDiagnosticNode(Node):
    def __init__(self):
        super().__init__("lowstate_motor_diagnostic_node")

        self.lowstate_topic = self.declare_parameter("lowstate_topic", "lf/lowstate").value
        self.report_period = float(self.declare_parameter("report_period", 0.5).value)
        self.top_k = int(self.declare_parameter("top_k", 6).value)

        self.latest_lowstate = None
        self.baseline_q = None
        self.message_count = 0

        self.create_subscription(LowState, self.lowstate_topic, self.lowstate_callback, 50)
        self.create_timer(self.report_period, self.report_once)

        self.get_logger().info(
            f"LowState motor diagnostic started: topic={self.lowstate_topic}, report_period={self.report_period}s"
        )

    def lowstate_callback(self, msg: LowState):
        self.latest_lowstate = msg
        self.message_count += 1

        if self.baseline_q is None:
            self.baseline_q = [float(motor.q) for motor in msg.motor_state]

    def report_once(self):
        if self.latest_lowstate is None:
            self.get_logger().warn("Waiting for lowstate messages...")
            return

        motors = self.latest_lowstate.motor_state
        baseline_q = self.baseline_q or [0.0] * len(motors)

        rows = []
        for index, motor in enumerate(motors):
            q = float(motor.q)
            dq = float(motor.dq)
            tau = float(motor.tau_est)
            delta_q = q - baseline_q[index]
            rows.append({
                "index": index,
                "q": q,
                "dq": dq,
                "tau": tau,
                "delta_q": delta_q,
                "abs_dq": abs(dq),
                "abs_delta_q": abs(delta_q),
            })

        top_by_dq = sorted(rows, key=lambda row: row["abs_dq"], reverse=True)[: self.top_k]
        top_by_delta_q = sorted(rows, key=lambda row: row["abs_delta_q"], reverse=True)[: self.top_k]

        dq_summary = ", ".join(
            f"{row['index']:02d}:dq={row['dq']:+.3f},q={row['q']:+.3f},dq0={row['delta_q']:+.3f}"
            for row in top_by_dq
        )
        delta_q_summary = ", ".join(
            f"{row['index']:02d}:dq0={row['delta_q']:+.3f},dq={row['dq']:+.3f},q={row['q']:+.3f}"
            for row in top_by_delta_q
        )

        all_motors = " | ".join(
            f"{row['index']:02d}:q={row['q']:+.3f},dq={row['dq']:+.3f},tau={row['tau']:+.3f}"
            for row in rows
        )

        self.get_logger().info(
            f"lowstate diag | msgs={self.message_count} | top|dq|: {dq_summary}"
        )
        self.get_logger().info(
            f"lowstate diag | top|q-q0|: {delta_q_summary}"
        )
        self.get_logger().info(
            f"lowstate diag | motors: {all_motors}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = LowStateMotorDiagnosticNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()