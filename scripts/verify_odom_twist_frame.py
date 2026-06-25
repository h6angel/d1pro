#!/usr/bin/env python3
"""Verify whether /ov_msckf/odomimu twist.linear is body or world frame.

OpenVINS publishes v_body = R_GtoI * v_IinG (see ov_msckf ROS2Visualizer.cpp).
Compare twist.linear directly vs R_ItoG * twist.linear against d(pos)/dt.

Usage (robot moving straight, ~0.3+ m/s, minimal rotation):
  ros2 run ...   # or ensure odom topic is publishing
  python3 scripts/verify_odom_twist_frame.py

Press Ctrl+C to stop and print summary.
"""

from __future__ import annotations

import math
import sys
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def quat_to_rot(qw: float, qx: float, qy: float, qz: float):
    """Body->world rotation matrix R_ItoG (Hamilton, active rotation)."""
    return [
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
        [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
        [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)],
    ]


def mat_vec(m, v):
    return (
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    )


def hypot3(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


class VerifyOdomTwistFrame(Node):
    def __init__(self):
        super().__init__("verify_odom_twist_frame")
        self.topic = self.declare_parameter("odom_topic", "/ov_msckf/odomimu").value
        self.window = int(self.declare_parameter("window", 30).value)
        self.samples: deque[dict] = deque(maxlen=self.window)
        self.child_frame_id: str | None = None
        self.header_frame_id: str | None = None
        self.create_subscription(Odometry, self.topic, self.cb, 10)
        self.get_logger().info(
            f"Listening {self.topic}. Drive straight ~0.3m/s with little rotation, then Ctrl+C."
        )

    def cb(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )
        q = msg.pose.pose.orientation
        v = (
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        )
        self.child_frame_id = msg.child_frame_id
        self.header_frame_id = msg.header.frame_id
        self.samples.append({"t": t, "p": p, "q": (q.w, q.x, q.y, q.z), "v": v})

    def summarize(self):
        if len(self.samples) < 5:
            print("Not enough samples. Is the topic publishing?")
            return 1

        print(f"\n=== Odom twist frame check: {self.topic} ===")
        print(f"header.frame_id     = {self.header_frame_id!r}")
        print(f"child_frame_id      = {self.child_frame_id!r}  (ROS: twist in this frame)")

        err_direct = []
        err_rotated = []
        for i in range(1, len(self.samples)):
            a, b = self.samples[i - 1], self.samples[i]
            dt = b["t"] - a["t"]
            if dt <= 1e-4 or dt > 0.5:
                continue
            v_num = (
                (b["p"][0] - a["p"][0]) / dt,
                (b["p"][1] - a["p"][1]) / dt,
                (b["p"][2] - a["p"][2]) / dt,
            )
            v_tw = b["v"]
            r = quat_to_rot(*b["q"])
            v_body_to_world = mat_vec(r, v_tw)

            err_direct.append(hypot3((v_num[0] - v_tw[0], v_num[1] - v_tw[1], v_num[2] - v_tw[2])))
            err_rotated.append(
                hypot3(
                    (
                        v_num[0] - v_body_to_world[0],
                        v_num[1] - v_body_to_world[1],
                        v_num[2] - v_body_to_world[2],
                    )
                )
            )

        if not err_direct:
            print("Could not form valid dt pairs.")
            return 1

        mean_direct = sum(err_direct) / len(err_direct)
        mean_rotated = sum(err_rotated) / len(err_rotated)
        print(f"\nSamples used: {len(err_direct)}")
        print(f"Mean |v_num - twist.linear|           (world hypothesis): {mean_direct:.4f} m/s")
        print(f"Mean |v_num - R_ItoG*twist.linear| (body hypothesis):   {mean_rotated:.4f} m/s")

        if mean_rotated < mean_direct * 0.5:
            print("\n>>> CONCLUSION: twist.linear is BODY (IMU) frame. Planner must rotate to world.")
        elif mean_direct < mean_rotated * 0.5:
            print("\n>>> CONCLUSION: twist.linear is WORLD (global) frame.")
        else:
            print("\n>>> INCONCLUSIVE: try longer straight run or check time sync.")

        print("\nOpenVINS source (ROS2Visualizer.cpp): state_plus(7:9) = R_GtoI * v_IinG  // body frame")
        return 0


def main():
    rclpy.init()
    node = VerifyOdomTwistFrame()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rc = node.summarize()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(rc)


if __name__ == "__main__":
    main()
