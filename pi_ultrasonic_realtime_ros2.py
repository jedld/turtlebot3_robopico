#!/usr/bin/env python3
"""
Realtime console viewer for dual ultrasonic ROS2 topics.

Subscribes to:
  /ultrasonic/left
  /ultrasonic/right

and prints continuously updated values.
"""

import argparse
import math
import sys
import time


def fmt_range(value):
    if value is None:
        return "---"
    if math.isinf(value):
        return "inf"
    return f"{value:.3f} m"


def main():
    parser = argparse.ArgumentParser(description="Realtime monitor for dual ultrasonic ROS2 topics")
    parser.add_argument("--left-topic", default="/ultrasonic/left")
    parser.add_argument("--right-topic", default="/ultrasonic/right")
    parser.add_argument("--refresh", type=float, default=0.2, help="screen refresh period in seconds")
    args = parser.parse_args()

    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Range
    except Exception:
        print("ERROR: ROS 2 Python packages unavailable in current shell.")
        print("Run: source ~/turtlebot3_ws/install/setup.bash")
        sys.exit(2)

    class RealtimeNode(Node):
        def __init__(self):
            super().__init__("pi_ultrasonic_realtime")
            self.left = None
            self.right = None
            self.left_t = None
            self.right_t = None

            self.create_subscription(Range, args.left_topic, self.left_cb, 10)
            self.create_subscription(Range, args.right_topic, self.right_cb, 10)

        def left_cb(self, msg):
            self.left = msg.range
            self.left_t = time.time()

        def right_cb(self, msg):
            self.right = msg.range
            self.right_t = time.time()

    rclpy.init()
    node = RealtimeNode()

    print("Monitoring ultrasonic topics (Ctrl+C to stop)")
    print(f"  left : {args.left_topic}")
    print(f"  right: {args.right_topic}")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=args.refresh)
            now = time.time()
            left_age = "-" if node.left_t is None else f"{(now - node.left_t):.2f}s"
            right_age = "-" if node.right_t is None else f"{(now - node.right_t):.2f}s"

            line = (
                f"L={fmt_range(node.left):>8} (age {left_age:>6})   "
                f"R={fmt_range(node.right):>8} (age {right_age:>6})"
            )
            print(line, end="\r", flush=True)
    except rclpy.executors.ExternalShutdownException:
        print()
    except KeyboardInterrupt:
        print()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
