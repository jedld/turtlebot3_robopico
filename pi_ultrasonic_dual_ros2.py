#!/usr/bin/env python3
"""
Publish two Raspberry Pi GPIO ultrasonic sensors as sensor_msgs/Range.

Topics (default):
  /ultrasonic/left
  /ultrasonic/right

Example:
  python3 pi_ultrasonic_dual_ros2.py --left-gpio 23 --right-gpio 24
"""

import argparse
import sys
import time


def read_distance_mm(gpio_mod, gpio: int, timeout_us: int = 30000):
    try:
        gpio_mod.setup(gpio, gpio_mod.OUT, initial=gpio_mod.LOW)
        time.sleep(2e-6)
        gpio_mod.output(gpio, gpio_mod.HIGH)
        time.sleep(10e-6)
        gpio_mod.output(gpio, gpio_mod.LOW)

        gpio_mod.setup(gpio, gpio_mod.IN)

        start_ns = time.perf_counter_ns()
        timeout_ns = int(timeout_us * 1000)
        while gpio_mod.input(gpio) == 0:
            if (time.perf_counter_ns() - start_ns) > timeout_ns:
                return None, "wait-timeout"

        rise_ns = time.perf_counter_ns()
        while gpio_mod.input(gpio) == 1:
            if (time.perf_counter_ns() - rise_ns) > timeout_ns:
                return None, "pulse-timeout"

        fall_ns = time.perf_counter_ns()
        pulse_us = (fall_ns - rise_ns) / 1000.0

        distance_mm = (pulse_us * 10.0) / 58.0
        if distance_mm < 20.0 or distance_mm > 3500.0:
            return None, "out-of-range"

        return distance_mm, None
    except Exception as exc:
        return None, f"gpio-error: {exc}"


def main():
    parser = argparse.ArgumentParser(description="ROS2 publisher for dual Pi ultrasonic sensors")
    parser.add_argument("--left-gpio", type=int, default=23)
    parser.add_argument("--right-gpio", type=int, default=24)
    parser.add_argument("--left-topic", default="/ultrasonic/left")
    parser.add_argument("--right-topic", default="/ultrasonic/right")
    parser.add_argument("--left-frame", default="ultrasonic_left_link")
    parser.add_argument("--right-frame", default="ultrasonic_right_link")
    parser.add_argument("--rate", type=float, default=8.0, help="publish rate in Hz (default: 8)")
    parser.add_argument("--inter-sensor-gap", type=float, default=0.05, help="seconds between left/right trigger")
    parser.add_argument("--timeout-us", type=int, default=30000)
    args = parser.parse_args()

    if args.left_gpio == args.right_gpio:
        print("ERROR: --left-gpio and --right-gpio must be different")
        sys.exit(2)

    try:
        import RPi.GPIO as GPIO
    except Exception:
        print("ERROR: RPi.GPIO backend is unavailable.")
        print("Install with: sudo apt install python3-rpi-lgpio")
        sys.exit(2)

    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from sensor_msgs.msg import Range
    except Exception:
        print("ERROR: ROS 2 Python packages unavailable in current shell.")
        print("Source your ROS environment first, e.g.: source ~/turtlebot3_ws/install/setup.bash")
        sys.exit(2)

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    class UltrasonicNode(Node):
        def __init__(self):
            super().__init__("pi_ultrasonic_dual")
            self.pub_left = self.create_publisher(Range, args.left_topic, 10)
            self.pub_right = self.create_publisher(Range, args.right_topic, 10)
            self.period = max(0.02, 1.0 / max(0.1, args.rate))
            self.timer = self.create_timer(self.period, self.tick)
            self._shutting_down = False
            self._last_tick_error = None

            self.min_range = 0.02
            self.max_range = 3.50
            self.fov_rad = 0.26

            self.get_logger().info(
                f"Publishing left GPIO{args.left_gpio}->{args.left_topic}, "
                f"right GPIO{args.right_gpio}->{args.right_topic} at {1.0/self.period:.1f} Hz"
            )

        def stop(self):
            self._shutting_down = True
            self.timer.cancel()

        def publish_one(self, pub, frame_id, dist_mm):
            if self._shutting_down or not rclpy.ok():
                return

            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = frame_id
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = float(self.fov_rad)
            msg.min_range = float(self.min_range)
            msg.max_range = float(self.max_range)

            if dist_mm is None:
                msg.range = float("inf")
            else:
                msg.range = float(dist_mm / 1000.0)

            try:
                pub.publish(msg)
            except Exception:
                self._shutting_down = True

        def tick(self):
            if self._shutting_down or not rclpy.ok():
                return

            left_mm, left_err = read_distance_mm(GPIO, args.left_gpio, args.timeout_us)
            time.sleep(max(0.0, args.inter_sensor_gap))
            right_mm, right_err = read_distance_mm(GPIO, args.right_gpio, args.timeout_us)

            try:
                self.publish_one(self.pub_left, args.left_frame, left_mm)
                self.publish_one(self.pub_right, args.right_frame, right_mm)
            except Exception as exc:
                err_text = str(exc)
                if err_text != self._last_tick_error:
                    self.get_logger().warning(f"ultrasonic publish tick failed: {err_text}")
                    self._last_tick_error = err_text
                return

            self._last_tick_error = None

            if left_err or right_err:
                self.get_logger().debug(
                    f"left={left_mm if left_mm is not None else left_err}, "
                    f"right={right_mm if right_mm is not None else right_err}"
                )

    rclpy.init()
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        try:
            GPIO.cleanup(args.left_gpio)
            GPIO.cleanup(args.right_gpio)
        except Exception:
            pass


if __name__ == "__main__":
    main()
