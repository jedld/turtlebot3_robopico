#!/usr/bin/env python3
"""Quick diagnostic: command a rotation and watch IMU yaw + angular_vel_z."""

import math, time, sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

def quat_to_yaw(o):
    siny = 2.0 * (o.w * o.z + o.x * o.y)
    cosy = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
    return math.atan2(siny, cosy)

def angle_diff(a, b):
    d = a - b
    while d >  math.pi: d -= 2*math.pi
    while d < -math.pi: d += 2*math.pi
    return d

class DiagNode(Node):
    def __init__(self):
        super().__init__("diag_imu_rot")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.imu = None
        self.cum_yaw = 0.0
        self.last_yaw = None
        self.samples = []
        self.create_subscription(Imu, "/imu", self._imu_cb, 10)

    def _imu_cb(self, msg):
        self.imu = msg
        yaw = quat_to_yaw(msg.orientation)
        if self.last_yaw is not None:
            self.cum_yaw += angle_diff(yaw, self.last_yaw)
        self.last_yaw = yaw
        self.samples.append((
            time.time(),
            math.degrees(yaw),
            math.degrees(self.cum_yaw),
            math.degrees(msg.angular_velocity.z),
        ))

def main():
    rclpy.init()
    node = DiagNode()

    # Wait for data
    t0 = time.time()
    while node.imu is None and time.time() - t0 < 5:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.imu is None:
        print("No IMU data")
        sys.exit(1)

    # Reset cumulative tracking
    node.cum_yaw = 0.0
    node.samples.clear()

    print("Commanding 0.5 rad/s rotation for 3 seconds...")
    print(f"Expected physical rotation: ~{math.degrees(0.5 * 3):.0f}°")
    cmd = Twist()
    cmd.angular.z = 0.5
    t0 = time.time()
    while time.time() - t0 < 3.0:
        node.pub.publish(cmd)
        rclpy.spin_once(node, timeout_sec=0.05)

    # Stop
    cmd.angular.z = 0.0
    node.pub.publish(cmd)
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.05)

    print(f"\nResults (3s at 0.5 rad/s = expected {math.degrees(0.5*3):.0f}° rotation):")
    print(f"  IMU cumulative yaw:  {node.cum_yaw:.1f}° ({math.degrees(node.cum_yaw):.1f}°)")
    print(f"  Samples collected:   {len(node.samples)}")
    if node.samples:
        gz_vals = [s[3] for s in node.samples]
        print(f"  Avg gyro_z:          {sum(gz_vals)/len(gz_vals):.1f} °/s")
        print(f"  Max gyro_z:          {max(gz_vals):.1f} °/s")
        print(f"\n  Last 10 samples (time, raw_yaw°, cumul°, gyro_z°/s):")
        for s in node.samples[-10:]:
            print(f"    t={s[0]-node.samples[0][0]:.2f}  yaw={s[1]:.1f}°  cum={s[2]:.1f}°  gz={s[3]:.1f}°/s")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
