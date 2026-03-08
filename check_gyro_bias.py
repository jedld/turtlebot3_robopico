#!/usr/bin/env python3
"""Quick check: is the BNO055 gyro-Z biased even when the robot is stationary?
Also reads odom twist to see if the heading hold is actively correcting.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import time, math, sys

class GyroBiasCheck(Node):
    def __init__(self):
        super().__init__('gyro_bias_check')
        self.gz_samples = []
        self.odom_wz_samples = []
        self.joint_pos_l = []
        self.joint_pos_r = []
        self.joint_vel_l = []
        self.joint_vel_r = []
        self.odom_vx_samples = []
        self.imu_sub = self.create_subscription(Imu, '/imu', self._imu_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._sampling = False

    def _imu_cb(self, msg):
        if self._sampling:
            self.gz_samples.append(msg.angular_velocity.z)

    def _odom_cb(self, msg):
        if self._sampling:
            self.odom_wz_samples.append(msg.twist.twist.angular.z)
            self.odom_vx_samples.append(msg.twist.twist.linear.x)

    def _joint_cb(self, msg):
        if self._sampling and len(msg.position) >= 2:
            self.joint_pos_l.append(msg.position[0])
            self.joint_pos_r.append(msg.position[1])
            if len(msg.velocity) >= 2:
                self.joint_vel_l.append(msg.velocity[0])
                self.joint_vel_r.append(msg.velocity[1])

    def spin_for(self, secs):
        end = time.monotonic() + secs
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def send_vel(self, lin_x, ang_z=0.0):
        msg = Twist()
        msg.linear.x = lin_x
        msg.angular.z = ang_z
        self.cmd_pub.publish(msg)

    def stop(self):
        for _ in range(5):
            self.send_vel(0.0)
            rclpy.spin_once(self, timeout_sec=0.02)

def main():
    rclpy.init()
    node = GyroBiasCheck()

    print("Waiting for topics...")
    node.spin_for(1.0)
    print("OK\n")

    # Phase 1: Record gyro at rest for 3s
    print("=== Phase 1: Gyro at REST (3 seconds) ===")
    node.gz_samples.clear()
    node.odom_wz_samples.clear()
    node._sampling = True
    node.spin_for(3.0)
    node._sampling = False

    if node.gz_samples:
        mean_gz = sum(node.gz_samples) / len(node.gz_samples)
        min_gz = min(node.gz_samples)
        max_gz = max(node.gz_samples)
        print(f"  Gyro-Z samples: {len(node.gz_samples)}")
        print(f"  Mean gyro-Z:    {mean_gz:+.6f} rad/s  ({math.degrees(mean_gz):+.3f} °/s)")
        print(f"  Range:          [{min_gz:+.6f}, {max_gz:+.6f}] rad/s")
    if node.odom_wz_samples:
        mean_owz = sum(node.odom_wz_samples) / len(node.odom_wz_samples)
        print(f"  Odom ω_z mean:  {mean_owz:+.6f} rad/s")

    # Phase 2: Drive forward at 0.10 m/s for 3s
    drive = '--drive' in sys.argv
    if drive:
        print("\n=== Phase 2: DRIVING at 0.10 m/s (3 seconds) ===")
        node.gz_samples.clear()
        node.odom_wz_samples.clear()
        node.odom_vx_samples.clear()
        node.joint_pos_l.clear()
        node.joint_pos_r.clear()
        node.joint_vel_l.clear()
        node.joint_vel_r.clear()
        node._sampling = True

        for i in range(60):  # 3s at 20Hz
            node.send_vel(0.10)
            rclpy.spin_once(node, timeout_sec=0.05)
            if (i+1) % 20 == 0:
                sec = (i+1) / 20
                if node.gz_samples:
                    gz = node.gz_samples[-1]
                    owz = node.odom_wz_samples[-1] if node.odom_wz_samples else 0.0
                    ovx = node.odom_vx_samples[-1] if node.odom_vx_samples else 0.0
                    print(f"  t={sec:.0f}s  gyro_z={gz:+.4f}  odom_wz={owz:+.4f}  odom_vx={ovx:+.4f}")

        node.stop()
        node._sampling = False
        node.spin_for(0.5)

        if node.gz_samples:
            mean_gz = sum(node.gz_samples) / len(node.gz_samples)
            print(f"\n  Gyro-Z samples:  {len(node.gz_samples)}")
            print(f"  Mean gyro-Z:     {mean_gz:+.6f} rad/s  ({math.degrees(mean_gz):+.3f} °/s)")
        if node.odom_wz_samples:
            mean_owz = sum(node.odom_wz_samples) / len(node.odom_wz_samples)
            mean_ovx = sum(node.odom_vx_samples) / len(node.odom_vx_samples)
            print(f"  Odom ω_z mean:   {mean_owz:+.6f} rad/s  (encoder-based rotation)")
            print(f"  Odom v_x mean:   {mean_ovx:+.6f} m/s")
        if node.joint_pos_l:
            dp_l = node.joint_pos_l[-1] - node.joint_pos_l[0]
            dp_r = node.joint_pos_r[-1] - node.joint_pos_r[0]
            print(f"  Joint pos delta: L={dp_l:+.4f} R={dp_r:+.4f} rad")
            dist_l = dp_l * 0.033  # wheel_radius
            dist_r = dp_r * 0.033
            print(f"  Distance:        L={dist_l*100:+.1f} R={dist_r*100:+.1f} cm")
        if node.joint_vel_l:
            mean_vl = sum(node.joint_vel_l) / len(node.joint_vel_l)
            mean_vr = sum(node.joint_vel_r) / len(node.joint_vel_r)
            print(f"  Mean joint vel:  L={mean_vl:+.4f} R={mean_vr:+.4f} rad/s")
            print(f"  Mean wheel speed: L={mean_vl*0.033*1000:.1f} R={mean_vr*0.033*1000:.1f} mm/s")
    else:
        print("\n(Use --drive to also test while driving)")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
