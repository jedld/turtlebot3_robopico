#!/usr/bin/env python3
"""
Calibration test: drive the robot forward exactly 10 cm while recording
IMU orientation, odometry, and (if available) lidar scan data to detect
straight-line drift and diagnose motor/wheel parameter mismatches.

Usage:  python3 test_calibration.py [--distance 0.10] [--speed 0.04]

After the run, prints:
  - Odometry displacement (X, Y) and heading change
  - IMU yaw drift during the run
  - Lateral deviation (Y displacement = drift from straight line)
  - Heading change (should be ~0 for a perfectly balanced robot)
  - Suggested parameter corrections if drift is significant

Requires: ROS 2 bringup already running.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
import time, math, sys, argparse


class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_tester')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Data storage
        self.odom_samples = []
        self.imu_samples = []
        self.scan_samples = []

        self.latest_odom = None
        self.latest_imu = None

        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(Imu, '/imu', self._imu_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)

    def _odom_cb(self, msg):
        self.latest_odom = msg
        self.odom_samples.append({
            'time': time.time(),
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'yaw': self._quat_to_yaw(msg.pose.pose.orientation),
            'vx': msg.twist.twist.linear.x,
            'wz': msg.twist.twist.angular.z,
        })

    def _imu_cb(self, msg):
        self.latest_imu = msg
        o = msg.orientation
        yaw = self._quat_to_yaw(o)
        self.imu_samples.append({
            'time': time.time(),
            'yaw': yaw,
            'gz': msg.angular_velocity.z,
            'ax': msg.linear_acceleration.x,
            'ay': msg.linear_acceleration.y,
            'az': msg.linear_acceleration.z,
            'qw': o.w, 'qx': o.x, 'qy': o.y, 'qz': o.z,
        })

    def _scan_cb(self, msg):
        # Store a summary: front-facing range and ranges at ±90°
        n = len(msg.ranges)
        if n == 0:
            return
        front_idx = 0
        left_idx = n // 4
        right_idx = 3 * n // 4

        def safe_range(idx):
            r = msg.ranges[idx % n]
            if r < msg.range_min or r > msg.range_max:
                return float('nan')
            return r

        self.scan_samples.append({
            'time': time.time(),
            'front': safe_range(front_idx),
            'left': safe_range(left_idx),
            'right': safe_range(right_idx),
        })

    @staticmethod
    def _quat_to_yaw(o):
        siny = 2.0 * (o.w * o.z + o.x * o.y)
        cosy = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        return math.atan2(siny, cosy)

    def wait_for_data(self, timeout=8.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_odom is not None:
                return True
        return False

    def send_vel(self, lin_x=0.0, ang_z=0.0):
        msg = Twist()
        msg.linear.x = lin_x
        msg.angular.z = ang_z
        self.pub.publish(msg)

    def drive_distance(self, target_dist, speed, rate_hz=20):
        """
        Drive forward at `speed` m/s until odometry reports `target_dist`
        metres of travel, then stop.  Records all sensor data during the run.
        """
        # Record start pose
        start_x = self.latest_odom.pose.pose.position.x
        start_y = self.latest_odom.pose.pose.position.y

        # Clear sample buffers
        self.odom_samples.clear()
        self.imu_samples.clear()
        self.scan_samples.clear()

        dt = 1.0 / rate_hz
        t_start = time.time()
        max_time = target_dist / speed * 3.0 + 5.0  # generous timeout

        while True:
            # Compute distance travelled
            dx = self.latest_odom.pose.pose.position.x - start_x
            dy = self.latest_odom.pose.pose.position.y - start_y
            dist = math.sqrt(dx * dx + dy * dy)

            if dist >= target_dist:
                break
            if time.time() - t_start > max_time:
                print(f"  WARNING: timed out after {max_time:.1f}s, "
                      f"only {dist:.4f}m travelled")
                break

            self.send_vel(lin_x=speed)
            rclpy.spin_once(self, timeout_sec=dt)

        # Stop
        self.send_vel(0.0, 0.0)

        # Let data settle
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)

        return time.time() - t_start


def analyse(node, target_dist, drive_time):
    """Analyse the recorded data and print calibration diagnostics."""
    odom = node.odom_samples
    imu = node.imu_samples

    if len(odom) < 2:
        print("ERROR: not enough odometry samples!")
        return

    # --- Odometry analysis ---
    o0, of = odom[0], odom[-1]
    dx = of['x'] - o0['x']
    dy = of['y'] - o0['y']
    total_dist = math.sqrt(dx * dx + dy * dy)
    odom_yaw_change = of['yaw'] - o0['yaw']
    # Normalize to [-pi, pi]
    while odom_yaw_change > math.pi:
        odom_yaw_change -= 2 * math.pi
    while odom_yaw_change < -math.pi:
        odom_yaw_change += 2 * math.pi

    # Compute displacement in the robot's INITIAL body frame.
    # This removes the effect of the robot starting at a non-zero odom heading.
    start_yaw = o0['yaw']
    cos_y = math.cos(-start_yaw)
    sin_y = math.sin(-start_yaw)
    fwd_dist = dx * cos_y - dy * sin_y    # along initial heading (forward)
    lat_dist = dx * sin_y + dy * cos_y    # perpendicular (positive = left)

    # Also compute per-sample lateral deviation from the ideal straight path
    # (the line from start in the direction of initial heading).
    max_lat_dev = 0.0
    for s in odom:
        sdx = s['x'] - o0['x']
        sdy = s['y'] - o0['y']
        slat = sdx * sin_y + sdy * cos_y
        if abs(slat) > abs(max_lat_dev):
            max_lat_dev = slat

    print("\n" + "=" * 60)
    print("  CALIBRATION RESULTS: 10 cm STRAIGHT-LINE TEST")
    print("=" * 60)

    print(f"\n--- Odometry ---")
    print(f"  Target distance   : {target_dist * 100:.1f} cm")
    print(f"  Measured distance : {total_dist * 100:.2f} cm  "
          f"(ΔX={dx * 100:.2f} cm, ΔY={dy * 100:.2f} cm)")
    print(f"  Distance error    : {(total_dist - target_dist) * 100:.2f} cm  "
          f"({(total_dist - target_dist) / target_dist * 100:.1f}%)")
    print(f"  Start heading     : {math.degrees(start_yaw):.2f}°  "
          f"(odom frame; robot was pre-rotated)")
    print(f"  Forward travel    : {fwd_dist * 100:.2f} cm  "
          f"(along initial heading)")
    print(f"  Lateral drift     : {lat_dist * 100:.2f} cm  "
          f"({'LEFT' if lat_dist > 0 else 'RIGHT'}, "
          f"{abs(lat_dist) / target_dist * 100:.1f}% of distance)")
    print(f"  Max lateral dev.  : {max_lat_dev * 100:.2f} cm")
    print(f"  Heading change    : {math.degrees(odom_yaw_change):.3f}°  "
          f"(during run)")
    print(f"  Drive time        : {drive_time:.2f} s")
    print(f"  Avg speed         : {total_dist / drive_time * 100:.2f} cm/s")
    print(f"  Samples collected : {len(odom)} odom, {len(imu)} IMU")

    # --- IMU analysis ---
    if len(imu) >= 2:
        i0, ifin = imu[0], imu[-1]
        imu_yaw_change = ifin['yaw'] - i0['yaw']
        while imu_yaw_change > math.pi:
            imu_yaw_change -= 2 * math.pi
        while imu_yaw_change < -math.pi:
            imu_yaw_change += 2 * math.pi

        # Compute max angular velocity during the run
        max_gz = max(abs(s['gz']) for s in imu)
        avg_gz = sum(s['gz'] for s in imu) / len(imu)

        print(f"\n--- IMU ---")
        print(f"  Yaw change (IMU)  : {math.degrees(imu_yaw_change):.3f}°")
        print(f"  Avg gyro Z        : {math.degrees(avg_gz):.3f}°/s")
        print(f"  Max |gyro Z|      : {math.degrees(max_gz):.3f}°/s")
        print(f"  Quaternion final  : w={ifin['qw']:.4f} x={ifin['qx']:.4f} "
              f"y={ifin['qy']:.4f} z={ifin['qz']:.4f}")
    else:
        print(f"\n--- IMU ---")
        print(f"  No IMU data received.")

    # --- Lidar analysis ---
    scans = node.scan_samples
    if len(scans) >= 2:
        s0, sf = scans[0], scans[-1]
        print(f"\n--- LiDAR ---")
        print(f"  Samples           : {len(scans)}")
        print(f"  Front range start : {s0['front']:.3f} m  →  end: {sf['front']:.3f} m  "
              f"(Δ={sf['front'] - s0['front']:.3f} m)")
        print(f"  Left range  start : {s0['left']:.3f} m  →  end: {sf['left']:.3f} m  "
              f"(Δ={sf['left'] - s0['left']:.3f} m)")
        print(f"  Right range start : {s0['right']:.3f} m  →  end: {sf['right']:.3f} m  "
              f"(Δ={sf['right'] - s0['right']:.3f} m)")

        # Left-right symmetry check
        left_drift = sf['left'] - s0['left']
        right_drift = sf['right'] - s0['right']
        if not (math.isnan(left_drift) or math.isnan(right_drift)):
            asym = left_drift - right_drift
            print(f"  L-R asymmetry     : {asym:.3f} m  "
                  f"({'drifting LEFT' if asym > 0.005 else 'drifting RIGHT' if asym < -0.005 else 'centered'})")
    else:
        print(f"\n--- LiDAR ---")
        print(f"  No scan data received (lidar may not be connected).")

    # --- Diagnostic interpretations ---
    print(f"\n--- DIAGNOSIS ---")
    issues = []

    # Check distance accuracy (wheel radius calibration)
    if total_dist < 0.001:
        print(f"  ⚠ Robot did not move! MOTOR_MIN_DUTY may be below stall point.")
        print(f"    → Increase MOTOR_MIN_DUTY in firmware/main.c")
        print("\n" + "=" * 60)
        return

    dist_err_pct = (fwd_dist - target_dist) / target_dist * 100.0
    if abs(dist_err_pct) > 5.0:
        direction = "overshooting" if dist_err_pct > 0 else "undershooting"
        # If robot travels MORE than expected → actual wheel is LARGER than configured
        # If robot travels LESS than expected → actual wheel is SMALLER than configured
        current_r = 0.033  # from firmware
        corrected_r = current_r * (target_dist / fwd_dist) if fwd_dist > 0.001 else current_r
        issues.append(
            f"Distance off by {dist_err_pct:+.1f}% ({direction}).\n"
            f"    → Adjust WHEEL_RADIUS: {current_r:.4f} → {corrected_r:.4f} m\n"
            f"    → In firmware/main.c AND config/burger_pico.yaml"
        )
    else:
        print(f"  ✓ Distance accuracy is good ({dist_err_pct:+.1f}%, within ±5%)")

    # Check lateral drift (motor imbalance / wheel separation)
    # Use body-frame lateral displacement, not raw odom Y
    lat_drift_pct = abs(lat_dist) / target_dist * 100.0
    if lat_drift_pct > 3.0:
        drift_dir = "LEFT" if lat_dist > 0 else "RIGHT"
        # Drift = one motor stronger than the other.
        # Drift RIGHT → left motor is faster → increase right motor duty or
        # reduce left. In practice: adjust MOTOR_MIN_DUTY per-wheel or
        # check wheel separation.
        issues.append(
            f"Lateral drift of {lat_dist * 100:.2f} cm ({drift_dir}, {lat_drift_pct:.1f}% of distance).\n"
            f"    → This indicates uneven motor output. Possible fixes:\n"
            f"      1. Verify wheel separation measurement (currently 0.135 m)\n"
            f"      2. Check for mechanical drag (one wheel binding)\n"
            f"      3. Consider per-motor duty trim in firmware"
        )
    else:
        print(f"  ✓ Lateral drift is minimal ({lat_dist * 100:.2f} cm, {lat_drift_pct:.1f}%)")

    # Check heading change
    heading_deg = abs(math.degrees(odom_yaw_change))
    if heading_deg > 2.0:
        turn_dir = "LEFT (CCW)" if odom_yaw_change > 0 else "RIGHT (CW)"
        # Heading drift indicates differential wheel speed mismatch
        current_sep = 0.135
        # If robot turns LEFT → right wheel faster → effective separation is smaller
        # If robot turns RIGHT → left wheel faster → effective separation is larger
        corrected_sep = current_sep * (1.0 + odom_yaw_change * current_sep / (2 * total_dist))
        issues.append(
            f"Heading drifted {heading_deg:.2f}° {turn_dir} over {total_dist*100:.1f} cm.\n"
            f"    → Adjust WHEEL_SEPARATION: {current_sep:.4f} → {corrected_sep:.4f} m\n"
            f"    → In firmware/main.c AND config/burger_pico.yaml"
        )
    else:
        print(f"  ✓ Heading stability is good ({heading_deg:.2f}° drift)")

    if issues:
        print(f"\n  ⚠ ISSUES FOUND ({len(issues)}):")
        for i, issue in enumerate(issues, 1):
            print(f"\n  {i}. {issue}")
    else:
        print(f"\n  ✅ All parameters look well-calibrated!")

    print("\n" + "=" * 60)


def main():
    parser = argparse.ArgumentParser(description='Motor/wheel calibration test')
    parser.add_argument('--distance', type=float, default=0.10,
                        help='Target distance in metres (default: 0.10)')
    parser.add_argument('--speed', type=float, default=0.04,
                        help='Forward speed in m/s (default: 0.04)')
    args = parser.parse_args()

    rclpy.init()
    node = CalibrationNode()

    print(f"Motor/Wheel Calibration Test")
    print(f"  Target distance: {args.distance * 100:.1f} cm")
    print(f"  Speed: {args.speed * 100:.1f} cm/s")

    print("\nWaiting for odometry data...")
    if not node.wait_for_data(8.0):
        print("ERROR: no odometry received. Is bringup running?")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # Let IMU settle for a moment
    print("Letting sensors settle (2s)...")
    t0 = time.time()
    while time.time() - t0 < 2.0:
        rclpy.spin_once(node, timeout_sec=0.05)

    print(f"\nDriving forward {args.distance * 100:.1f} cm at {args.speed * 100:.1f} cm/s...")
    drive_time = node.drive_distance(args.distance, args.speed)

    # Stop and collect final samples
    print("Stopped. Collecting final data...")
    t0 = time.time()
    while time.time() - t0 < 1.0:
        rclpy.spin_once(node, timeout_sec=0.05)

    analyse(node, args.distance, drive_time)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
