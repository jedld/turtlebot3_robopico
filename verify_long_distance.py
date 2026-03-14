#!/usr/bin/env python3
"""
verify_long_distance.py — LiDAR-aware long-distance straight-line verification.

This script does not assume the robot is already in a safe place.
Before driving, it uses the LiDAR scan to:
  1. find a heading with enough forward clearance for the requested run,
  2. require adequate side clearance so the path is not a narrow obstacle slot,
  3. rotate the robot to that heading,
  4. re-check the corridor and only then perform the drive.

During the run it monitors front clearance continuously and aborts if an
obstacle enters the safety envelope.

Outputs:
  - console summary of chosen heading and drive result
  - CSV with odom, IMU, and LiDAR preflight metrics

Requires: turtlebot3-bringup.service running (/scan, /odom, /imu, /cmd_vel).
"""

from __future__ import annotations

import argparse
import csv
import math
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

RED = "\033[0;31m"
GRN = "\033[0;32m"
YLW = "\033[1;33m"
CYN = "\033[0;36m"
BLD = "\033[1m"
NC = "\033[0m"

RATE_HZ = 20.0
STOP_MARGIN_M = 0.50
SIDE_SAMPLE_ANGLE_DEG = 25.0
SIDE_CLEAR_MIN_M = 0.30
SEARCH_STEP_DEG = 4.0
HEADING_TOL_DEG = 1.0
ROTATE_MAX_RAD_S = 0.35
ROTATE_KP = 1.2
ROTATE_SETTLE_SAMPLES = 8
DRIVE_STOP_FRONT_M = 0.28
DRIVE_STOP_CONFIRM_SAMPLES = 3
FORWARD_WINDOW_DEG = 8.0
SIDE_WINDOW_DEG = 6.0
MAX_EXTRA_DRIVE_TIME_S = 8.0


@dataclass
class DriveSample:
    t: float
    phase: str
    odom_x: float
    odom_y: float
    odom_yaw_deg: float
    imu_yaw_deg: float
    cmd_lin: float
    cmd_ang: float
    front_clearance_m: float
    left_clearance_m: float
    right_clearance_m: float
    target_heading_deg: float


@dataclass
class CorridorChoice:
    heading_rad: float
    heading_deg: float
    forward_clearance_m: float
    left_clearance_m: float
    right_clearance_m: float
    score: float


def angle_diff(a: float, b: float) -> float:
    delta = a - b
    while delta > math.pi:
        delta -= 2.0 * math.pi
    while delta < -math.pi:
        delta += 2.0 * math.pi
    return delta


def quat_to_yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class LongRunVerifier(Node):
    def __init__(self):
        super().__init__("long_run_verifier")
        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._scan: Optional[LaserScan] = None
        self._odom: Optional[Odometry] = None
        self._imu: Optional[Imu] = None
        self.create_subscription(LaserScan, "/scan", self._scan_cb, SENSOR_QOS)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.create_subscription(Imu, "/imu", self._imu_cb, SENSOR_QOS)

    def _scan_cb(self, msg: LaserScan):
        self._scan = msg

    def _odom_cb(self, msg: Odometry):
        self._odom = msg

    def _imu_cb(self, msg: Imu):
        self._imu = msg

    def spin_for(self, secs: float):
        end = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_topics(self, timeout: float = 8.0) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._scan is not None and self._odom is not None and self._imu is not None:
                return True
        return False

    def send(self, lin: float = 0.0, ang: float = 0.0):
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self._pub.publish(msg)

    def stop(self, settle: float = 0.5):
        self.send(0.0, 0.0)
        self.spin_for(settle)

    @property
    def odom_xy(self) -> tuple[float, float]:
        pose = self._odom.pose.pose.position
        return pose.x, pose.y

    @property
    def odom_yaw(self) -> float:
        return quat_to_yaw(self._odom.pose.pose.orientation)

    @property
    def imu_yaw(self) -> float:
        return quat_to_yaw(self._imu.orientation)

    def range_at_angle(self, angle_rad: float, scan: Optional[LaserScan] = None) -> Optional[float]:
        scan = scan or self._scan
        if scan is None or not scan.ranges:
            return None
        angle = math.atan2(math.sin(angle_rad), math.cos(angle_rad))
        idx = int(round((angle - scan.angle_min) / scan.angle_increment))
        if idx < 0 or idx >= len(scan.ranges):
            return None
        radius = scan.ranges[idx]
        if not math.isfinite(radius) or radius < scan.range_min or radius > scan.range_max:
            return None
        return radius

    def min_range_in_window(self, center_rad: float, half_width_deg: float,
                            scan: Optional[LaserScan] = None) -> Optional[float]:
        scan = scan or self._scan
        if scan is None or not scan.ranges:
            return None
        half_width = math.radians(half_width_deg)
        valid: list[float] = []
        for idx, radius in enumerate(scan.ranges):
            if not math.isfinite(radius) or radius < scan.range_min or radius > scan.range_max:
                continue
            angle = scan.angle_min + idx * scan.angle_increment
            if abs(angle_diff(angle, center_rad)) <= half_width:
                valid.append(radius)
        return min(valid) if valid else None

    def corridor_choice(self, run_distance_m: float) -> Optional[CorridorChoice]:
        scan = self._scan
        if scan is None or not scan.ranges:
            return None

        needed_forward = run_distance_m + STOP_MARGIN_M
        best: Optional[CorridorChoice] = None
        step = math.radians(SEARCH_STEP_DEG)
        side_angle = math.radians(SIDE_SAMPLE_ANGLE_DEG)
        search_angles = []
        angle = -math.pi
        while angle < math.pi:
            search_angles.append(angle)
            angle += step

        for heading in search_angles:
            forward = self.min_range_in_window(heading, FORWARD_WINDOW_DEG, scan)
            left = self.min_range_in_window(heading + side_angle, SIDE_WINDOW_DEG, scan)
            right = self.min_range_in_window(heading - side_angle, SIDE_WINDOW_DEG, scan)
            if forward is None or left is None or right is None:
                continue
            if forward < needed_forward:
                continue
            if min(left, right) < SIDE_CLEAR_MIN_M:
                continue
            center_bonus = -0.15 * abs(math.degrees(heading))
            side_bonus = 0.35 * min(left, right)
            score = forward + side_bonus + center_bonus
            candidate = CorridorChoice(
                heading_rad=heading,
                heading_deg=math.degrees(heading),
                forward_clearance_m=forward,
                left_clearance_m=left,
                right_clearance_m=right,
                score=score,
            )
            if best is None or candidate.score > best.score:
                best = candidate
        return best

    def rotate_relative(self, delta_rad: float, timeout: float = 20.0) -> bool:
        start = self.imu_yaw
        target = start + delta_rad
        target = math.atan2(math.sin(target), math.cos(target))
        settled = 0
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            err = angle_diff(target, self.imu_yaw)
            cmd = max(-ROTATE_MAX_RAD_S, min(ROTATE_MAX_RAD_S, ROTATE_KP * err))
            self.send(0.0, cmd)
            if abs(math.degrees(err)) < HEADING_TOL_DEG:
                settled += 1
                if settled >= ROTATE_SETTLE_SAMPLES:
                    self.stop(0.3)
                    return True
            else:
                settled = 0
        self.stop(0.3)
        return False


def write_csv(path: Path, samples: list[DriveSample]) -> None:
    with path.open("w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(asdict(samples[0]).keys()))
        writer.writeheader()
        for sample in samples:
            writer.writerow(asdict(sample))


def main() -> int:
    ap = argparse.ArgumentParser(description="LiDAR-aware long-distance straight-line verifier")
    ap.add_argument("--distance", type=float, default=2.0, help="target straight distance in metres")
    ap.add_argument("--speed", type=float, default=0.10, help="forward speed in m/s")
    ap.add_argument("--csv", help="output CSV path")
    args = ap.parse_args()

    rclpy.init()
    node = LongRunVerifier()
    samples: list[DriveSample] = []

    try:
        print(f"{BLD}Waiting for /scan, /odom, /imu ...{NC}")
        if not node.wait_for_topics():
            print(f"{RED}Required ROS topics not available. Is bringup running?{NC}")
            return 1

        node.stop(0.5)
        choice = node.corridor_choice(args.distance)
        if choice is None:
            print(f"{RED}No LiDAR corridor is wide and long enough for a {args.distance:.2f} m run.{NC}")
            print(f"{YLW}Move the robot to a more open area and retry.{NC}")
            return 2

        print(f"{CYN}Chosen heading {choice.heading_deg:+.1f}°  forward={choice.forward_clearance_m:.2f} m  left={choice.left_clearance_m:.2f} m  right={choice.right_clearance_m:.2f} m{NC}")
        if abs(choice.heading_deg) > 2.0:
            print(f"{CYN}Rotating into the selected corridor...{NC}")
            if not node.rotate_relative(choice.heading_rad):
                print(f"{RED}Failed to align to the selected LiDAR corridor.{NC}")
                return 3

        node.spin_for(0.8)
        confirm = node.corridor_choice(args.distance)
        if confirm is None or confirm.forward_clearance_m < args.distance + STOP_MARGIN_M:
            print(f"{RED}The corridor is no longer safe after alignment. Aborting.{NC}")
            return 4

        start_x, start_y = node.odom_xy
        start_yaw = node.odom_yaw
        nominal_duration = args.distance / max(args.speed, 1e-6)
        end = time.time() + nominal_duration + MAX_EXTRA_DRIVE_TIME_S
        target_heading_deg = math.degrees(start_yaw)
        print(f"{BLD}Driving {args.distance:.2f} m at {args.speed:.2f} m/s in LiDAR-cleared space...{NC}")

        unsafe_count = 0
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=1.0 / RATE_HZ)
            cur_x, cur_y = node.odom_xy
            dx = cur_x - start_x
            dy = cur_y - start_y
            along_track = math.cos(start_yaw) * dx + math.sin(start_yaw) * dy
            front = node.min_range_in_window(0.0, FORWARD_WINDOW_DEG)
            left = node.min_range_in_window(math.radians(SIDE_SAMPLE_ANGLE_DEG), SIDE_WINDOW_DEG)
            right = node.min_range_in_window(math.radians(-SIDE_SAMPLE_ANGLE_DEG), SIDE_WINDOW_DEG)
            if front is not None and front < DRIVE_STOP_FRONT_M:
                unsafe_count += 1
            else:
                unsafe_count = 0
            if unsafe_count >= DRIVE_STOP_CONFIRM_SAMPLES:
                print(f"{RED}Obstacle entered safety stop envelope at {front:.2f} m. Stopping.{NC}")
                node.stop(0.2)
                break
            if along_track >= args.distance:
                node.stop(0.2)
                break
            yaw_err = angle_diff(start_yaw, node.odom_yaw)
            ang = max(-0.20, min(0.20, 1.0 * yaw_err))
            node.send(args.speed, ang)
            samples.append(DriveSample(
                t=time.time(),
                phase="drive",
                odom_x=node.odom_xy[0],
                odom_y=node.odom_xy[1],
                odom_yaw_deg=math.degrees(node.odom_yaw),
                imu_yaw_deg=math.degrees(node.imu_yaw),
                cmd_lin=args.speed,
                cmd_ang=ang,
                front_clearance_m=front if front is not None else float("nan"),
                left_clearance_m=left if left is not None else float("nan"),
                right_clearance_m=right if right is not None else float("nan"),
                target_heading_deg=target_heading_deg,
            ))

        node.stop(0.5)
        end_x, end_y = node.odom_xy
        dx = end_x - start_x
        dy = end_y - start_y
        travel = math.hypot(dx, dy)
        lateral = -math.sin(start_yaw) * dx + math.cos(start_yaw) * dy
        heading_err_deg = math.degrees(angle_diff(node.odom_yaw, start_yaw))

        print(f"\n{BLD}Result{NC}")
        print(f"  Travelled:      {travel:.3f} m")
        print(f"  Lateral offset: {lateral*100:.1f} cm")
        print(f"  Heading error:  {heading_err_deg:+.2f}°")

        if samples:
            csv_path = Path(args.csv) if args.csv else Path(__file__).resolve().parent / f"verify_long_distance_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            write_csv(csv_path, samples)
            print(f"  CSV saved:      {csv_path}")
        return 0
    finally:
        node.stop(0.2)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
