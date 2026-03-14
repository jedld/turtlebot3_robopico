#!/usr/bin/env python3
"""Validate relative turn accuracy using odometry and IMU yaw measurements.

This script commands a sequence of in-place rotations and reports the achieved
yaw change from both /odom and /imu. It can drive the turn controller using
either odometry or IMU feedback so the effect of the chosen feedback source is
visible before changing any calibration constants.
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
from sensor_msgs.msg import Imu

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

RATE_HZ = 20.0
ROTATE_MAX_RAD_S = 0.35
ROTATE_KP = 1.5
HEADING_TOL_DEG = 1.0
SETTLE_SAMPLES = 8


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


@dataclass
class TurnResult:
    trial: int
    repeat: int
    control_source: str
    target_deg: float
    odom_delta_deg: float
    imu_delta_deg: float
    odom_error_deg: float
    imu_error_deg: float
    elapsed_s: float


class RotationVerifier(Node):
    def __init__(self):
        super().__init__("rotation_verifier")
        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._odom: Optional[Odometry] = None
        self._imu: Optional[Imu] = None
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.create_subscription(Imu, "/imu", self._imu_cb, SENSOR_QOS)

    def _odom_cb(self, msg: Odometry):
        self._odom = msg

    def _imu_cb(self, msg: Imu):
        self._imu = msg

    def wait_for_topics(self, timeout: float = 8.0) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._odom is not None and self._imu is not None:
                return True
        return False

    def spin_for(self, secs: float):
        end = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def send(self, ang: float):
        msg = Twist()
        msg.angular.z = ang
        self._pub.publish(msg)

    def stop(self, settle: float = 0.4):
        self.send(0.0)
        self.spin_for(settle)

    @property
    def odom_yaw(self) -> float:
        return quat_to_yaw(self._odom.pose.pose.orientation)

    @property
    def imu_yaw(self) -> float:
        return quat_to_yaw(self._imu.orientation)

    def yaw_for_source(self, source: str) -> float:
        if source == "imu":
            return self.imu_yaw
        return self.odom_yaw

    def rotate_relative(self, delta_rad: float, control_source: str, timeout: float = 20.0) -> tuple[bool, float]:
        start = self.yaw_for_source(control_source)
        target = math.atan2(math.sin(start + delta_rad), math.cos(start + delta_rad))
        settled = 0
        end = time.time() + timeout
        started_at = time.time()
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=1.0 / RATE_HZ)
            current = self.yaw_for_source(control_source)
            err = angle_diff(target, current)
            cmd = max(-ROTATE_MAX_RAD_S, min(ROTATE_MAX_RAD_S, ROTATE_KP * err))
            self.send(cmd)
            if abs(math.degrees(err)) < HEADING_TOL_DEG:
                settled += 1
                if settled >= SETTLE_SAMPLES:
                    self.stop()
                    return True, time.time() - started_at
            else:
                settled = 0
        self.stop()
        return False, time.time() - started_at


def parse_angles(raw: str) -> list[float]:
    values = []
    for item in raw.split(","):
        item = item.strip()
        if not item:
            continue
        values.append(float(item))
    if not values:
        raise ValueError("at least one angle is required")
    return values


def write_csv(path: Path, rows: list[TurnResult]) -> None:
    with path.open("w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(asdict(rows[0]).keys()))
        writer.writeheader()
        for row in rows:
            writer.writerow(asdict(row))


def summarize(rows: list[TurnResult], source: str) -> tuple[float, float]:
    errors = [abs(row.imu_error_deg) for row in rows if row.control_source == source]
    if not errors:
        return float("nan"), float("nan")
    mean_abs = sum(errors) / len(errors)
    worst = max(errors)
    return mean_abs, worst


def stddev(values: list[float]) -> float:
    if len(values) < 2:
        return 0.0
    mean = sum(values) / len(values)
    variance = sum((value - mean) ** 2 for value in values) / (len(values) - 1)
    return math.sqrt(variance)


def print_repeatability(rows: list[TurnResult], source: str) -> None:
    matching = [row for row in rows if row.control_source == source]
    if not matching:
        return
    targets = sorted({row.target_deg for row in matching}, key=lambda value: (abs(value), value))
    print(f"\n{source} control repeatability:")
    for target in targets:
        target_rows = [row for row in matching if row.target_deg == target]
        imu_errors = [row.imu_error_deg for row in target_rows]
        imu_abs_errors = [abs(value) for value in imu_errors]
        imu_deltas = [row.imu_delta_deg for row in target_rows]
        mean_error = sum(imu_errors) / len(imu_errors)
        mean_abs_error = sum(imu_abs_errors) / len(imu_abs_errors)
        repeat_sigma = stddev(imu_deltas)
        worst = max(imu_abs_errors)
        print(
            f"  target={target:+6.1f}°  runs={len(target_rows)}  "
            f"mean={sum(imu_deltas) / len(imu_deltas):+7.2f}°  "
            f"bias={mean_error:+6.2f}°  mean|err|={mean_abs_error:5.2f}°  "
            f"sigma={repeat_sigma:5.2f}°  worst={worst:5.2f}°"
        )


def main() -> int:
    ap = argparse.ArgumentParser(description="Validate relative turn accuracy using odom and IMU")
    ap.add_argument("--angles", default="45,90,180,-90", help="comma-separated target turn angles in degrees")
    ap.add_argument("--control-source", choices=["odom", "imu", "both"], default="both",
                    help="feedback source used to stop each turn")
    ap.add_argument("--repeats", type=int, default=1,
                    help="number of repeated sweeps over the angle list")
    ap.add_argument("--pause", type=float, default=1.0,
                    help="settle time between trials in seconds")
    ap.add_argument("--csv", help="output CSV path")
    args = ap.parse_args()

    angles_deg = parse_angles(args.angles)
    if args.repeats < 1:
        raise SystemExit("--repeats must be at least 1")
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = Path(args.csv) if args.csv else Path(f"rotation_accuracy_{timestamp}.csv")
    control_sources = ["odom", "imu"] if args.control_source == "both" else [args.control_source]

    rclpy.init()
    node = RotationVerifier()
    try:
        print("Waiting for /odom and /imu ...")
        if not node.wait_for_topics():
            print("Timed out waiting for motion topics.")
            return 1

        rows: list[TurnResult] = []
        trial = 0
        for source in control_sources:
            print(f"\nControl source: {source}")
            for repeat in range(1, args.repeats + 1):
                print(f"  Sweep {repeat}/{args.repeats}")
                for angle_deg in angles_deg:
                    trial += 1
                    node.stop(0.6)
                    start_odom = node.odom_yaw
                    start_imu = node.imu_yaw
                    ok, elapsed = node.rotate_relative(math.radians(angle_deg), source)
                    end_odom = node.odom_yaw
                    end_imu = node.imu_yaw
                    odom_delta_deg = math.degrees(angle_diff(end_odom, start_odom))
                    imu_delta_deg = math.degrees(angle_diff(end_imu, start_imu))
                    row = TurnResult(
                        trial=trial,
                        repeat=repeat,
                        control_source=source,
                        target_deg=angle_deg,
                        odom_delta_deg=odom_delta_deg,
                        imu_delta_deg=imu_delta_deg,
                        odom_error_deg=odom_delta_deg - angle_deg,
                        imu_error_deg=imu_delta_deg - angle_deg,
                        elapsed_s=elapsed,
                    )
                    rows.append(row)
                    status = "OK" if ok else "TIMEOUT"
                    print(
                        f"    {status:7s} target={angle_deg:+6.1f}°  "
                        f"odom={odom_delta_deg:+7.2f}°  imu={imu_delta_deg:+7.2f}°  "
                        f"imu_err={row.imu_error_deg:+6.2f}°"
                    )
                    node.stop(args.pause)

        write_csv(csv_path, rows)
        print(f"\nCSV saved: {csv_path}")
        for source in control_sources:
            mean_abs, worst = summarize(rows, source)
            print(f"{source:>4s} control: mean |IMU error|={mean_abs:.2f}°  worst={worst:.2f}°")
            print_repeatability(rows, source)
        return 0
    finally:
        node.stop(0.2)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())