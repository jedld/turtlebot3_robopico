#!/usr/bin/env python3
"""
Guided IMU mount diagnostic for TurtleBot3 Pico.

Collects averaged /imu samples for a set of manual poses and reports whether
the current published robot-frame IMU axes appear correct, swapped, sign-
flipped, or mixed by a non-yaw mount offset.

Usage:
    python3 test_imu_mount_guided.py
    python3 test_imu_mount_guided.py --hold 2.5

Requires:
    - turtlebot3 bringup already running
    - /imu publishing sensor_msgs/Imu
"""

import argparse
import math
import sys
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


RED = "\033[0;31m"
GRN = "\033[0;32m"
YLW = "\033[1;33m"
CYN = "\033[0;36m"
BLD = "\033[1m"
DIM = "\033[2m"
NC = "\033[0m"


def ok(msg: str) -> None:
    print(f"{GRN}[PASS]{NC}  {msg}")


def warn(msg: str) -> None:
    print(f"{YLW}[WARN]{NC}  {msg}")


def fail(msg: str) -> None:
    print(f"{RED}[FAIL]{NC}  {msg}")


def info(msg: str) -> None:
    print(f"{CYN}[INFO]{NC}  {msg}")


def wait_enter(prompt: str) -> None:
    try:
        input(prompt)
    except KeyboardInterrupt:
        print()
        raise


def mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def quat_to_euler_deg(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


@dataclass
class PoseSpec:
    name: str
    instruction: str
    expected_axis: str | None
    expected_sign: int | None


@dataclass
class PoseResult:
    spec: PoseSpec
    accel: tuple[float, float, float]
    euler_deg: tuple[float, float, float]
    delta: tuple[float, float, float]
    dominant_axis: str
    dominant_value: float
    cross_ratio: float


POSES = [
    PoseSpec(
        name="flat",
        instruction="Place the robot flat on the floor in its normal driving orientation.",
        expected_axis=None,
        expected_sign=None,
    ),
    PoseSpec(
        name="nose_up",
        instruction="Lift the FRONT slightly so the nose points upward.",
        expected_axis="x",
        expected_sign=-1,
    ),
    PoseSpec(
        name="nose_down",
        instruction="Tilt the FRONT slightly downward.",
        expected_axis="x",
        expected_sign=1,
    ),
    PoseSpec(
        name="left_down",
        instruction="Tilt the robot so the LEFT wheel goes downward.",
        expected_axis="y",
        expected_sign=-1,
    ),
    PoseSpec(
        name="right_down",
        instruction="Tilt the robot so the RIGHT wheel goes downward.",
        expected_axis="y",
        expected_sign=1,
    ),
]


class MountDiagNode(Node):
    def __init__(self):
        super().__init__("imu_mount_guided")
        self.samples: list[Imu] = []
        self.window_samples: list[Imu] | None = None
        self.create_subscription(Imu, "/imu", self._imu_cb, 20)

    def _imu_cb(self, msg: Imu) -> None:
        self.samples.append(msg)
        if self.window_samples is not None:
            self.window_samples.append(msg)
        if len(self.samples) > 400:
            self.samples = self.samples[-400:]

    def wait_for_imu(self, timeout: float = 5.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.samples:
                return True
        return False

    def collect_window(self, duration: float) -> list[Imu]:
        self.window_samples = []
        end_time = time.time() + duration
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)
        samples = self.window_samples
        self.window_samples = None
        return samples


def summarize_pose(spec: PoseSpec, samples: list[Imu], baseline: tuple[float, float, float]) -> PoseResult:
    ax_vals = [s.linear_acceleration.x for s in samples]
    ay_vals = [s.linear_acceleration.y for s in samples]
    az_vals = [s.linear_acceleration.z for s in samples]
    roll_vals = []
    pitch_vals = []
    yaw_vals = []

    for sample in samples:
        roll, pitch, yaw = quat_to_euler_deg(
            sample.orientation.x,
            sample.orientation.y,
            sample.orientation.z,
            sample.orientation.w,
        )
        roll_vals.append(roll)
        pitch_vals.append(pitch)
        yaw_vals.append(yaw)

    accel = (mean(ax_vals), mean(ay_vals), mean(az_vals))
    euler_deg = (mean(roll_vals), mean(pitch_vals), mean(yaw_vals))
    delta = (
        accel[0] - baseline[0],
        accel[1] - baseline[1],
        accel[2] - baseline[2],
    )

    abs_x = abs(delta[0])
    abs_y = abs(delta[1])
    if abs_x >= abs_y:
        dominant_axis = "x"
        dominant_value = delta[0]
        cross_ratio = abs_y / abs_x if abs_x > 1e-6 else 0.0
    else:
        dominant_axis = "y"
        dominant_value = delta[1]
        cross_ratio = abs_x / abs_y if abs_y > 1e-6 else 0.0

    return PoseResult(
        spec=spec,
        accel=accel,
        euler_deg=euler_deg,
        delta=delta,
        dominant_axis=dominant_axis,
        dominant_value=dominant_value,
        cross_ratio=cross_ratio,
    )


def sign_name(value: float) -> str:
    return "+" if value >= 0.0 else "-"


def print_pose_result(result: PoseResult) -> None:
    ax, ay, az = result.accel
    roll, pitch, yaw = result.euler_deg
    dx, dy, dz = result.delta
    print()
    print(f"{BLD}{result.spec.name}{NC}")
    print(f"  accel avg      x={ax:+.4f}  y={ay:+.4f}  z={az:+.4f}  m/s²")
    print(f"  accel delta    x={dx:+.4f}  y={dy:+.4f}  z={dz:+.4f}  m/s²")
    print(f"  euler avg      roll={roll:+6.2f}°  pitch={pitch:+6.2f}°  yaw={yaw:+6.2f}°")
    print(f"  dominant axis  {result.dominant_axis} ({sign_name(result.dominant_value)}), cross-axis ratio={result.cross_ratio:.2f}")


def analyse_results(results: list[PoseResult]) -> int:
    print()
    print(f"{BLD}── Analysis ───────────────────────────────────────────────{NC}")

    flat = next(result for result in results if result.spec.name == "flat")
    flat_xy = math.hypot(flat.accel[0], flat.accel[1])
    if flat_xy < 0.35:
        ok(f"Flat baseline lateral gravity is small ({flat_xy:.3f} m/s²).")
    else:
        warn(f"Flat baseline lateral gravity is {flat_xy:.3f} m/s² — board or robot may not be level.")

    status = 0
    swapped_count = 0
    sign_fail_count = 0
    mixed_count = 0

    for result in results:
        if result.spec.expected_axis is None:
            continue

        axis_ok = (result.dominant_axis == result.spec.expected_axis)
        sign_ok = ((result.dominant_value >= 0.0) == (result.spec.expected_sign > 0))
        mixed = result.cross_ratio > 0.35

        if axis_ok:
            ok(f"{result.spec.name}: dominant axis is {result.dominant_axis} as expected.")
        else:
            fail(f"{result.spec.name}: dominant axis is {result.dominant_axis}, expected {result.spec.expected_axis}.")
            swapped_count += 1
            status = 1

        if sign_ok:
            ok(f"{result.spec.name}: sign {sign_name(result.dominant_value)} matches expectation.")
        else:
            fail(f"{result.spec.name}: sign {sign_name(result.dominant_value)} is opposite of expectation.")
            sign_fail_count += 1
            status = 1

        if mixed:
            warn(f"{result.spec.name}: strong cross-axis mixing ({result.cross_ratio:.2f}) suggests the board is not only yaw-rotated.")
            mixed_count += 1

    print()
    if swapped_count >= 2:
        warn("Observed behavior is consistent with X/Y being swapped in the published robot frame.")
    if sign_fail_count >= 2:
        warn("Observed behavior is consistent with at least one axis sign being flipped.")
    if mixed_count >= 2:
        warn("Observed behavior is consistent with a fixed 3D mount tilt; yaw-only remap is probably insufficient.")
    if swapped_count == 0 and sign_fail_count == 0 and mixed_count == 0 and flat_xy < 0.35:
        ok("The guided tilt results look consistent with the expected robot-frame IMU axes.")

    print()
    print(f"{BLD}Interpretation guide{NC}")
    print("  nose_up / nose_down should mainly move accel X")
    print("  left_down / right_down should mainly move accel Y")
    print("  persistent large cross-axis motion indicates the board is mounted with real tilt, not only a 90° yaw rotation")

    return status


def main() -> int:
    parser = argparse.ArgumentParser(description="Guided IMU mount-axis diagnostic")
    parser.add_argument("--hold", type=float, default=2.0,
                        help="seconds of IMU data to collect for each pose (default: 2.0)")
    parser.add_argument("--settle", type=float, default=0.75,
                        help="seconds to wait after each prompt before sampling (default: 0.75)")
    parser.add_argument("--min-samples", type=int, default=15,
                        help="minimum IMU samples required per pose (default: 15)")
    args = parser.parse_args()

    print()
    print(f"{BLD}============================================================{NC}")
    print(f"{BLD}  Guided IMU Mount Diagnostic{NC}")
    print(f"{BLD}============================================================{NC}")
    print()
    print("This script will guide you through several robot poses and infer whether")
    print("the published /imu axes are correct, swapped, sign-flipped, or mixed.")

    rclpy.init()
    node = MountDiagNode()

    try:
        info("Waiting for /imu ...")
        if not node.wait_for_imu():
            fail("No /imu data received. Is bringup running?")
            return 1

        results: list[PoseResult] = []
        baseline = (0.0, 0.0, 0.0)

        for index, spec in enumerate(POSES, start=1):
            print()
            print(f"{BLD}Pose {index}/{len(POSES)}: {spec.name}{NC}")
            print(f"  {spec.instruction}")
            wait_enter("  Press Enter when ready ... ")
            info(f"Settling for {args.settle:.2f} s ...")
            node.collect_window(args.settle)
            info(f"Collecting IMU for {args.hold:.2f} s ...")
            samples = node.collect_window(args.hold)
            if len(samples) < args.min_samples:
                fail(f"Only {len(samples)} samples collected for {spec.name}; need at least {args.min_samples}.")
                return 1

            if spec.name == "flat":
                baseline = (
                    mean([sample.linear_acceleration.x for sample in samples]),
                    mean([sample.linear_acceleration.y for sample in samples]),
                    mean([sample.linear_acceleration.z for sample in samples]),
                )

            result = summarize_pose(spec, samples, baseline)
            results.append(result)
            print_pose_result(result)

        return analyse_results(results)
    except KeyboardInterrupt:
        print()
        warn("Interrupted by user.")
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())