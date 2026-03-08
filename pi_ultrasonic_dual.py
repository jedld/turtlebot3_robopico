#!/usr/bin/env python3
"""
Read two Grove Ultrasonic single-wire sensors connected directly to Raspberry Pi GPIO.

Uses RPi.GPIO (python3-rpi-lgpio on Ubuntu) for daemon-free operation.

Example:
  python3 pi_ultrasonic_dual.py --left-gpio 23 --right-gpio 24
"""

import argparse
import sys
import time


def read_distance_mm(gpio_mod, gpio: int, timeout_us: int = 30000):
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


def fmt(dist_mm, err):
    if err:
        return err
    return f"{dist_mm:7.1f} mm ({dist_mm / 10.0:6.1f} cm)"


def main():
    parser = argparse.ArgumentParser(description="Dual Grove ultrasonic test on Raspberry Pi")
    parser.add_argument("--left-gpio", type=int, default=23, help="BCM GPIO for left sensor SIG (default: 23)")
    parser.add_argument("--right-gpio", type=int, default=24, help="BCM GPIO for right sensor SIG (default: 24)")
    parser.add_argument("--interval", type=float, default=0.2, help="seconds between loops (default: 0.2)")
    parser.add_argument("--inter-sensor-gap", type=float, default=0.05, help="seconds between left/right trigger (default: 0.05)")
    parser.add_argument("--timeout-us", type=int, default=30000, help="echo timeout in microseconds (default: 30000)")
    parser.add_argument("--count", type=int, default=0, help="number of loops; 0 = run forever")
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

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    print(
        f"Reading dual ultrasonic: left=GPIO{args.left_gpio}, right=GPIO{args.right_gpio} "
        f"(Ctrl+C to stop)"
    )

    loop_idx = 0
    try:
        while True:
            loop_idx += 1

            left_mm, left_err = read_distance_mm(GPIO, args.left_gpio, args.timeout_us)
            time.sleep(max(0.0, args.inter_sensor_gap))
            right_mm, right_err = read_distance_mm(GPIO, args.right_gpio, args.timeout_us)

            print(
                f"{loop_idx:5d}: "
                f"L[{fmt(left_mm, left_err)}]  "
                f"R[{fmt(right_mm, right_err)}]"
            )

            if args.count > 0 and loop_idx >= args.count:
                break
            time.sleep(max(0.0, args.interval))
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup(args.left_gpio)
        GPIO.cleanup(args.right_gpio)


if __name__ == "__main__":
    main()
