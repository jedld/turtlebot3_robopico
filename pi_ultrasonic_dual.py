#!/usr/bin/env python3
"""
Read two Grove Ultrasonic single-wire sensors connected directly to Raspberry Pi GPIO.

Uses lgpio directly (the kernel-level GPIO library) for reliable pin lifecycle
management.  Avoids RPi.GPIO's rpi-lgpio wrapper which cannot re-claim pins
across runs that crashed without cleanup.

Example:
  python3 pi_ultrasonic_dual.py --left-gpio 23 --right-gpio 24
"""

import argparse
import sys
import time

import lgpio


def read_distance_mm(h: int, gpio: int, timeout_us: int = 30000):
    """Single-wire trigger + echo using lgpio handle *h*."""
    # -- Trigger: pull low, pulse high 10 µs, pull low --
    lgpio.gpio_claim_output(h, gpio, 0)
    time.sleep(2e-6)
    lgpio.gpio_write(h, gpio, 1)
    time.sleep(10e-6)
    lgpio.gpio_write(h, gpio, 0)
    lgpio.gpio_free(h, gpio)

    # -- Echo: switch to input and time the return pulse --
    lgpio.gpio_claim_input(h, gpio)

    start_ns = time.perf_counter_ns()
    timeout_ns = int(timeout_us * 1000)
    while lgpio.gpio_read(h, gpio) == 0:
        if (time.perf_counter_ns() - start_ns) > timeout_ns:
            lgpio.gpio_free(h, gpio)
            return None, "wait-timeout"

    rise_ns = time.perf_counter_ns()
    while lgpio.gpio_read(h, gpio) == 1:
        if (time.perf_counter_ns() - rise_ns) > timeout_ns:
            lgpio.gpio_free(h, gpio)
            return None, "pulse-timeout"

    fall_ns = time.perf_counter_ns()
    lgpio.gpio_free(h, gpio)

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
    parser.add_argument("--gpiochip", type=int, default=None,
                        help="gpiochip device number (default: auto-detect; 4 on Pi 5, 0 on Pi 4)")
    args = parser.parse_args()

    if args.left_gpio == args.right_gpio:
        print("ERROR: --left-gpio and --right-gpio must be different")
        sys.exit(2)

    # Auto-detect gpiochip: Pi 5 uses gpiochip4 (RP1), Pi 4 and earlier use 0.
    if args.gpiochip is not None:
        chip = args.gpiochip
    else:
        import os
        if os.path.exists("/dev/gpiochip4"):
            chip = 4   # Raspberry Pi 5
        else:
            chip = 0   # Raspberry Pi 4 / earlier

    try:
        h = lgpio.gpiochip_open(chip)
    except lgpio.error as e:
        print(f"ERROR: cannot open gpiochip{chip}: {e}")
        print("Run with sudo, or add your user to the gpio/dialout group.")
        print("You can also specify --gpiochip N manually.")
        sys.exit(2)

    # Ensure target GPIOs are free on this handle (safe even if not claimed).
    for g in (args.left_gpio, args.right_gpio):
        try:
            lgpio.gpio_free(h, g)
        except lgpio.error:
            pass

    # Verify we can actually claim the GPIOs before entering the main loop.
    # If another process holds them, give a clear diagnostic instead of crashing.
    for g in (args.left_gpio, args.right_gpio):
        try:
            lgpio.gpio_claim_input(h, g)
            lgpio.gpio_free(h, g)
        except lgpio.error:
            # Find the offending process
            import subprocess
            holder = ""
            try:
                out = subprocess.check_output(
                    ["fuser", f"/dev/gpiochip{chip}"],
                    stderr=subprocess.STDOUT, text=True
                ).strip()
                if out:
                    holder = f"\n  PIDs using /dev/gpiochip{chip}: {out}"
                    holder += "\n  Kill them first, e.g.:  kill " + out.split()[-1]
            except Exception:
                pass
            print(
                f"ERROR: GPIO {g} is busy — another process already has it claimed.{holder}"
            )
            lgpio.gpiochip_close(h)
            sys.exit(1)

    print(
        f"Reading dual ultrasonic: left=GPIO{args.left_gpio}, right=GPIO{args.right_gpio} "
        f"(Ctrl+C to stop)"
    )

    loop_idx = 0
    try:
        while True:
            loop_idx += 1

            left_mm, left_err = read_distance_mm(h, args.left_gpio, args.timeout_us)
            time.sleep(max(0.0, args.inter_sensor_gap))
            right_mm, right_err = read_distance_mm(h, args.right_gpio, args.timeout_us)

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
        for g in (args.left_gpio, args.right_gpio):
            try:
                lgpio.gpio_free(h, g)
            except lgpio.error:
                pass
        lgpio.gpiochip_close(h)


if __name__ == "__main__":
    main()
