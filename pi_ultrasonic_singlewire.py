#!/usr/bin/env python3
"""
Read one Grove Ultrasonic Ranger (single SIG wire) from Raspberry Pi GPIO.

Backend selection (automatic by default):
- pigpio daemon (best timing)
- RPi.GPIO / rpi-lgpio fallback (no daemon)

Example:
    python3 pi_ultrasonic_singlewire.py --gpio 23
"""

import argparse
import sys
import time


def read_distance_mm_pigpio(pi, gpio: int, timeout_us: int = 30000):
    # Trigger pulse on shared SIG line
    pi.set_mode(gpio, pigpio.OUTPUT)
    pi.write(gpio, 0)
    time.sleep(2e-6)
    pi.write(gpio, 1)
    time.sleep(10e-6)
    pi.write(gpio, 0)

    # Switch to input and wait for echo
    pi.set_mode(gpio, pigpio.INPUT)

    start_wait = pi.get_current_tick()
    while pi.read(gpio) == 0:
        if pigpio.tickDiff(start_wait, pi.get_current_tick()) > timeout_us:
            return None, "wait-timeout"

    rise_tick = pi.get_current_tick()

    while pi.read(gpio) == 1:
        if pigpio.tickDiff(rise_tick, pi.get_current_tick()) > timeout_us:
            return None, "pulse-timeout"

    fall_tick = pi.get_current_tick()
    pulse_us = pigpio.tickDiff(rise_tick, fall_tick)

    # Distance conversion: mm = us * 10 / 58
    distance_mm = (pulse_us * 10.0) / 58.0
    if distance_mm < 20.0 or distance_mm > 3500.0:
        return None, "out-of-range"

    return distance_mm, None


def read_distance_mm_rpigpio(gpio_mod, gpio: int, timeout_us: int = 30000):
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


def main():
    parser = argparse.ArgumentParser(description="Single-wire Grove Ultrasonic test on Raspberry Pi")
    parser.add_argument("--gpio", type=int, default=23, help="BCM GPIO number for SIG (default: 23)")
    parser.add_argument("--interval", type=float, default=0.2, help="seconds between reads (default: 0.2)")
    parser.add_argument("--timeout-us", type=int, default=30000, help="echo timeout in microseconds (default: 30000)")
    parser.add_argument("--count", type=int, default=0, help="number of samples; 0 = run forever")
    parser.add_argument("--backend", choices=["auto", "pigpio", "rpigpio"], default="auto", help="GPIO backend")
    args = parser.parse_args()

    backend = None
    pi = None
    gpio_mod = None

    if args.backend in ("auto", "pigpio"):
        try:
            import pigpio
            candidate = pigpio.pi()
            if candidate.connected:
                pi = candidate
                backend = "pigpio"
            else:
                candidate.stop()
                if args.backend == "pigpio":
                    print("ERROR: pigpio backend requested but daemon is not running.")
                    print("If installed, start it with: sudo systemctl start pigpiod")
                    sys.exit(2)
        except Exception:
            if args.backend == "pigpio":
                print("ERROR: pigpio backend requested but python module is unavailable.")
                print("Install with: sudo apt install python3-pigpio")
                sys.exit(2)

    if backend is None and args.backend in ("auto", "rpigpio"):
        try:
            import RPi.GPIO as GPIO
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            gpio_mod = GPIO
            backend = "rpigpio"
        except Exception:
            if args.backend == "rpigpio":
                print("ERROR: RPi.GPIO backend requested but module is unavailable.")
                print("Install with: sudo apt install python3-rpi-lgpio")
                sys.exit(2)

    if backend is None:
        print("ERROR: No usable GPIO backend found.")
        print("Install one of:")
        print("  sudo apt install python3-pigpio")
        print("  sudo apt install python3-rpi-lgpio")
        sys.exit(2)

    print(f"Reading Grove ultrasonic on BCM GPIO{args.gpio} via {backend} (Ctrl+C to stop)...")

    sample = 0
    try:
        while True:
            sample += 1
            if backend == "pigpio":
                dist_mm, err = read_distance_mm_pigpio(pi, args.gpio, args.timeout_us)
            else:
                dist_mm, err = read_distance_mm_rpigpio(gpio_mod, args.gpio, args.timeout_us)
            if err:
                print(f"{sample:5d}: {err}")
            else:
                print(f"{sample:5d}: {dist_mm:7.1f} mm ({dist_mm/10.0:6.1f} cm)")

            if args.count > 0 and sample >= args.count:
                break
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        if pi is not None:
            pi.stop()
        if gpio_mod is not None:
            gpio_mod.cleanup(args.gpio)


if __name__ == "__main__":
    main()
