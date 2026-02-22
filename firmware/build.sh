#!/usr/bin/env bash
# =============================================================================
# setup_and_build.sh — Install Pico SDK + toolchain, build firmware, flash
#
# Usage:
#   ./setup_and_build.sh          # build only (SDK must already be installed)
#   ./setup_and_build.sh install  # install SDK + toolchain, then build
#   ./setup_and_build.sh flash    # build + flash via UF2 bootloader
#   ./setup_and_build.sh all      # install + build + flash
#
# After a successful build, the flashable image is:
#   firmware/build/turtlebot3_pico_fw.uf2
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FIRMWARE_DIR="$SCRIPT_DIR"
SDK_DIR="$HOME/pico-sdk"
BUILD_DIR="$FIRMWARE_DIR/build"

# ─────────────────────────────────────────────────────────────────────────────
# Install prerequisites and Pico SDK 2.x (RP2350 support requires SDK >= 2.0)
# ─────────────────────────────────────────────────────────────────────────────
do_install() {
    echo "=== Installing build dependencies ==="
    sudo apt-get update -qq
    sudo apt-get install -y \
        cmake \
        gcc-arm-none-eabi \
        libnewlib-arm-none-eabi \
        libstdc++-arm-none-eabi-newlib \
        git \
        python3 \
        ninja-build

    if [[ ! -d "$SDK_DIR" ]]; then
        echo "=== Cloning Pico SDK 2.x ==="
        git clone --depth=1 --branch 2.1.1 \
            https://github.com/raspberrypi/pico-sdk.git "$SDK_DIR"
        cd "$SDK_DIR"
        git submodule update --init --recursive
    else
        echo "=== Pico SDK already present at $SDK_DIR ==="
    fi

    export PICO_SDK_PATH="$SDK_DIR"

    # Copy the standard import helper into the firmware directory
    if [[ ! -f "$FIRMWARE_DIR/pico_sdk_import.cmake" ]]; then
        cp "$SDK_DIR/external/pico_sdk_import.cmake" "$FIRMWARE_DIR/"
        echo "Copied pico_sdk_import.cmake → firmware/"
    fi

    echo "=== Install done ==="
}

# ─────────────────────────────────────────────────────────────────────────────
# Build
# ─────────────────────────────────────────────────────────────────────────────
do_build() {
    if [[ -z "${PICO_SDK_PATH:-}" ]]; then
        if [[ -d "$SDK_DIR" ]]; then
            export PICO_SDK_PATH="$SDK_DIR"
            echo "Using Pico SDK at $SDK_DIR"
        else
            echo "ERROR: PICO_SDK_PATH is not set and $SDK_DIR does not exist."
            echo "Run:  ./setup_and_build.sh install"
            exit 1
        fi
    fi

    # Copy pico_sdk_import.cmake if missing
    if [[ ! -f "$FIRMWARE_DIR/pico_sdk_import.cmake" ]]; then
        cp "$PICO_SDK_PATH/external/pico_sdk_import.cmake" "$FIRMWARE_DIR/"
    fi

    echo "=== Building firmware (target: pico2 / RP2350) ==="
    mkdir -p "$BUILD_DIR"
    cmake -S "$FIRMWARE_DIR" -B "$BUILD_DIR" \
        -DPICO_BOARD=pico2 \
        -DPICO_SDK_PATH="$PICO_SDK_PATH" \
        -GNinja \
        -DCMAKE_BUILD_TYPE=Release
    cmake --build "$BUILD_DIR" -- -j"$(nproc)"

    local uf2="$BUILD_DIR/turtlebot3_pico_fw.uf2"
    if [[ -f "$uf2" ]]; then
        echo ""
        echo "=== Build SUCCESS ==="
        echo "    UF2: $uf2"
        echo "    Size: $(du -h "$uf2" | cut -f1)"
    else
        echo "ERROR: UF2 not found — check build output above."
        exit 1
    fi
}

# ─────────────────────────────────────────────────────────────────────────────
# Flash via UF2 bootloader
# ─────────────────────────────────────────────────────────────────────────────
do_flash() {
    local uf2="$BUILD_DIR/turtlebot3_pico_fw.uf2"
    if [[ ! -f "$uf2" ]]; then
        echo "UF2 not found — building first..."
        do_build
    fi

    echo "=== Entering UF2 bootloader on Pico 2 ==="
    echo ""
    echo "The Pico must expose the RP2350 bootloader drive (RPI-RP2 or RP2350)."
    echo "If Pico is currently running CircuitPython or the C firmware, it will"
    echo "reboot into bootloader mode automatically.  Otherwise, hold BOOTSEL"
    echo "and power-cycle the board."
    echo ""

    # Try to trigger bootloader via picotool (if available)
    if command -v picotool &>/dev/null; then
        echo "Using picotool to reboot into UF2 mode..."
        picotool reboot -f -u 2>/dev/null || true
        sleep 3
    else
        echo "(picotool not found — triggering 1200-baud reset)"
        # NOTE: 1200-baud trick with CircuitPython enters UF2 bootloader
        port=$(ls /dev/ttyACM* 2>/dev/null | head -1)
        if [[ -n "$port" ]]; then
            python3 -c "
import serial, time
s = serial.Serial('$port', 1200, timeout=0.5)
time.sleep(0.5)
s.close()
" || true
            sleep 4
        fi
    fi

    # Find the UF2 bootloader mount point
    local mnt=""
    for label in RPI-RP2 RP2350 RP2350A; do
        mnt=$(lsblk -rno NAME,LABEL 2>/dev/null \
              | awk -v L="$label" '$2==L{print "/dev/"$1}' \
              | head -1)
        if [[ -n "$mnt" ]]; then break; fi
    done

    if [[ -z "$mnt" ]]; then
        echo ""
        echo "ERROR: RP2350 UF2 bootloader drive not found."
        echo "Please hold BOOTSEL and power-cycle the Pico, then re-run:"
        echo "    ./setup_and_build.sh flash"
        exit 1
    fi

    echo "Found bootloader at $mnt — mounting..."
    local boot_mnt="/mnt/rp2350"
    sudo mkdir -p "$boot_mnt"
    sudo mount "$mnt" "$boot_mnt" 2>/dev/null || true

    echo "Flashing $uf2 ..."
    sudo cp "$uf2" "$boot_mnt/"
    sync
    echo ""
    echo "=== Flash complete! ==="
    echo "The Pico will reboot automatically."
    echo ""
    echo "After ~3 seconds, the host should see:"
    echo "  /dev/ttyACM0  — Dynamixel data port"
    echo ""
    echo "Run bringup with:"
    echo "  source ~/turtlebot3_ws/install/setup.bash"
    echo "  TURTLEBOT3_MODEL=burger LDS_MODEL=LDS-03 \\"
    echo "  ros2 launch turtlebot3_bringup robot.launch.py usb_port:=/dev/ttyACM0"
}

# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────
CMD="${1:-build}"
case "$CMD" in
    install) do_install ;;
    build)   do_build ;;
    flash)   do_flash ;;
    all)     do_install; do_build; do_flash ;;
    *)
        echo "Usage: $0 [install|build|flash|all]"
        exit 1
        ;;
esac
