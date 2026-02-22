#!/usr/bin/env bash
# =============================================================================
# build.sh — Build and optionally flash the LDS-01 LIDAR Emulator firmware
#
# Usage:
#   ./build.sh              # build only (SDK must already be installed)
#   ./build.sh install      # install Pico SDK + ARM toolchain, then build
#   ./build.sh flash        # build + flash via UF2 bootloader
#   ./build.sh all          # install + build + flash
#
# After a successful build the flashable image is:
#   build/lds01_emulator.uf2
#
# Shared Pico SDK with the sibling firmware/ project
# ---------------------------------------------------
# If $HOME/pico-sdk already exists (built by firmware/build.sh) it is reused.
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EMULATOR_DIR="$SCRIPT_DIR"
SDK_DIR="$HOME/pico-sdk"
BUILD_DIR="$EMULATOR_DIR/build"

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

    if [[ ! -f "$EMULATOR_DIR/pico_sdk_import.cmake" ]]; then
        cp "$SDK_DIR/external/pico_sdk_import.cmake" "$EMULATOR_DIR/"
        echo "Copied pico_sdk_import.cmake"
    fi

    echo "=== Install done ==="
}

# ─────────────────────────────────────────────────────────────────────────────
do_build() {
    if [[ -z "${PICO_SDK_PATH:-}" ]]; then
        if [[ -d "$SDK_DIR" ]]; then
            export PICO_SDK_PATH="$SDK_DIR"
        else
            echo "ERROR: PICO_SDK_PATH not set and $SDK_DIR not found."
            echo "Run:  ./build.sh install"
            exit 1
        fi
    fi

    # Copy pico_sdk_import.cmake if it hasn't been placed yet.
    if [[ ! -f "$EMULATOR_DIR/pico_sdk_import.cmake" ]]; then
        # Try the sibling firmware/ directory first (avoid re-cloning the SDK).
        sibling="$(dirname "$EMULATOR_DIR")/firmware/pico_sdk_import.cmake"
        if [[ -f "$sibling" ]]; then
            cp "$sibling" "$EMULATOR_DIR/"
            echo "Copied pico_sdk_import.cmake from sibling firmware/"
        else
            cp "$PICO_SDK_PATH/external/pico_sdk_import.cmake" "$EMULATOR_DIR/"
            echo "Copied pico_sdk_import.cmake from SDK"
        fi
    fi

    echo "=== Building LDS-01 emulator (target: pico2 / RP2350) ==="
    mkdir -p "$BUILD_DIR"
    cmake -S "$EMULATOR_DIR" -B "$BUILD_DIR" \
        -DPICO_BOARD=pico2 \
        -DPICO_SDK_PATH="$PICO_SDK_PATH" \
        -GNinja \
        -DCMAKE_BUILD_TYPE=Release
    cmake --build "$BUILD_DIR" -- -j"$(nproc)"

    local uf2="$BUILD_DIR/lds01_emulator.uf2"
    if [[ -f "$uf2" ]]; then
        echo ""
        echo "=== Build SUCCESS ==="
        echo "    UF2 : $uf2"
        echo "    Size: $(du -h "$uf2" | cut -f1)"
        echo ""
        echo "Flash instructions:"
        echo "  ./build.sh flash"
    else
        echo "ERROR: UF2 not found — check build output above."
        exit 1
    fi
}

# ─────────────────────────────────────────────────────────────────────────────
# Helper: find the raw block device for the RP2350 UF2 bootloader.
# Checks every common label; falls back to USB vendor/product string search.
# Prints the device path (e.g. /dev/sda1) or nothing if not found.
_find_rp2350_dev() {
    local dev=""

    # 1. Search via lsblk label
    for label in RP2350 RPI-RP2 RP2350A; do
        dev=$(lsblk -rno NAME,LABEL 2>/dev/null \
              | awk -v L="$label" '$2==L{print "/dev/"$1}' \
              | head -1)
        [[ -n "$dev" ]] && echo "$dev" && return 0
    done

    # 2. Fall back: any removable USB block device whose model contains "RP2350"
    #    (shown as VENDOR=RPI MODEL=RP2350 in lsblk)
    dev=$(lsblk -rno NAME,VENDOR,MODEL 2>/dev/null \
          | awk '/RP2350/{print "/dev/"$1}' \
          | head -1)
    [[ -n "$dev" ]] && echo "$dev" && return 0

    return 1
}

# Helper: mount a block device and print the mountpoint.
# Tries udisksctl (no sudo) first, then sudo mount.
_mount_dev() {
    local dev="$1"

    # Already mounted?
    local existing
    existing=$(lsblk -rno MOUNTPOINT "$dev" 2>/dev/null | grep -v '^$' | head -1)
    if [[ -n "$existing" ]]; then
        echo "$existing"
        return 0
    fi

    # Try udisksctl (user-space, no password, works on Ubuntu/Raspberry Pi OS)
    if command -v udisksctl &>/dev/null; then
        local out
        out=$(udisksctl mount -b "$dev" 2>&1) && {
            # Output: "Mounted <dev> at <path>."
            echo "$out" | awk 'NF{print $NF}' | tr -d '.'
            return 0
        }
    fi

    # Fall back to sudo mount
    local boot_mnt="/mnt/rp2350_lds"
    sudo mkdir -p "$boot_mnt"
    if sudo mount "$dev" "$boot_mnt"; then
        echo "$boot_mnt"
        return 0
    fi

    return 1
}

# ─────────────────────────────────────────────────────────────────────────────
do_flash() {
    local uf2="$BUILD_DIR/lds01_emulator.uf2"
    if [[ ! -f "$uf2" ]]; then
        echo "UF2 not found — building first..."
        do_build
    fi

    # ── Step 1: trigger bootloader if firmware is currently running ──────────
    local already_in_boot=0
    if _find_rp2350_dev > /dev/null 2>&1; then
        echo "RP2350 bootloader drive already present — skipping reboot trigger."
        already_in_boot=1
    fi

    if [[ "$already_in_boot" -eq 0 ]]; then
        echo "=== Triggering UF2 bootloader ==="
        if command -v picotool &>/dev/null; then
            echo "Using picotool..."
            picotool reboot -f -u 2>/dev/null || true
            sleep 3
        else
            echo "(picotool not found — trying 1200-baud CDC reset)"
            local port
            port=$(ls /dev/ttyACM* 2>/dev/null | head -1)
            if [[ -n "$port" ]]; then
                python3 -c "
import serial, time
s = serial.Serial('$port', 1200, timeout=0.5)
time.sleep(0.5)
s.close()
" || true
                sleep 4
            else
                echo "No /dev/ttyACM* found — is the firmware already running?"
                echo "Hold BOOTSEL, power-cycle the Pico, then re-run:  ./build.sh flash"
                exit 1
            fi
        fi

        # Wait up to 8 s for the mass-storage drive to appear
        echo -n "Waiting for RP2350 drive"
        for i in $(seq 1 16); do
            sleep 0.5
            echo -n "."
            _find_rp2350_dev > /dev/null 2>&1 && break
        done
        echo ""
    fi

    # ── Step 2: locate and mount the drive ──────────────────────────────────
    local blkdev
    blkdev=$(_find_rp2350_dev) || {
        echo ""
        echo "ERROR: RP2350 UF2 bootloader drive not found."
        echo "Hold BOOTSEL, power-cycle the Pico, then re-run:  ./build.sh flash"
        exit 1
    }
    echo "Found bootloader block device: $blkdev"

    local mntpt
    mntpt=$(_mount_dev "$blkdev") || {
        echo "ERROR: Could not mount $blkdev."
        echo "Try:  sudo mount $blkdev /mnt/rp && sudo cp $uf2 /mnt/rp/"
        exit 1
    }
    echo "Mounted at: $mntpt"

    # ── Step 3: copy the UF2 ────────────────────────────────────────────────
    # The drive is typically owned by root; use sudo cp directly to avoid a
    # spurious "Permission denied" message from the non-sudo attempt.
    echo "Flashing $uf2 ..."
    if [[ -w "$mntpt" ]]; then
        cp "$uf2" "$mntpt/"
    else
        sudo cp "$uf2" "$mntpt/"
    fi
    sync

    # Unmount cleanly (the Pico reboots on eject; ignore errors)
    if command -v udisksctl &>/dev/null; then
        udisksctl unmount -b "$blkdev" 2>/dev/null || true
    else
        sudo umount "$mntpt" 2>/dev/null || true
    fi

    echo ""
    echo "=== Flash complete! ==="
    echo "After ~3 seconds the Pico will enumerate as:"
    echo "  /dev/ttyACM0  — LDS-01 config port (USB CDC)"
    echo ""
    echo "Connect the robot's LIDAR data line to GP0 (UART0 TX)."
    echo ""
    echo "Quick test:"
    echo "  minicom -D /dev/ttyACM0 -b 115200   (then type HELP)"
    echo ""
    echo "ROS 2 LIDAR driver:"
    echo "  ros2 launch hls_lfcd_lds_driver hlds_laser.launch.py port:=/dev/ttyUSB0"
}

# ─────────────────────────────────────────────────────────────────────────────
CMD="${1:-build}"
case "$CMD" in
    install) do_install ;;
    build)   do_build   ;;
    flash)   do_flash   ;;
    all)     do_install; do_build; do_flash ;;
    *)
        echo "Usage: $0 [install|build|flash|all]"
        exit 1
        ;;
esac
