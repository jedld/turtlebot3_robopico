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

# Reboot a running Pico into BOOTSEL mode.
# Specifically targets the TurtleBot3 motor firmware Pico (by USB product
# string) to avoid accidentally rebooting other Pico devices (e.g. LiDAR).
# Tries multiple methods in order; returns 0 on success.
enter_bootsel() {
    # Find the CDC port belonging to TurtleBot3 Pico OpenCR (motor firmware).
    # Other Pico devices (LDS-01 LIDAR Emulator) must NOT be touched.
    local target_port=""
    for port in /dev/ttyACM*; do
        local model
        model=$(udevadm info --query=property "$port" 2>/dev/null \
                | grep "^ID_MODEL=" | cut -d= -f2)
        if [[ "$model" == "TurtleBot3_Pico_OpenCR" ]]; then
            target_port="$port"
            break
        fi
    done

    if [[ -z "$target_port" ]]; then
        echo "  No TurtleBot3 Pico OpenCR port found."
        echo "  (If the device is already in BOOTSEL mode, this is expected.)"
        return 0
    fi

    echo "  Target port: ${target_port} (TurtleBot3 Pico OpenCR)"

    # Method 1: 1200-baud touch via CDC port (firmware implements this in
    #   tud_cdc_line_coding_cb — opening at 1200 baud triggers reset_usb_boot).
    #   This is preferred because it targets a specific serial port, unlike
    #   picotool -f which may reboot the wrong device.
    echo "  Sending 1200-baud reset on ${target_port} ..."
    python3 -c "
import serial, time
try:
    s = serial.Serial('$target_port', 1200, timeout=0.5)
    time.sleep(0.3)
    s.close()
    print('  1200-baud reset sent.')
except Exception as e:
    print(f'  Warning: {e}')
" 2>/dev/null || true
    return 0
}

# Wait for the Pico to appear in BOOTSEL mode (as a USB mass-storage device
# *or* detectable by picotool).  Returns 0 once ready, 1 on timeout.
wait_for_bootsel() {
    local max_tries=${1:-50}   # default 50 × 200 ms = 10 s
    echo "Waiting for BOOTSEL mode (up to $(( max_tries / 5 )) s)..."

    for _i in $(seq 1 "$max_tries"); do
        # picotool can see the device in BOOTSEL mode
        if command -v picotool &>/dev/null; then
            if sudo picotool info 2>/dev/null | grep -qi "pico\|rp2"; then
                echo "  Device detected by picotool."
                return 0
            fi
        fi
        # Mass-storage drive with known label
        for label in RPI-RP2 RP2350 RP2350A; do
            if lsblk -rno NAME,LABEL 2>/dev/null | grep -q "$label"; then
                echo "  Bootloader drive found (label: $label)."
                return 0
            fi
        done
        sleep 0.2
    done
    return 1
}

# Wait for the TurtleBot3 Pico CDC port to reappear after flashing.
# We wait for the udev SYMLINK /dev/ttyTB3 (created by 99-turtlebot3-pico.rules)
# rather than a raw /dev/ttyACM* node, because:
#   - The ACM number can change between flash cycles.
#   - The bringup service and launch files reference /dev/ttyTB3.
#   - udev needs ~1 s after the ACM node appears to process rules and create
#     the symlink; waiting for the symlink guarantees it is ready.
wait_for_cdc() {
    echo "Waiting for /dev/ttyTB3 udev symlink (up to 15 s)..."
    for _i in $(seq 1 75); do
        if [[ -L /dev/ttyTB3 ]]; then
            local target
            target=$(readlink -f /dev/ttyTB3)
            echo "  /dev/ttyTB3 → ${target}"
            return 0
        fi
        sleep 0.2
    done
    # Fallback: if the symlink is not created (udev rules not installed),
    # accept any /dev/ttyACM* so the flash is not reported as failed.
    if ls /dev/ttyACM* &>/dev/null; then
        local port
        port=$(ls /dev/ttyACM* 2>/dev/null | head -1)
        echo "  WARNING: /dev/ttyTB3 symlink not found — udev rules may not be installed."
        echo "  Raw port: ${port}"
        return 0
    fi
    return 1
}

# ─────────────────────────────────────────────────────────────────────────────
# Restart TurtleBot3 services after a successful flash
# ─────────────────────────────────────────────────────────────────────────────
do_restart_services() {
    echo "Restarting TurtleBot3 services..."
    if systemctl is-active --quiet turtlebot3-bringup.service 2>/dev/null || \
       systemctl is-failed --quiet turtlebot3-bringup.service 2>/dev/null; then
        sudo systemctl restart turtlebot3-bringup.service && \
            echo "  turtlebot3-bringup.service restarted." || \
            echo "  WARNING: failed to restart turtlebot3-bringup.service"
    else
        echo "  turtlebot3-bringup.service is not enabled — skipping."
    fi
    if systemctl is-active --quiet turtlebot3-camera.service 2>/dev/null || \
       systemctl is-failed --quiet turtlebot3-camera.service 2>/dev/null; then
        sudo systemctl restart turtlebot3-camera.service && \
            echo "  turtlebot3-camera.service restarted." || \
            echo "  WARNING: failed to restart turtlebot3-camera.service"
    fi
}

do_flash() {
    local uf2="$BUILD_DIR/turtlebot3_pico_fw.uf2"
    # Always build first so the flashed image reflects any source changes.
    do_build

    echo "=== Flashing firmware ==="
    echo "    UF2: $uf2"
    echo ""

    # Step 1: Check if already in BOOTSEL mode
    local in_bootsel=false
    if command -v picotool &>/dev/null; then
        if sudo picotool info 2>/dev/null | grep -qi "pico\|rp2"; then
            in_bootsel=true
        fi
    fi
    if ! $in_bootsel; then
        for label in RPI-RP2 RP2350 RP2350A; do
            if lsblk -rno NAME,LABEL 2>/dev/null | grep -q "$label"; then
                in_bootsel=true
                break
            fi
        done
    fi

    # Step 2: If not in BOOTSEL, reboot into it
    if ! $in_bootsel; then
        echo "Device is running — rebooting into BOOTSEL mode..."
        enter_bootsel || true
        sleep 1
        if ! wait_for_bootsel 50; then
            echo ""
            echo "ERROR: Could not enter BOOTSEL mode automatically."
            echo ""
            echo "Manual steps:"
            echo "  1. Hold BOOTSEL on the Pico 2"
            echo "  2. Press the Reset button (or re-plug USB)"
            echo "  3. Release BOOTSEL"
            echo "  4. Re-run:  ./build.sh flash"
            exit 1
        fi
    else
        echo "Device already in BOOTSEL mode."
    fi

    # Step 3: Flash the UF2
    if command -v picotool &>/dev/null; then
        echo "Flashing via picotool..."
        sudo picotool load "$uf2" -f
        echo "Rebooting..."
        sudo picotool reboot 2>/dev/null || true
    else
        # Fallback: copy UF2 to mass-storage drive
        echo "Flashing via mass-storage copy..."
        local mnt_dev=""
        for label in RPI-RP2 RP2350 RP2350A; do
            mnt_dev=$(lsblk -rno NAME,LABEL 2>/dev/null \
                      | awk -v L="$label" '$2==L{print "/dev/"$1}' \
                      | head -1)
            if [[ -n "$mnt_dev" ]]; then break; fi
        done
        if [[ -z "$mnt_dev" ]]; then
            echo "ERROR: No bootloader drive found for mass-storage copy."
            exit 1
        fi
        local boot_mnt="/mnt/rp2350"
        sudo mkdir -p "$boot_mnt"
        sudo mount "$mnt_dev" "$boot_mnt" 2>/dev/null || true
        sudo cp "$uf2" "$boot_mnt/"
        sync
        sudo umount "$boot_mnt" 2>/dev/null || true
    fi

    echo ""

    # Step 4: Wait for the device to come back as CDC
    if wait_for_cdc; then
        echo ""
        echo "=== Flash complete! ==="
        do_restart_services
    else
        echo ""
        echo "=== Flash sent — /dev/ttyTB3 did not appear within 15 s ==="
        echo "Check: ls -la /dev/ttyTB3 /dev/ttyACM*"
        echo "If /dev/ttyTB3 is missing, reinstall udev rules:"
        echo "  sudo cp 99-turtlebot3-pico.rules /etc/udev/rules.d/"
        echo "  sudo udevadm control --reload-rules && sudo udevadm trigger"
    fi
}

# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────
CMD="${1:-build}"
case "$CMD" in
    install)  do_install ;;
    build)    do_build ;;
    flash)    do_flash ;;
    restart)  do_restart_services ;;
    all)      do_install; do_flash ;;
    *)
        echo "Usage: $0 [install|build|flash|restart|all]"
        exit 1
        ;;
esac
