#!/usr/bin/env bash
# =============================================================================
#  flash_and_bringup.sh
#
#  Flash CircuitPython + TurtleBot3 firmware onto a Raspberry Pi Pico 2
#  (Cytron Robo Pico), then verify and launch the TurtleBot3 ROS 2 stack.
#
#  Usage:
#    ./flash_and_bringup.sh [OPTIONS]
#
#  Options:
#    --flash-only        Stop after flashing; do not launch ROS 2
#    --bringup-only      Skip flashing; go straight to ROS 2 bringup
#    --port  <dev>       Override the detected /dev/ttyACM* port (data port)
#    --model <model>     TurtleBot3 model: burger | waffle | waffle_pi (default: burger)
#    --lidar <model>     LiDAR model:  LDS-01 | LDS-02 | LDS-03 (default: LDS-03)
#    --no-lidar          Bringup without LiDAR
#    -y / --yes          Answer yes to all prompts
#    -h / --help         Show this help
#
#  Requirements:
#    sudo privileges (to mount the Pico mass-storage drive)
#    ROS 2 Humble sourced (or will be sourced from /opt/ros/humble)
#    curl  — for downloading CircuitPython UF2 (optional; skipped if .uf2 exists)
# =============================================================================

set -euo pipefail

### --------------------------------------------------------------------------
### CONFIGURATION  (edit these defaults as needed)
### --------------------------------------------------------------------------

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

TB3_MODEL="burger"
LDS_MODEL_VAR="LDS-03"
PICO_DATA_PORT=""          # auto-detect if empty
FLASH_ONLY=false
BRINGUP_ONLY=false
NO_LIDAR=false
AUTO_YES=false

# CircuitPython 9.x for RP2350 (Pico 2)
CP_VERSION="9.2.1"
CP_UF2_URL="https://downloads.circuitpython.org/bin/raspberry_pi_pico2/en_US/adafruit-circuitpython-raspberry_pi_pico2-en_US-${CP_VERSION}.uf2"
CP_UF2_CACHE="${SCRIPT_DIR}/circuitpython_pico2_${CP_VERSION}.uf2"

# adafruit_motor library — no longer needed; motor control is built into main.py
# (keeping variable for reference only)
LIB_BUNDLE_VERSION="9.x"
LIB_BUNDLE_CACHE="${SCRIPT_DIR}/adafruit_bundle.zip"  # unused

MOUNT_POINT="/mnt/pico_flash"

# Colours
RED='\033[0;31m'; YLW='\033[1;33m'; GRN='\033[0;32m'; CYN='\033[0;36m'; NC='\033[0m'

### --------------------------------------------------------------------------
### HELPERS
### --------------------------------------------------------------------------

info()    { echo -e "${CYN}[INFO]${NC}  $*"; }
success() { echo -e "${GRN}[OK]${NC}    $*"; }
warn()    { echo -e "${YLW}[WARN]${NC}  $*"; }
die()     { echo -e "${RED}[ERROR]${NC} $*" >&2; exit 1; }

confirm() {
    # confirm "question"  — returns 0 (yes) or 1 (no)
    if $AUTO_YES; then
        echo -e "${YLW}[AUTO-YES]${NC} $1"
        return 0
    fi
    read -r -p "$(echo -e "${YLW}?${NC} $1 [Y/n] ")" reply
    [[ "${reply:-Y}" =~ ^[Yy]$ ]]
}

require_cmd() {
    command -v "$1" >/dev/null 2>&1 || die "Required command not found: $1"
}

### --------------------------------------------------------------------------
### ARGUMENT PARSING
### --------------------------------------------------------------------------

while [[ $# -gt 0 ]]; do
    case "$1" in
        --flash-only)   FLASH_ONLY=true   ;;
        --bringup-only) BRINGUP_ONLY=true ;;
        --port)         PICO_DATA_PORT="$2"; shift ;;
        --model)        TB3_MODEL="$2";      shift ;;
        --lidar)        LDS_MODEL_VAR="$2";  shift ;;
        --no-lidar)     NO_LIDAR=true ;;
        -y|--yes)       AUTO_YES=true ;;
        -h|--help)
            sed -n '/^#  Usage:/,/^# =.*$/p' "$0" | head -n -1
            exit 0 ;;
        *) die "Unknown option: $1" ;;
    esac
    shift
done

### --------------------------------------------------------------------------
### STEP 1 — FLASH CIRCUITPYTHON + FIRMWARE
### --------------------------------------------------------------------------

flash_pico() {
    info "=== STEP 1: FLASH CIRCUITPYTHON + FIRMWARE ==="

    # ---- 1a. Find the RP2350 block device -----------------------------------
    info "Searching for RP2350 in BOOTSEL mode…"
    local PICO_DEV=""

    # Prefer the partition (sda1) over the whole-disk device (sda)
    # lsblk reports VENDOR/MODEL only on the disk node, but the partition is
    # what we need for FAT mounting.  Check parent disks then pick partition.
    for d in $(lsblk -rno NAME,VENDOR,MODEL 2>/dev/null \
               | awk '/2350|RP2|RPI/{print $1}'); do
        local candidate="/dev/${d}"
        # If a partition of this disk exists, prefer it
        local part
        part=$(lsblk -rno NAME "/dev/${d}" 2>/dev/null | grep "^${d}[0-9]" | head -1)
        if [[ -n "$part" ]]; then
            PICO_DEV="/dev/${part}"
        else
            PICO_DEV="${candidate}"
        fi
        break
    done

    # Fallback: look by block device label
    if [[ -z "$PICO_DEV" ]]; then
        PICO_DEV=$(lsblk -o NAME,LABEL -rn 2>/dev/null \
                   | awk '/RP2350|RPI-RP2/{print "/dev/"$1}' | head -1)
    fi

    if [[ -z "$PICO_DEV" ]]; then
        die "Pico not found in BOOTSEL mode.\n\n  Hold BOOTSEL on the Pico and reconnect USB, then re-run this script."
    fi
    success "Found RP2350 at ${PICO_DEV}"

    # ---- 1b. Download CircuitPython UF2 if not cached -----------------------
    if [[ ! -f "$CP_UF2_CACHE" ]]; then
        info "Downloading CircuitPython ${CP_VERSION} for RP2350…"
        curl -fL --progress-bar -o "$CP_UF2_CACHE" "$CP_UF2_URL" \
            || die "Download failed. Check internet connection."
    else
        info "Using cached CircuitPython UF2: ${CP_UF2_CACHE}"
    fi

    # ---- 1c. Mount the Pico BOOTSEL drive and copy the UF2 -----------------
    # Strategy 1: plain mount (works when partition table is intact)
    # Strategy 2: mount the whole-disk device directly (single-FAT, no PT)
    # Strategy 3: dd the UF2 directly to the block device (always works)
    info "Mounting ${PICO_DEV} at ${MOUNT_POINT}…"
    sudo mkdir -p "$MOUNT_POINT"

    local MOUNTED=false
    if sudo mount -t vfat -o uid="$(id -u)",gid="$(id -g)" \
            "$PICO_DEV" "$MOUNT_POINT" 2>/dev/null; then
        MOUNTED=true
    else
        # Try the parent disk as a raw FAT volume (no partition table)
        local PARENT_DEV
        PARENT_DEV=$(lsblk -no PKNAME "$PICO_DEV" 2>/dev/null | head -1)
        PARENT_DEV="/dev/${PARENT_DEV}"
        if [[ "$PARENT_DEV" != "/dev/" ]] && \
           sudo mount -t vfat -o uid="$(id -u)",gid="$(id -g)" \
               "$PARENT_DEV" "$MOUNT_POINT" 2>/dev/null; then
            PICO_DEV="$PARENT_DEV"
            MOUNTED=true
        fi
    fi

    # ---- 1d. Copy UF2 — the Pico reboots automatically after copy ----------
    info "Flashing CircuitPython (Pico will reboot automatically)…"
    if $MOUNTED; then
        cp "$CP_UF2_CACHE" "${MOUNT_POINT}/"
        sync
        sleep 1
        sudo umount "$MOUNT_POINT" 2>/dev/null || true
    else
        # Fallback: write UF2 directly to the block device with dd
        warn "Could not mount as FAT; falling back to raw dd write…"
        sudo dd if="$CP_UF2_CACHE" of="$PICO_DEV" bs=512K conv=fsync status=progress \
            || die "dd flash failed. Check that ${PICO_DEV} is the correct device."
        sudo sync
    fi
    success "CircuitPython flashed."

    # ---- 1e. Wait for CIRCUITPY drive to appear ----------------------------
    info "Waiting for CIRCUITPY drive to enumerate (up to 30 s)…"
    local CIRCUITPY_DEV=""
    for i in $(seq 1 30); do
        sleep 1
        CIRCUITPY_DEV=$(lsblk -o NAME,LABEL -rn 2>/dev/null \
                        | awk '/CIRCUITPY/{print "/dev/"$1}' | head -1)
        if [[ -n "$CIRCUITPY_DEV" ]]; then
            break
        fi
    done

    if [[ -z "$CIRCUITPY_DEV" ]]; then
        die "CIRCUITPY drive did not appear. Check USB cable and power."
    fi
    success "CIRCUITPY drive at ${CIRCUITPY_DEV}"

    # ---- 1f. Mount CIRCUITPY and install files ------------------------------
    info "Mounting CIRCUITPY…"
    sudo mount -t vfat -o uid="$(id -u)",gid="$(id -g)",sync "$CIRCUITPY_DEV" "$MOUNT_POINT" \
        || die "Failed to mount CIRCUITPY drive."

    # ---- 1g. Copy boot.py and main.py first (no lib required to boot) ------
    info "Copying boot.py and main.py…"
    cp "${SCRIPT_DIR}/boot.py"  "${MOUNT_POINT}/boot.py"
    cp "${SCRIPT_DIR}/main.py"  "${MOUNT_POINT}/main.py"
    sync

    # ---- 1h. Install adafruit_motor library ---------------------------------
    # Motor control is now built into main.py — no external library needed.
    success "No external libraries required (motor driver is built-in)."

    sync
    sudo umount "$MOUNT_POINT"
    success "Firmware installed on Pico."

    # ---- 1i. Trigger soft-reset so boot.py (dual CDC) takes effect ----------
    # CircuitPython needs a full reboot (not just main.py reload) to re-enumerate
    # USB with the new boot.py settings.  Send Ctrl+D to the REPL port.
    local REPL_PORT
    REPL_PORT=$(ls /dev/ttyACM* 2>/dev/null | sort | head -1)
    if [[ -n "$REPL_PORT" ]]; then
        info "Sending soft-reset to ${REPL_PORT} so boot.py (dual CDC) takes effect…"
        python3 - "$REPL_PORT" <<'PYEOF'
import sys, serial, time
try:
    s = serial.Serial(sys.argv[1], 115200, timeout=2)
    time.sleep(0.5)
    s.write(b'\x03\x03\x04')   # Ctrl+C, Ctrl+C, Ctrl+D  (soft reset)
    time.sleep(0.3)
    s.close()
except Exception as e:
    print(f"Reset warning: {e}", file=sys.stderr)
PYEOF
    fi

    # ---- 1j. Pico reboots, wait for CDC data port (/dev/ttyACM1) -----------
    info "Waiting for Pico to reboot with dual CDC (up to 20 s)…"
    for i in $(seq 1 20); do
        sleep 1
        local port_count
        port_count=$(ls /dev/ttyACM* 2>/dev/null | wc -l)
        if [[ "$port_count" -ge 2 ]]; then
            break
        fi
    done

    port_count=$(ls /dev/ttyACM* 2>/dev/null | wc -l)
    if [[ "$port_count" -lt 2 ]]; then
        echo ""
        warn "Only one CDC port appeared. The Pico needs one more physical reset"
        warn "for boot.py's dual-CDC setting to take full effect."
        echo ""
        if $AUTO_YES; then
            warn "Auto-yes mode: continuing with single port."
        else
            echo -e "  ${YLW}Please press the RESET button on the Pico now, then press Enter.${NC}"
            read -r _
            sleep 3
        fi
    fi

    if ! ls /dev/ttyACM* >/dev/null 2>&1; then
        warn "Serial ports did not appear. Try re-plugging the USB cable."
        return 1
    fi
    success "Pico serial ports are up."
}

### --------------------------------------------------------------------------
### STEP 2 — DETECT PORTS
### --------------------------------------------------------------------------

detect_ports() {
    info "=== STEP 2: DETECT SERIAL PORTS ==="

    # Give udev a moment to settle
    sleep 2

    local ports=( $(ls /dev/ttyACM* 2>/dev/null | sort) )
    if [[ ${#ports[@]} -eq 0 ]]; then
        die "No /dev/ttyACM* ports found. Is the Pico connected and firmware running?"
    fi

    info "Detected ports: ${ports[*]}"

    if [[ -n "$PICO_DATA_PORT" ]]; then
        info "Using user-specified port: ${PICO_DATA_PORT}"
    elif [[ ${#ports[@]} -ge 2 ]]; then
        # Old dual-CDC boot.py: ACM0 = REPL, ACM1 = Dynamixel data port
        PICO_DATA_PORT="${ports[1]}"
        info "Two CDC ports found. Using ${PICO_DATA_PORT} as the Dynamixel data port."
        info "(${ports[0]} is the REPL console)"
    else
        # Single data-only port (current boot.py default): ACM0 = Dynamixel data
        PICO_DATA_PORT="${ports[0]}"
        info "Single CDC port mode: using ${PICO_DATA_PORT} as Dynamixel data port."
    fi

    # Ensure dialout group access
    if ! id -nG | grep -qw dialout; then
        warn "User is not in 'dialout' group. Granting temporary permission…"
        sudo chmod 666 "$PICO_DATA_PORT"
    fi

    success "Dynamixel data port: ${PICO_DATA_PORT}"
}

### --------------------------------------------------------------------------
### STEP 3 — VERIFY DYNAMIXEL PING
### --------------------------------------------------------------------------

verify_pico_comms() {
    info "=== STEP 3: VERIFY DYNAMIXEL PING ==="

    # Python-based ping: send Dynamixel Protocol 2.0 PING to ID 200
    python3 - "${PICO_DATA_PORT}" <<'PYEOF'
import sys, serial, struct, time

PORT   = sys.argv[1]
DEV_ID = 200
TIMEOUT = 5.0

def crc16(data):
    tbl = []
    for i in range(256):
        crc, d = 0, i << 8
        for _ in range(8):
            if (crc ^ d) & 0x8000:
                crc = ((crc << 1) ^ 0x8005) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
            d = (d << 1) & 0xFFFF
        tbl.append(crc)
    crc = 0
    for b in data:
        crc = ((crc << 8) ^ tbl[((crc >> 8) ^ b) & 0xFF]) & 0xFFFF
    return crc

# Build PING packet
pkt = bytearray([0xFF, 0xFF, 0xFD, 0x00, DEV_ID, 0x03, 0x00, 0x01])
crc = crc16(bytes(pkt))
pkt += bytearray([crc & 0xFF, (crc >> 8) & 0xFF])

try:
    ser = serial.Serial(PORT, 1000000, timeout=0.5)
    time.sleep(0.5)
    ser.reset_input_buffer()
    ser.write(bytes(pkt))
    time.sleep(0.2)
    resp = ser.read(14)
    ser.close()
except Exception as e:
    print(f"Serial error: {e}")
    sys.exit(1)

if len(resp) >= 11 and resp[0:3] == bytes([0xFF, 0xFF, 0xFD]):
    print(f"PING OK — device ID {resp[4]}, model=0x{resp[9]|resp[10]<<8:04X}")
    sys.exit(0)
else:
    print(f"No valid PING reply (got {len(resp)} bytes: {resp.hex()})")
    sys.exit(1)
PYEOF
    local rc=$?
    if [[ $rc -ne 0 ]]; then
        warn "PING verification failed."
        warn "The firmware may still be booting. Retry with --bringup-only after"
        warn "the Pico has fully started."
        return 1
    fi
    success "Pico Dynamixel interface is responding correctly."
}

### --------------------------------------------------------------------------
### STEP 4 — ROS 2 BRINGUP
### --------------------------------------------------------------------------

ros2_bringup() {
    info "=== STEP 4: ROS 2 BRINGUP ==="

    # Source ROS 2
    if [[ -z "${ROS_DISTRO:-}" ]]; then
        [[ -f "$ROS_SETUP" ]] || die "ROS 2 setup not found at ${ROS_SETUP}"
        set +u
        # shellcheck disable=SC1090
        source "$ROS_SETUP"
        set -u
    fi

    # Source workspace install
    local WS_SETUP="${WS_DIR}/install/setup.bash"
    if [[ -f "$WS_SETUP" ]]; then
        set +u
        # shellcheck disable=SC1090
        source "$WS_SETUP"
        set -u
        info "Sourced workspace: ${WS_SETUP}"
    else
        warn "Workspace not yet built (install/setup.bash not found)."
        if confirm "Build the workspace now? (colcon build --symlink-install)"; then
            pushd "$WS_DIR" >/dev/null
            colcon build --symlink-install 2>&1 | tail -20
            set +u
            source "${WS_DIR}/install/setup.bash"
            set -u
            popd >/dev/null
            success "Workspace built."
        else
            die "Cannot proceed without a built workspace."
        fi
    fi

    export TURTLEBOT3_MODEL="${TB3_MODEL}"
    export LDS_MODEL="${LDS_MODEL_VAR}"

    info "Using TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL}  LDS_MODEL=${LDS_MODEL}"
    info "Dynamixel port: ${PICO_DATA_PORT}"

    local LAUNCH_ARGS="usb_port:=${PICO_DATA_PORT}"
    if $NO_LIDAR; then
        LAUNCH_ARGS+=" lidar_model:=none"
    fi

    info "Launching TurtleBot3 bringup…"
    info "  ros2 launch turtlebot3_bringup robot.launch.py ${LAUNCH_ARGS}"
    echo ""
    info "Press Ctrl+C to stop."
    echo ""

    # Run the bringup; this blocks until Ctrl+C
    ros2 launch turtlebot3_bringup robot.launch.py ${LAUNCH_ARGS}
}

### --------------------------------------------------------------------------
### MAIN
### --------------------------------------------------------------------------

main() {
    echo ""
    echo -e "${CYN}╔══════════════════════════════════════════════════════╗"
    echo -e "║   TurtleBot3 Pico 2 — Flash & Bringup                  ║"
    echo -e "╚══════════════════════════════════════════════════════╝${NC}"
    echo ""

    if ! $BRINGUP_ONLY; then
        # Check that the Pico is in BOOTSEL mode
        if ! lsblk -o VENDOR,MODEL 2>/dev/null | grep -qi "2350\|RP2"; then
            warn "RP2350 not found in BOOTSEL mode."
            if ls /dev/ttyACM* >/dev/null 2>&1; then
                warn "But /dev/ttyACM* ports exist — Pico may already have firmware."
                if confirm "Skip flashing and go straight to bringup?"; then
                    BRINGUP_ONLY=true
                else
                    die "Re-run after holding BOOTSEL and reconnecting USB."
                fi
            else
                die "Pico not detected at all. Check USB connection."
            fi
        fi
    fi

    if ! $BRINGUP_ONLY; then
        flash_pico || { warn "Flash step reported an issue; continuing anyway…"; }
    fi

    if ! $FLASH_ONLY; then
        detect_ports
        verify_pico_comms || warn "Skipping ping verification and continuing…"
        ros2_bringup
    else
        success "Flash-only mode: done."
    fi
}

main "$@"
