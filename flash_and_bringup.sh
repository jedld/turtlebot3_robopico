#!/usr/bin/env bash
# =============================================================================
#  flash_and_bringup.sh
#
#  Build the C firmware (Pico SDK 2.x), flash it onto a Raspberry Pi Pico 2
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
#    Pico SDK 2.x (installed by firmware/build.sh install)
#    arm-none-eabi-gcc   (installed by firmware/build.sh install)
#    ROS 2 Humble sourced (or will be sourced from /opt/ros/humble)
# =============================================================================

set -euo pipefail

### --------------------------------------------------------------------------
### CONFIGURATION
### --------------------------------------------------------------------------

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
FIRMWARE_DIR="${SCRIPT_DIR}/firmware"

ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

TB3_MODEL="burger"
LDS_MODEL_VAR="YDLidar-TMiniPlus"
PICO_DATA_PORT=""          # auto-detect if empty
FLASH_ONLY=false
BRINGUP_ONLY=false
NO_LIDAR=false
AUTO_YES=false

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
### STEP 1 — BUILD + FLASH C FIRMWARE
### --------------------------------------------------------------------------

flash_pico() {
    info "=== STEP 1: BUILD AND FLASH C FIRMWARE ==="

    if [[ ! -f "${FIRMWARE_DIR}/build.sh" ]]; then
        die "firmware/build.sh not found at ${FIRMWARE_DIR}/build.sh"
    fi

    # Build the firmware (install SDK if needed)
    info "Building firmware..."
    cd "$FIRMWARE_DIR"
    if [[ ! -d "$HOME/pico-sdk" ]]; then
        info "Pico SDK not found — running install first..."
        bash build.sh install
    fi
    bash build.sh flash

    success "Firmware flashed."

    # Wait for the Pico to reboot and the CDC port to appear
    info "Waiting for Pico to enumerate USB (up to 10 s)..."
    for _i in $(seq 1 20); do
        sleep 0.5
        if ls /dev/ttyACM* >/dev/null 2>&1; then
            break
        fi
    done

    if ! ls /dev/ttyACM* >/dev/null 2>&1; then
        warn "Serial port did not appear. Try re-plugging the USB cable."
        return 1
    fi
    success "Pico serial port is up."
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
    else
        # C firmware uses a single CDC port: ACM0 = Dynamixel data
        PICO_DATA_PORT="${ports[0]}"
        info "Using ${PICO_DATA_PORT} as Dynamixel data port."
    fi

    # Ensure dialout group access
    if ! id -nG | grep -qw dialout; then
        warn "User is not in 'dialout' group. Granting temporary permission..."
        sudo chmod 666 "$PICO_DATA_PORT"
    fi

    success "Dynamixel data port: ${PICO_DATA_PORT}"
}

### --------------------------------------------------------------------------
### STEP 3 — VERIFY DYNAMIXEL PING
### --------------------------------------------------------------------------

verify_pico_comms() {
    info "=== STEP 3: VERIFY DYNAMIXEL PING ==="

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

    # Use Pico-specific parameter file (custom wheel/chassis dimensions)
    # instead of the stock burger.yaml, so the original stays untouched
    # for use on a real TurtleBot3 Burger.
    local PICO_PARAM="${SCRIPT_DIR}/config/${TB3_MODEL}_pico.yaml"
    local LAUNCH_ARGS="usb_port:=${PICO_DATA_PORT}"
    if [[ -f "${PICO_PARAM}" ]]; then
        LAUNCH_ARGS+=" tb3_param_dir:=${PICO_PARAM}"
        info "Using Pico param override: ${PICO_PARAM}"
    else
        warn "No Pico param override found at ${PICO_PARAM}, using stock params"
    fi
    if $NO_LIDAR; then
        LAUNCH_ARGS+=" lidar_model:=none"
    fi

    info "Launching TurtleBot3 bringup..."
    info "  ros2 launch turtlebot3_bringup robot.launch.py ${LAUNCH_ARGS}"
    echo ""
    info "Press Ctrl+C to stop."
    echo ""

    ros2 launch turtlebot3_bringup robot.launch.py ${LAUNCH_ARGS}
}

### --------------------------------------------------------------------------
### MAIN
### --------------------------------------------------------------------------

main() {
    echo ""
    echo -e "${CYN}╔══════════════════════════════════════════════════════╗"
    echo -e "║   TurtleBot3 Pico 2 — Build, Flash & Bringup          ║"
    echo -e "╚══════════════════════════════════════════════════════╝${NC}"
    echo ""

    if ! $BRINGUP_ONLY; then
        flash_pico || { warn "Flash step reported an issue; continuing anyway..."; }
    fi

    if ! $FLASH_ONLY; then
        detect_ports
        verify_pico_comms || warn "Skipping ping verification and continuing..."
        ros2_bringup
    else
        success "Flash-only mode: done."
    fi
}

main "$@"
