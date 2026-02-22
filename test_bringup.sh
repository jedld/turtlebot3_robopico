#!/usr/bin/env bash
# =============================================================================
#  test_bringup.sh
#
#  Verify that all expected TurtleBot3 nodes and topics are alive after bringup.
#  Run this in a SECOND terminal while flash_and_bringup.sh (or the bringup
#  launch) is running.
#
#  Usage:
#    ./test_bringup.sh [--timeout <secs>] [--no-lidar]
#
#  Exit codes:
#    0  all checks passed
#    1  one or more checks failed
# =============================================================================

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"

WAIT_TIMEOUT=30   # seconds to wait for each topic/node
NO_LIDAR=false

RED='\033[0;31m'; YLW='\033[1;33m'; GRN='\033[0;32m'; CYN='\033[0;36m'
BLD='\033[1m'; NC='\033[0m'

info()    { echo -e "${CYN}[CHECK]${NC}  $*"; }
success() { echo -e "${GRN}[PASS]${NC}   $*"; }
fail()    { echo -e "${RED}[FAIL]${NC}   $*"; FAILURES+=("$*"); }
warn()    { echo -e "${YLW}[WARN]${NC}   $*"; }

FAILURES=()

### --------------------------------------------------------------------------
### Parse arguments
### --------------------------------------------------------------------------

while [[ $# -gt 0 ]]; do
    case "$1" in
        --timeout)  WAIT_TIMEOUT="$2"; shift ;;
        --no-lidar) NO_LIDAR=true ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
    shift
done

### --------------------------------------------------------------------------
### Source ROS 2
### --------------------------------------------------------------------------

if [[ -z "${AMENT_PREFIX_PATH:-}" ]]; then
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO}/setup.bash" 2>/dev/null || true
    WS_SETUP="${WS_DIR}/install/setup.bash"
    [[ -f "$WS_SETUP" ]] && source "$WS_SETUP"
fi

### --------------------------------------------------------------------------
### Helper: wait for topic and grab one message
### --------------------------------------------------------------------------

wait_for_topic() {
    local topic="$1"
    local msg_type="$2"
    local timeout="${3:-${WAIT_TIMEOUT}}"

    info "Waiting for ${topic} (${msg_type})…"

    local output
    if output=$(timeout "$timeout" ros2 topic echo --once "$topic" 2>&1); then
        success "${topic}  ✓"
        return 0
    else
        fail "${topic}  ✗  (not published within ${timeout}s)"
        return 1
    fi
}

### --------------------------------------------------------------------------
### Helper: check node is alive
### --------------------------------------------------------------------------

check_node() {
    local node_name="$1"
    info "Checking node: ${node_name}…"
    if ros2 node list 2>/dev/null | grep -q "${node_name}"; then
        success "Node ${node_name}  ✓"
        return 0
    else
        fail "Node ${node_name}  ✗  (not found in ros2 node list)"
        return 1
    fi
}

### --------------------------------------------------------------------------
### MAIN CHECKS
### --------------------------------------------------------------------------

echo ""
echo -e "${BLD}${CYN}══════════════════════════════════════════════════╗"
echo -e "  TurtleBot3 Bringup Test                          ║"
echo -e "══════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "  Timeout per check : ${WAIT_TIMEOUT}s"
echo -e "  LiDAR expected    : $(if $NO_LIDAR; then echo 'no'; else echo 'yes'; fi)"
echo ""

# ---- 1. Expected ROS 2 nodes -----------------------------------------------
echo -e "${BLD}--- Nodes ---${NC}"
check_node "turtlebot3_node" || true
check_node "robot_state_publisher" || true
! $NO_LIDAR && check_node "lidar" || true
echo ""

# ---- 2. Core TurtleBot3 topics ----------------------------------------------
echo -e "${BLD}--- Topics ---${NC}"
wait_for_topic "/odom"          "nav_msgs/msg/Odometry"        20 || true
wait_for_topic "/imu"           "sensor_msgs/msg/Imu"          20 || true
wait_for_topic "/joint_states"  "sensor_msgs/msg/JointState"   20 || true
wait_for_topic "/battery_state" "sensor_msgs/msg/BatteryState" 20 || true
! $NO_LIDAR && wait_for_topic "/scan" "sensor_msgs/msg/LaserScan" 20 || true
echo ""

# ---- 3. TF tree contains odom → base_footprint ----------------------------
echo -e "${BLD}--- TF tree ---${NC}"
info "Checking odom → base_footprint transform…"
if timeout 10 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 \
        | grep -q "translation\|At time"; then
    success "TF: odom → base_footprint  ✓"
else
    fail "TF: odom → base_footprint  ✗"
fi
echo ""

# ---- 4. /cmd_vel round-trip: publish a zero-velocity command ---------------
echo -e "${BLD}--- cmd_vel ---${NC}"
info "Publishing zero cmd_vel…"
if ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
        '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1; then
    success "/cmd_vel publish  ✓"
else
    fail "/cmd_vel publish  ✗"
fi
echo ""

# ---- 5. Topic rate spot-check ----------------------------------------------
echo -e "${BLD}--- Publish rates (2 s sample) ---${NC}"

check_rate() {
    local topic="$1"
    local min_hz="$2"
    local actual
    actual=$(timeout 4 ros2 topic hz "$topic" 2>/dev/null \
             | grep "average rate" | awk '{print $3}' | tr -d ':' | head -1)
    if [[ -z "$actual" ]]; then
        fail "${topic} rate: unable to measure"
        return
    fi
    # Compare using python3 since bash can't do float comparison
    if python3 -c "import sys; sys.exit(0 if float('${actual}') >= ${min_hz} else 1)" 2>/dev/null; then
        success "${topic} @ ${actual} Hz  (>= ${min_hz} Hz expected)  ✓"
    else
        fail "${topic} @ ${actual} Hz  (expected >= ${min_hz} Hz)  ✗"
    fi
}

check_rate "/odom"         10
check_rate "/imu"          10
check_rate "/joint_states" 10
! $NO_LIDAR && check_rate "/scan" 3 || true
echo ""

# ---- 6. Summary -------------------------------------------------------------
echo -e "${BLD}══════════════════════════════════════════════════${NC}"
if [[ ${#FAILURES[@]} -eq 0 ]]; then
    echo -e "${GRN}${BLD}  ALL CHECKS PASSED — TurtleBot3 bringup OK  ✓${NC}"
    echo ""
    echo "  Active topics:"
    ros2 topic list 2>/dev/null | grep -E "^/(scan|odom|imu|joint|battery|cmd_vel|tf)" \
        | sed 's/^/    /'
    EXIT_CODE=0
else
    echo -e "${RED}${BLD}  ${#FAILURES[@]} CHECK(S) FAILED:${NC}"
    for f in "${FAILURES[@]}"; do
        echo -e "    ${RED}✗${NC} ${f}"
    done
    echo ""
    echo "  Troubleshooting tips:"
    echo "    • Ensure flash_and_bringup.sh is still running in another terminal"
    echo "    • Check Pico data port: ls /dev/ttyACM*"
    echo "    • Monitor Pico REPL: screen /dev/ttyACM0 115200"
    echo "    • Check ROS 2 nodes:  ros2 node list"
    EXIT_CODE=1
fi
echo -e "${BLD}══════════════════════════════════════════════════${NC}"
echo ""

exit $EXIT_CODE
