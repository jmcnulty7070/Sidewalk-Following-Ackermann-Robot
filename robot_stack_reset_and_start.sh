#!/usr/bin/env bash
# robot_stack_reset_and_start.sh
# Purpose: One-shot "clean reset" + manual startup for MAVLink Router, MAVROS, LDLiDAR, and SegMAV.
# - Stops/kills anything leftover, disables services (to avoid auto-respawn),
# - Starts the stack in the correct order for manual ops.
#
# Usage:
#   chmod +x robot_stack_reset_and_start.sh
#   ./robot_stack_reset_and_start.sh [--headless] [--no-disable] [--skip-lidar]
#
# Options:
#   --headless     Run SegMAV without GUI preview.
#   --no-disable   Stop services but do NOT disable them.
#   --skip-lidar   Do not start the LDLiDAR node (useful if testing without sensor).
#
# Logs:
#   ~/mavros.log, ~/ldlidar.log, ~/segmav.log
#
# Requirements:
#   - ROS env sourced (e.g., source ~/catkin_ws/devel/setup.bash)
#   - mavlink-router service configured
#   - ldlidar_stl_ros installed & /dev/ldlidar udev present
#   - segmav repo at ~/segmav with mavsegmav_merged.py
#   - Astra camera udev as /dev/Astra_rgb (or edit --input below)

set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

say() { echo -e "${YELLOW}[*]${NC} $*"; }
ok()  { echo -e "${GREEN}[✓]${NC} $*"; }
warn(){ echo -e "${RED}[!]${NC} $*"; }
info(){ echo -e "${BLUE}[-]${NC} $*"; }

HEADLESS=0
DISABLE=1
START_LIDAR=1

for arg in "$@"; do
  case "$arg" in
    --headless) HEADLESS=1 ;;
    --no-disable) DISABLE=0 ;;
    --skip-lidar) START_LIDAR=0 ;;
    *) warn "Unknown option: $arg" ;;
  esac
done

# Confirm ROS availability early
if ! command -v rostopic >/dev/null 2>&1; then
  warn "ROS environment not detected (no 'rostopic' in PATH). Make sure to 'source /opt/ros/<distro>/setup.bash' and your workspace."
fi

say "Refreshing sudo credentials..."
sudo -v || true

############################################
# CLEAN RESET
############################################
say "Killing leftover processes: mavros | mavlink-router | mavsegmav | ldlidar"
sudo pkill -f "mavros|mavlink-router|mavsegmav|ldlidar" 2>/dev/null || true
sleep 0.5
ok "pkill issued (ignoring 'no process found')."

say "Processes still matching (if any):"
ps aux | egrep -E "mavros|mavlink-router|mavsegmav|ldlidar" | egrep -v "egrep|robot_stack_reset_and_start.sh" || true

SERVICES=(
  "ldlidar.service"
  "mavros.service"
  "mavlink-router.service"
  "segmav.service"
)

for svc in "${SERVICES[@]}"; do
  say "Stopping ${svc} ..."
  sudo systemctl stop "${svc}" 2>/dev/null || true
  ok "Stopped ${svc} (or already stopped)."

  if [[ "$DISABLE" -eq 1 ]]; then
    say "Disabling ${svc} ..."
    sudo systemctl disable "${svc}" 2>/dev/null || true
    ok "Disabled ${svc} (or already disabled)."
  else
    info "Skipping disable for ${svc} (per --no-disable)."
  fi
done

say "Enabled unit files (system-wide):"
sudo systemctl list-unit-files | grep enabled || true

############################################
# STARTUP SEQUENCE
############################################
say "Starting MAVLink Router (FC → UDP 14550/14551) ..."
sudo systemctl restart mavlink-router
sleep 1
sudo systemctl status mavlink-router --no-pager | head -n 12 || true
say "Checking UDP sockets for mavlink-routerd..."
sudo ss -ulpn | egrep '14550|14551|mavlink-routerd' || warn "No router sockets found (verify mavlink-router config)."
echo "--------------------------------------------"
echo "14550 → Mission Planner"
echo "14551 → SegMAV (+ MAVROS)"
echo "--------------------------------------------"

say "Launching MAVROS (binds udp://0.0.0.0:14551@) ..."
nohup roslaunch mavros apm.launch fcu_url:=udp://0.0.0.0:14551@ > ~/mavros.log 2>&1 &
sleep 3
say "Quick MAVROS checks:"
rostopic echo -n1 /mavros/state || warn "No MAVROS state yet (is ROS core up? UDP 14551 active?)"
rostopic echo -n1 /mavros/global_position/raw/fix || warn "No GNSS fix yet (may be OK if indoors / no lock)."

if [[ "$START_LIDAR" -eq 1 ]]; then
  say "Starting LDLiDAR node..."
  nohup rosrun ldlidar_stl_ros ldlidar_stl_ros_node \
    _product_name:=LDLiDAR_LD19 \
    _port_name:=/dev/ldlidar \
    _port_baudrate:=230400 \
    _frame_id:=laser \
    _topic_name:=/scan > ~/ldlidar.log 2>&1 &
  sleep 2
  say "Verifying /scan topic ..."
  rostopic hz /scan || warn "/scan not ticking (check cabling, udev, permissions)."

  info "To relay LiDAR to Mission Planner Proximity:"
  echo "rosrun topic_tools relay /scan /mavros/obstacle/send"
else
  info "Skipping LDLiDAR startup (--skip-lidar)."
fi

say "Starting SegMAV sidewalk follower ..."
cd ~/segmav
SEG_ARGS=(
  "--device=udpin:127.0.0.1:14551"
  "--input=v4l2:///dev/Astra_rgb"
)
if [[ "$HEADLESS" -eq 0 ]]; then
  SEG_ARGS+=("--preview")
fi
nohup python3 mavsegmav_merged.py "${SEG_ARGS[@]}" > ~/segmav.log 2>&1 &
ok "SegMAV launched (log: ~/segmav.log)."

info "Optional GUIDED Waypoint Streamer:"
echo "cd ~/segmav && python3 guided_mission.py --device=udpin:127.0.0.1:14551"

ok "All done. Router → MAVROS → LiDAR → SegMAV are up (per options)."
