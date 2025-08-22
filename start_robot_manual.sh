#!/usr/bin/env bash
# start_robot_manual.sh
# Purpose: Manual startup sequence for SegMAV + MAVROS + MAVLink Router + LDLiDAR.
# This follows the Sidewalk GPS Robot – Manual Ops Cheat Sheet.

set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

say() { echo -e "${YELLOW}[*]${NC} $*"; }
ok()  { echo -e "${GREEN}[✓]${NC} $*"; }
warn(){ echo -e "${RED}[!]${NC} $*"; }

say "Refreshing sudo credentials..."
sudo -v || true

############################################
# 1) Start MAVLink Router
############################################
say "Starting MAVLink Router (FC → UDP 14550/14551)..."
sudo systemctl restart mavlink-router
sleep 1
sudo systemctl status mavlink-router --no-pager | head -n 10 || true
say "Checking UDP sockets for mavlink-routerd..."
sudo ss -ulpn | egrep '14550|14551|mavlink-routerd' || warn "No router sockets found."

echo "--------------------------------------------"
echo "14550 → Mission Planner"
echo "14551 → SegMAV (+ MAVROS)"
echo "--------------------------------------------"

############################################
# 2) Start MAVROS
############################################
say "Launching MAVROS (binds to UDP 14551)..."
# Run in background with nohup to keep alive if terminal closes
nohup roslaunch mavros apm.launch fcu_url:=udp://0.0.0.0:14551@ > ~/mavros.log 2>&1 &
sleep 3
say "Quick MAVROS checks:"
rostopic echo -n1 /mavros/state || warn "No MAVROS state yet."
rostopic echo -n1 /mavros/global_position/raw/fix || warn "No GNSS fix yet."

############################################
# 3) Start LiDAR
############################################
say "Starting LDLiDAR node..."
nohup rosrun ldlidar_stl_ros ldlidar_stl_ros_node \
  _product_name:=LDLiDAR_LD19 \
  _port_name:=/dev/ldlidar \
  _port_baudrate:=230400 \
  _frame_id:=laser \
  _topic_name:=/scan > ~/ldlidar.log 2>&1 &

sleep 2
say "Verifying /scan topic..."
rostopic hz /scan || warn "/scan not ticking."

say "To relay to Mission Planner Proximity:"
echo "rosrun topic_tools relay /scan /mavros/obstacle/send"

############################################
# 4) Start SegMAV sidewalk follower
############################################
say "Starting SegMAV sidewalk follower (with preview)..."
cd ~/segmav
nohup python3 mavsegmav_merged.py \
  --device=udpin:127.0.0.1:14551 \
  --input=v4l2:///dev/Astra_rgb \
  --preview > ~/segmav.log 2>&1 &

say "SegMAV launched (check ~/segmav.log for details)."

############################################
# 5) Optional: GUIDED Waypoint Streamer
############################################
say "You can start the tiny GUIDED mission streamer with:"
echo "cd ~/segmav && python3 guided_mission.py --device=udpin:127.0.0.1:14551"

############################################
# DONE
############################################
ok "Manual startup sequence complete."
