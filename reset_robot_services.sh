#!/usr/bin/env bash
# reset_robot_services.sh
# Purpose: Cleanly stop any running SegMAV / MAVROS / MAVLink Router / LDLiDAR processes
# and disable their systemd services so you can run everything manually.
# Usage:
#   chmod +x reset_robot_services.sh
#   ./reset_robot_services.sh
#
# Notes:
# - This script uses sudo for systemctl operations.
# - Nonexistent services or processes won't cause the script to fail.
# - After it finishes, it will print any services still enabled on the system.

set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

say() { echo -e "${YELLOW}[*]${NC} $*"; }
ok()  { echo -e "${GREEN}[✓]${NC} $*"; }
warn(){ echo -e "${RED}[!]${NC} $*"; }

# Ensure sudo timestamp is fresh (will prompt once)
say "Refreshing sudo credentials..."
sudo -v || true

# 1) Kill leftover processes
say "Killing leftover processes: mavros | mavlink-router | mavsegmav | ldlidar"
sudo pkill -f "mavros|mavlink-router|mavsegmav|ldlidar" 2>/dev/null || true
sleep 0.5
ok "pkill issued (ignoring 'no process found' messages)."

# Show what's still around
say "Processes still matching patterns (if any):"
ps aux | egrep -E "mavros|mavlink-router|mavsegmav|ldlidar" | egrep -v "egrep|reset_robot_services.sh" || true

# 2) Stop & disable services
SERVICES=(
  "ldlidar.service"
  "mavros.service"
  "mavlink-router.service"
  "segmav.service"
)

for svc in "${SERVICES[@]}"; do
  say "Stopping ${svc} ..."
  sudo systemctl stop "${svc}" 2>/dev/null || true
  ok "Stopped ${svc} (or it was not running)."

  say "Disabling ${svc} ..."
  sudo systemctl disable "${svc}" 2>/dev/null || true
  ok "Disabled ${svc} (or it was already disabled)."
done

# 3) Show any remaining enabled services on the box
say "Listing all enabled unit files (system-wide):"
sudo systemctl list-unit-files | grep enabled || true

# 4) Show currently running units related to our stack (if any slipped by)
say "Running units related to our stack:"
systemctl --type=service --state=running | egrep -E "mavros|mavlink-router|ldlidar|segmav" || true

ok "Reset complete. You can now launch components manually."
