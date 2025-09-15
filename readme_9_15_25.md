
# SegMAV Guided Mission + LiDAR Avoidance (TCP Runner)
**Date:** 2025‑09‑15  
**Scenario:** Mission from Mission Planner, GUIDED mode runner (pymavlink) over **TCP 5760**, SegMAV “nudges”, and LiDAR → `OBSTACLE_DISTANCE` via UDP.

---

## Topology (Quick Mental Model)
- **Flight Controller (FC)** ⟷ **mavlink-router** (Jetson)  
  - Serial: `/dev/telem1` @ **115200** (ArduPilot `SERIALx_PROTOCOL=2`, `SERIALx_BAUD=115`)
- **Local Hubs (mavlink-router)**  
  - **TCP 5760**: for the **runner** (reliable, no UDP peer learning)  
  - **UDP 14553** (Server): **Local hub** for tools (LiDAR → FC)  
  - **UDP 14555** (Server): **SegMAV TX** (router listens)  
  - **UDP 14601** (Client/Normal): **SegMAV RX** (router pushes FC traffic)

**Files used on Jetson:**  
- `~/segmav/guided_waypoint_runner_pymav.py` (runner, pymavlink)  
- `~/segmav/scan_to_obstacle_pymav.py` (LiDAR → OBSTACLE_DISTANCE, pymavlink)  
- `~/segmav/mavsegmav_merged.py` (SegMAV nudges)  

---

## One‑Time Setup
```bash
python3 -m pip install --user --upgrade "pymavlink>=2.4.40"
chmod +x ~/segmav/guided_waypoint_runner_pymav.py
chmod +x ~/segmav/scan_to_obstacle_pymav.py
```

Ensure `/etc/mavlink-router/main.conf` exposes the endpoints mentioned above (esp. **TCP 5760**). Minimal example:
```ini
[General]
TcpServerPort = 5760
ReportStats   = true
MavlinkDialect= ardupilotmega
Log           = /var/log/mavlink-router

[UartEndpoint telem]
Device = /dev/telem1
Baud   = 115200
FlowControl = 0

[UdpEndpoint LocalHub]
Mode   = Server
Address= 127.0.0.1
Port   = 14553

[UdpEndpoint segmav_tx]
Mode   = Server
Address= 127.0.0.1
Port   = 14555

[UdpEndpoint segmav_rx]
Mode   = Normal
Address= 127.0.0.1
Port   = 14601
```

---

## 0) Clean Slate
Stop services (if any) and kill leftovers so nothing fights the router or serial:
```bash
sudo -v
for s in mavlink-router mavros mavros-params ldlidar segmav segmav-lidar-bridge; do
  sudo systemctl stop "$s" 2>/dev/null || true
done
sudo pkill mavlink-routerd 2>/dev/null || true
pkill -f mavsegmav_merged.py 2>/dev/null || true
pkill -f scan_to_obstacle_pymav.py 2>/dev/null || true
pkill -f ldlidar_stl_ros_node 2>/dev/null || true
pkill -f roscore 2>/dev/null || true
pkill -f roslaunch 2>/dev/null || true
pkill -f rosrun 2>/dev/null || true
```

---

## Open a **NEW terminal** for each step

### Terminal A — Start ROS (needed for LiDAR `/scan`)
```bash
source /opt/ros/noetic/setup.bash
roscore
```

### Terminal B — Start **mavlink-router** (bridges FCU ⟷ apps)
```bash
source /opt/ros/noetic/setup.bash
sudo mavlink-routerd -v -c /etc/mavlink-router/main.conf
# Expect logs like:
#   Opened UART ... /dev/telem1 (115200)
#   Opened UDP Server ... 127.0.0.1:14553
#   Opened UDP Server ... 127.0.0.1:14555
#   Opened UDP Client ... 127.0.0.1:14601
#   Opened TCP Server ... [::]:5760
```

### Terminal C — Start LiDAR driver (publishes `/scan`)
```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch ldlidar_stl_ros ld19.launch
# Check (optional in another shell):
#   rostopic hz /scan   # should be ~9–10 Hz
```

### Terminal D — LiDAR → FC (**OBSTACLE_DISTANCE** via UDP)
```bash
source /opt/ros/noetic/setup.bash
python3 ~/segmav/scan_to_obstacle_pymav.py \
  --hub udpout:127.0.0.1:14553 \
  --sectors 72 --min 0.20 --max 12.0 --yaw_offset_deg 0
# If the Proximity ring is rotated in Mission Planner:
#   Ctrl-C and rerun with --yaw_offset_deg 90   (or -90 / 180)
```

### Terminal E — **SegMAV** (gentle velocity “nudges”)
```bash
source /opt/ros/noetic/setup.bash
python3 ~/segmav/mavsegmav_merged.py --headless
# Defaults in script: TX→14555 (router listens), RX←14601 (router pushes FC traffic)
```

### On your **PC** — Mission Planner (SiK radio)
1) **Plan** tab → load/create your mission → **Write** to the rover.  
2) If required by policy, **Arm** in Mission Planner (or let the runner arm if permitted).

### Terminal F — Run the **GUIDED** mission (pymavlink over **TCP 5760**)
```bash
source /opt/ros/noetic/setup.bash
python3 ~/segmav/guided_waypoint_runner_pymav.py \
  --hub tcp:127.0.0.1:5760 \
  --speed 1.2
# Add --arm ONLY if your FC arming policy allows arming from this SYSID; otherwise arm in MP.
```

> ✅ The rover now executes your uploaded mission in **GUIDED** mode; SegMAV sends velocity nudges in parallel; FC uses LiDAR **OBSTACLE_DISTANCE** for avoidance.

---

## Quick Health Checks
```bash
# Router sockets (UDP hubs + TCP 5760)
sudo ss -ltnp | egrep ':5760\b' || true
sudo ss -lunp | egrep ':(14553|14555|14601)\b' || true

# LiDAR topic rate
rostopic hz /scan

# In Mission Planner (Ctrl-F):
#   Mavlink Inspector → OBSTACLE_DISTANCE > 0 Hz
#   Proximity → 360° ring renders and updates
```

---

## Troubleshooting
**Runner prints “No HEARTBEAT”:**
- Make sure Terminal B shows **“Opened TCP Server … 5760”**.  
- Ensure nothing else is binding `:5760`:
  ```bash
  sudo ss -ltnp | grep :5760
  ```
- Verify FC serial is really on the expected port/baud. Quick direct‑serial test (stop router first):
  ```bash
  sudo pkill mavlink-routerd 2>/dev/null || true
  python3 - <<'PY'
from pymavlink import mavutil
print("Opening /dev/telem1 @115200 ...")
m = mavutil.mavlink_connection('/dev/telem1', baud=115200)
hb = m.wait_heartbeat(timeout=10)
print("Heartbeat:", hb)
PY
  ```
  - If this fails: check FC params (`SERIALx_PROTOCOL=2`, `SERIALx_BAUD=115`) and wiring/port mapping.
  - If this succeeds: restart the router and run the runner against `tcp:127.0.0.1:5760` again.

**Mission pull says count zero:**  
- In Mission Planner → **Plan** → click **Write** again to ensure a mission is on the FC.

**Proximity ring not moving:**  
- Confirm Terminal D is running and printing its startup line.  
- In Mission Planner → Ctrl‑F → **Mavlink Inspector** → verify `OBSTACLE_DISTANCE` > 0 Hz.  
- Adjust `--yaw_offset_deg` (+/‑90 or 180) if the ring is rotated.

---

## Once‑Only FC Parameters (ArduPilot)
- `PRX_TYPE = 5` (MAVLink `OBSTACLE_DISTANCE`)  
- `AVOID_ENABLE = 1`  
- `OA_TYPE = 1` (BendyRuler)  
- Optional: `AVOID_MARGIN = 1.0`

---

## Clean Shutdown
```bash
# Ctrl‑C in each terminal, then:
sudo pkill mavlink-routerd 2>/dev/null || true
pkill -f mavsegmav_merged.py 2>/dev/null || true
pkill -f scan_to_obstacle_pymav.py 2>/dev/null || true
pkill -f ldlidar_stl_ros_node 2>/dev/null || true
pkill -f roscore 2>/dev/null || true
```

---

### Notes
- Using **TCP 5760** for the runner avoids UDP peer‑learning hiccups; keep LiDAR + SegMAV on UDP as configured.  
- If you later consolidate hubs to **UDP 14550**, you can switch the LiDAR script’s `--hub` to `udpout:127.0.0.1:14550` (runner can stay on TCP).

**Happy rolling 🚗💨**
