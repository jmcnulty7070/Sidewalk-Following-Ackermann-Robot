
# SegMAV — **GUIDED Mission** with Vision Nudges & LiDAR Avoidance
_Updated workflow using mavlink-router + pymavlink (no MAVROS needed for control)._

This README gives you **start-to-finish** steps (8th-grade level), the **exact commands**, and explains the **file changes** in plain English.

---

## 🧭 What you’ll run (big picture)

- **mavlink-router** — the “switchboard” that fans out the flight controller’s MAVLink stream  
- **LDLiDAR driver** — publishes `/scan` in ROS  
- **LiDAR→MAVLink bridge** — turns `/scan` into `OBSTACLE_DISTANCE` messages  
- **SegMAV** — sends small **velocity nudges** (stay on sidewalk)  
- **GKeep avoidance on, but not overbearing:

AVOID_ENABLE=1, OA_TYPE=1 (BendyRuler), AVOID_MARGIN≈0.30 m (≈1 ft).

If it’s still too cautious, lower AVOID_MARGIN a bit (e.g., 0.20–0.25 m).

Worst case for testing: set OA_TYPE=0 to disable avoidance temporarily (but then you rely only on SegMAV + mission runner).UIDED runner** — pulls the mission from the FC and walks waypoints using Rover-friendly commands

-Mission Planner (GCS)
        │  (MAVLink mission upload)
        ▼
-mavlink-router :14550  (UDP server for GCS)
        │
        ▼
-/dev/telem1  ↔  Flight Controller  (mission is stored here)
        ▲
        │  (MISSION_REQUEST_LIST / MISSION_ITEM[_INT] replies)
        │
-Jetson runner  ↔  mavlink-router :5760 (TCP)
-(guided_waypoint_runner_pymav.py pulls waypoints from the FC)


What [UdpEndpoint GCS_14550] does
[UdpEndpoint GCS_14550]
Mode    = Server
Address = 0.0.0.0
Port    = 14550


chmod +x ~/segmav/guided_waypoint_runner_pymav.py

chmod +x ~/segmav/scan_to_obstacle_pymav.py


---

## 🔌 Ports & wiring (cheat-sheet)

- **Serial** FC link: `/dev/telem1` @ **115200** (router opens it)  
- **Router TCP server**: `:5760` (used by the GUIDED runner)  
- **Router UDP servers**:  
  - `:14553` — “LocalHub” for LiDAR bridge input  
  - `:14555` — SegMAV **TX** (we send nudges here)  
- **Router UDP client**:  
  - `→ 127.0.0.1:14601` — SegMAV **RX** (router pushes FC telemetry here)  

---

## 0) Clean slate (stop any old daemons/processes)

```bash
# stop services if present (ignore errors)
sudo systemctl stop mavlink-router 2>/dev/null || true

# kill any leftovers
sudo pkill mavlink-routerd 2>/dev/null || true
pkill -f mavsegmav_merged.py 2>/dev/null || true
pkill -f guided_waypoint_runner_pymav.py 2>/dev/null || true
pkill -f scan_to_obstacle_pymav.py 2>/dev/null || true
pkill -f ldlidar_stl_ros 2>/dev/null || true
pkill -f roscore 2>/dev/null || true
```

---

## 1) Start **ROS core** (Terminal A)

```bash
source /opt/ros/noetic/setup.bash
roscore
```

Leave it running.

---

## 2) Start **mavlink-router** (Terminal B)

```bash
# in case it was running:
sudo pkill mavlink-routerd 2>/dev/null || true

# start router with your main.conf
sudo mavlink-routerd -v -c /etc/mavlink-router/main.conf
```

**You should see** it open: `/dev/telem1` @115200, `TCP :5760`, `UDP :14553`, `UDP :14555`, and a client to `→ 14601`.

Quick check:
```bash
sudo ss -lunp | egrep ':(14550|14553|14555|14601)\b' || true
sudo ss -ltnp | egrep ':5760\b' || true
```

---

## 3) Start the **LDLiDAR** (Terminal C)

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch ldlidar_stl_ros ld19.launch
```
(Optional) If the Proximity ring is rotated in Mission Planner, try --yaw_offset_deg 90 (or -90, 180).

Check it’s alive:
```bash
rostopic hz /scan   # expect ~9–10 Hz
```

---

## 4) Start **LiDAR → OBSTACLE_DISTANCE** bridge (Terminal D)

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

python3 ~/segmav/scan_to_obstacle_pymav.py \
  --hub udpout:127.0.0.1:14553 \
  --sectors 72 --min 0.20 --max 0.61 --fov_deg 120 --yaw_offset_deg 0
```

> If the Proximity ring looks rotated in Mission Planner, restart with `--yaw_offset_deg 90` (or `-90`, `180`).

---

## 5) Start **SegMAV** (velocity nudges) (Terminal E)

**Use exactly this command (as requested):**
```bash
python3 ~/segmav/mavsegmav_merged.py \
  --tx-hub udpout:127.0.0.1:14555 \
  --rx-hub udpin:0.0.0.0:14601 \
  --vel 0.6 --nudge-hz 2.0 --nudge-ttl 0.3 --lat-k 0.6 --gps-weight 0.3 --preview
```
- If your revised script doesn’t support `--preview`, remove it or use `--headless` to disable overlay.  
- RC switch (ch10 by default): **Low** = stop, **Mid** = record, **High** = start Seg+GPS nudges.

---

## 6) Mission Planner (on your PC)

- In **Plan**, create/load your mission and **Write** to the rover.  
- **Ctrl-F → MAVLink Inspector**: confirm `OBSTACLE_DISTANCE` > 0 Hz.  
- **Ctrl-F → Proximity**: 360° ring should render.

---

## 7) Run the **GUIDED mission runner** (Terminal F)

```bash
source /opt/ros/noetic/setup.bash
python3 ~/segmav/guided_waypoint_runner_pymav.py \
  --hub tcp:127.0.0.1:5760 \
  --speed 0.6 --lookahead 2.0 --accept 0.8
```

- Pulls **NAV_WAYPOINT (16)** from the FC.  
- Drives in **GUIDED** using **SET_POSITION_TARGET_GLOBAL_INT** (correct for Rover).  
- Sends a **“carrot”** 2 m ahead along the sidewalk, refreshed ~**2 Hz**.  
- Arrival: **acceptance** OR **overshoot** OR **timeout**.  

> Start at 0.6 m/s; increase to 0.8–1.0 when stable.

---

## ✅ Quick health checks

```bash
# Router & sockets
sudo ss -lunp | egrep ':(14550|14553|14555|14601)\b' ; sudo ss -ltnp | egrep ':5760\b'

# LiDAR topic rate
rostopic hz /scan

# OBSTACLE_DISTANCE in MP
#   Ctrl-F → MAVLink Inspector (should tick)
#   Ctrl-F → Proximity ring (should draw)
```

---

## 🧪 Safety bubble & “see” distance

- Bridge sends LiDAR ranges **0.20 → 12.0 m** (`--min/--max`).  
- FC clearance via **`AVOID_MARGIN`**; set to **0.30 m** (≈ 1 ft).  
  - In MP: `AVOID_ENABLE=1`, `OA_TYPE=1 (BendyRuler)`, `PRX_TYPE=5`, `AVOID_MARGIN=0.30`.

If you **must** make the FC only care about very close things, you *can* run the bridge with `--max 0.61` (2 ft), but it’ll feel last‑second. Better to keep `--max 12.0` and let `AVOID_MARGIN` handle how early to dodge.

---

## 🧯 Fixing “PreArm: Proximity”

Start the bridge **before arming** and verify in MP that `OBSTACLE_DISTANCE` is ticking.  
If not:
- Bridge `--hub udpout:127.0.0.1:14553` matches router `:14553`
- Router is running and shows that port
- Try `--yaw_offset_deg` = `90`, `-90`, or `180` if the ring is rotated

---

## 🧰 What changed in the code (8th-grade level)

### `guided_waypoint_runner_pymav.py` (runner)
- **Before**: used `DO_REPOSITION` — Rover often ignored it and kept going straight.  
- **Now**: uses **`SET_POSITION_TARGET_GLOBAL_INT`**, which Rover understands for GUIDED.  
- Sends a moving **target (“carrot”)** 2 m ahead and refreshes it at **2 Hz** so the rover follows the **sidewalk line**, not a far-away dot.

### `scan_to_obstacle_pymav.py` (LiDAR bridge)
- Turns `/scan` into **72 slices** around the rover and sends the **closest distance** in each slice using **`OBSTACLE_DISTANCE`**.  
- Packs data in the **right units and sizes** (cm, uint16 array), so no more `ubyte` errors.  
- `--yaw_offset_deg` rotates the ring if “front” is off.

### `mavsegmav_merged.py` (nudges)
- **Before**: could send strong position/yaw commands and fight the runner.  
- **Now**: sends **small, short pulses of velocity** in the rover’s body frame — like gentle taps.  
  - Forward speed near what you set (e.g., `--vel 0.6`).  
  - Sideways capped to **±0.35 m/s** so it doesn’t overpower the mission.  
  - **2 nudges/sec**, each lasting **~0.3 s**, so the runner remains in charge.

Result: Follow the mission **and** stay centered on the sidewalk without wandering off.

---

## 🛑 Stopping everything

Press **Ctrl-C** in each terminal, or:

```bash
sudo pkill mavlink-routerd 2>/dev/null || true
pkill -f mavsegmav_merged.py 2>/dev/null || true
pkill -f guided_waypoint_runner_pymav.py 2>/dev/null || true
pkill -f scan_to_obstacle_pymav.py 2>/dev/null || true
pkill -f ldlidar_stl_ros 2>/dev/null || true
pkill -f roscore 2>/dev/null || true
```

---

## Appendix — Example router `/etc/mavlink-router/main.conf`

```ini
[General]
TcpServerPort = 5760
ReportStats   = true
MavlinkDialect= ardupilotmega
Log           = /var/log/mavlink-router

[UartEndpoint telem1]
Device = /dev/telem1
Baud   = 115200
FlowControl = 0

[UdpEndpoint LocalHub_14553]
Mode    = Server
Address = 127.0.0.1
Port    = 14553

[UdpEndpoint SegMAV_TX_14555]
Mode    = Server
Address = 127.0.0.1
Port    = 14555

[UdpEndpoint SegMAV_RX_14601]
Mode    = Normal
Address = 127.0.0.1
Port    = 14601

[UdpEndpoint GCS_14550]   # optional for Mission Planner / QGC
Mode    = Server
Address = 0.0.0.0
Port    = 14550
```
