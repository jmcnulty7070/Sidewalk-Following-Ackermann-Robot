
# Sidewalk GPS Follower + SegMAV System

This README is a **master document** that combines:
1. The Communication Plan
2. The File Structure Layout

---

## 📡 Communication Plan

### Overview
This system uses multiple communication channels to coordinate between:
- **Mission Planner** (on PC via SiK radio)
- **Pixhawk / CrossFlight FC**
- **Jetson (Nano/Orin) running SegMAV**
- **Sensors** (LiDAR LD19, Astra RGB camera)

All communication runs over **UDP links** and **MAVLink messages**.

### Communication Paths

#### 1. Waypoint Delivery (Mission Planner → Pixhawk → Jetson)
- Mission Planner sends **waypoints** over the **SiK radio (915 MHz)** → CrossFlight FC (Pixhawk-compatible).
- The Pixhawk stores waypoints and broadcasts them via MAVLink.
- Jetson (SegMAV script) listens on UDP (`udpin:0.0.0.0:14550`) through MAVLink Router.
- Waypoints are read by SegMAV for blending with vision-based navigation.

#### 2. Velocity Control (Jetson → Pixhawk)
- SegMAV generates **velocity commands** from camera segmentation + GPS heading.
- Commands sent as **MAVLink SET_POSITION_TARGET_LOCAL_NED** messages.
- Forwarded via UDP → MAVLink Router → Pixhawk TELEM1.

#### 3. LiDAR Data (Jetson → Pixhawk)
- LD19 LiDAR publishes `/scan` via ROS driver.
- `lidar_to_mavlink_bridge.py` converts scan ranges → MAVLink `OBSTACLE_DISTANCE`.
- Sent to Pixhawk, which runs **Bendy Ruler** obstacle avoidance.

#### 4. Status Feedback (Pixhawk → Jetson + Mission Planner)
- Heartbeats and status messages broadcast on **UDP 14550**.
- Jetson SegMAV listens for state (GUIDED mode, RC5 switch, etc.).
- Mission Planner displays telemetry, map, and vehicle status.

---

## 📂 File Structure Layout

### On Jetson (`~/segmav`)
```
~/segmav/
├── mavsegmav_merged.py      # Main script (Segmentation + GPS waypoint blending)
├── segmav.py                # Segmentation + steering logic
├── segmav.service           # systemd service for auto-start (optional)
├── lidar_to_mavlink_bridge.py # ROS node: converts LiDAR → MAVLink
├── README.md                # Project documentation
├── README_communication_plan.md (merged here)
├── README_file_structure.md (merged here)
└── udev/                    # udev rules for persistent device names
    ├── 99-astra.rules       # Astra RGB camera → /dev/astra_rgb
    ├── 99-ld19.rules        # LD LiDAR → /dev/ldlidar
    └── 99-telem.rules       # MAVLink TELEM1 FTDI → /dev/telem1
```

### On Pixhawk (CrossFlight / ArduPilot)
- Receives:
  - Waypoints from Mission Planner (via SiK Radio).
  - Velocity commands from Jetson (via MAVLink UDP → TELEM1).
  - Obstacle data from Jetson LiDAR bridge.
- Runs:
  - **Bendy Ruler** for obstacle avoidance.
  - **Guided mode** for blending waypoint + vision control.

### On Mission Planner (PC)
- Connects to CrossFlight via **SiK Radio COM Port**.
- Sends waypoints, parameter updates, and telemetry requests.
- Displays:
  - Vehicle position (from GPS).
  - Proximity/LiDAR avoidance (via OBSTACLE_DISTANCE).
  - SegMAV velocity blending in GUIDED mode.

---

## 🔑 Udev Rules Summary
- `/dev/astra_rgb` → Astra RGB camera
- `/dev/ldlidar`   → LD19 LiDAR
- `/dev/telem1`    → FTDI MAVLink to Pixhawk TELEM1

---

## 🚀 Launch Workflow

1. **Start MAVLink Router** (auto via systemd):
   ```bash
   mavlink-routerd -e 127.0.0.1:14550 /dev/telem1
   ```

2. **Run SegMAV**:
   ```bash
   cd ~/segmav
   python3 mavsegmav_merged.py --device=udp:127.0.0.1:14550 --preview
   ```

3. **Check Mission Planner**:
   - Connect via SiK Radio.
   - Upload waypoints.
   - Switch to GUIDED mode (RC5 switch controls SegMAV blending).

---

## ✅ Summary
This system blends:
- **GPS waypoint following** (from Mission Planner).
- **Sidewalk vision tracking** (Astra RGB segmentation).
- **Obstacle avoidance** (LD19 LiDAR + Bendy Ruler).

All coordinated via **UDP + MAVLink** between Mission Planner, Pixhawk, and Jetson.

---
