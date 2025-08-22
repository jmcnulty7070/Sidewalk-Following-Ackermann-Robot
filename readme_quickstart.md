# Quickstart Guide – Sidewalk GPS Robot Service Scripts

This repo contains three helper scripts to manage your SegMAV + MAVROS + MAVLink Router + LDLiDAR stack.
They are designed to let you **reset**, **start manually**, or **reset+start** in one go.

---

## 1. Reset Script

**File:** `reset_robot_services.sh`  
**Purpose:** Stop and disable all related services for a clean slate.

### What it does (in order)
- Refreshes sudo credentials (prompts once if needed).
- Kills any lingering processes: `mavros`, `mavlink-router`, `mavsegmav`, `ldlidar`.
- Shows anything still running that matches those names.
- Stops and disables these services:
  - `ldlidar.service`
  - `mavros.service`
  - `mavlink-router.service`
  - `segmav.service`
- Prints a list of all **enabled** unit files on the system.
- Prints any still-running units that match the stack (just in case).

### Use it
```bash
chmod +x reset_robot_services.sh
./reset_robot_services.sh
```

---

## 2. Manual Startup Script

**File:** `start_robot_manual.sh`  
**Purpose:** Start the stack step by step, following the Sidewalk GPS Robot Cheat Sheet.

### What it does
- **MAVLink Router** – restarts service, checks UDP ports:
  - `14550` → Mission Planner
  - `14551` → SegMAV (+ MAVROS)
- **MAVROS** – launches bound to `udp://0.0.0.0:14551@` (logs → `~/mavros.log`).
- **LiDAR** – launches LDLiDAR LD19 node (logs → `~/ldlidar.log`) and verifies `/scan`.
- **SegMAV** – launches `mavsegmav_merged.py` with preview mode (logs → `~/segmav.log`).
- **GUIDED Waypoint Streamer** – prints the optional command for you to run manually.

### Use it
```bash
chmod +x start_robot_manual.sh
./start_robot_manual.sh
```

---

## 3. Merged Reset + Start Script

**File:** `robot_stack_reset_and_start.sh`  
**Purpose:** One-shot reset + startup for the full stack.

### What it does
- Kills leftovers (`mavros|mavlink-router|mavsegmav|ldlidar`).
- Stops services and (by default) **disables** them:
  - `ldlidar`, `mavros`, `mavlink-router`, `segmav`
- Shows enabled unit files.
- Starts: **mavlink-router → MAVROS → (optional) LDLiDAR → SegMAV**.
- Prints quick checks and optional Proximity relay command.

### Usage
```bash
chmod +x robot_stack_reset_and_start.sh
./robot_stack_reset_and_start.sh             # preview UI on, disables services, starts LiDAR
./robot_stack_reset_and_start.sh --headless  # no SegMAV preview window
./robot_stack_reset_and_start.sh --no-disable # stop services but don't disable them
./robot_stack_reset_and_start.sh --skip-lidar # skip starting LDLiDAR
```

### Logs
- `~/mavros.log`
- `~/ldlidar.log`
- `~/segmav.log`

---

## Notes
- All scripts require `sudo` (may prompt once per run).
- Ensure your ROS workspace is sourced (`source ~/catkin_ws/devel/setup.bash`) before starting.
- Device paths (like `/dev/Astra_rgb` or `/dev/ldlidar`) assume you’ve set up udev rules.

