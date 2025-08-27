# segmav (ROS Noetic) – PRX Publisher + MAVLink Router layout

This repo gives you a ready-to-use **ROS package** and **system services** to run:
- LD19 LiDAR → `/scan` (via `ldlidar_stl_ros`)
- `/scan` → **OBSTACLE_DISTANCE** over MAVLink (Proximity/PRX for Bendy Ruler)
- `mavsegmav_merged.py` sending velocity nudges
- **mavlink-router** multiplexing TELEM1 (FTDI) to multiple UDP ports

Tested target: Jetson (Ubuntu 20.04, ROS Noetic)

---

## Folder layout

```
segmav_pkg/
├─ package.xml
├─ CMakeLists.txt
├─ launch/
│  └─ prx.launch
├─ src/
│  ├─ scan_to_obstacle_distance.py
│  └─ mavsegmav_merged.py   # placeholder – replace with your real script
├─ bin/
│  └─ segmav-prx.sh         # wrapper to start PRX node
├─ systemd/
│  └─ segmav-prx.service    # systemd service unit
├─ config/
│  └─ mavlink-router/
│     └─ main.conf.sample   # 14550 (GCS), 14551 (mavsegmav), 14552 (PRX)
├─ udev/
│  └─ 99-segmav-serial.rules.sample
├─ .gitignore
└─ LICENSE
```

---

## Quick start

### 0) Prereqs
- ROS Noetic installed
- `ldlidar_stl_ros` package installed and a working LD19 on USB
- `mavlink-router` installed as a service

### 1) Catkin package install
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
# Copy segmav_pkg into this folder and rename to segmav
cp -r /path/to/segmav_pkg ./segmav
cd .. && catkin_make
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

### 2) Configure mavlink-router
```bash
sudo mkdir -p /etc/mavlink-router
sudo cp ~/catkin_ws/src/segmav/config/mavlink-router/main.conf.sample /etc/mavlink-router/main.conf
sudo systemctl enable --now mavlink-router
```

### 3) (Recommended) Create stable device names with udev
First, find attributes for your FTDI and LD19:
```bash
# Plug in FTDI (TELEM1) and LD19, then check:
udevadm info -a -n /dev/ttyUSB0 | head -n 40
udevadm info -a -n /dev/ttyUSB1 | head -n 40
# If one enumerates as /dev/ttyACM0, use that node instead
```

Edit `udev/99-segmav-serial.rules.sample` and pick **one** rule per device that matches your VID/PID or serial.
Then install:
```bash
sudo cp ~/catkin_ws/src/segmav/udev/99-segmav-serial.rules.sample /etc/udev/rules.d/99-segmav-serial.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
# Replug devices and verify:
ls -l /dev/telem1 /dev/ldlidar
```

### 4) FC parameters (ArduPilot Rover)
```
SERIAL1_PROTOCOL = 2     # MAVLink2 on TELEM1
SERIAL1_BAUD     = 921   # 921600
# If present in your firmware: ensure HW flow control OFF for TELEM1
# SERIAL1_OPTIONS -> HWFC bit off OR BRD_SER1_RTSCTS = 0

PRX1_TYPE        = 2     # MAVLink PRX from the Jetson
OA_TYPE          = 1     # Bendy Ruler
AVOID_ENABLE     = 7     # act on all sources
```

### 5) Run manually (for testing)
In terminal A (LiDAR driver):
```bash
roslaunch ldlidar_stl_ros ldlidar.launch
```

In terminal B (PRX publisher):
```bash
rosrun segmav scan_to_obstacle_distance.py
```

In terminal C (SegMAV nudges):
```bash
rosrun segmav mavsegmav_merged.py --device=udp:127.0.0.1:14551
```

Mission Planner → **UDP** to `JETSON_IP:14550`. Check **Data → Proximity** for the PRX ring.

### 6) Install as a service (auto-start)
```bash
sudo cp ~/catkin_ws/src/segmav/bin/segmav-prx.sh /usr/local/bin/segmav-prx.sh
sudo chmod +x /usr/local/bin/segmav-prx.sh

sudo cp ~/catkin_ws/src/segmav/systemd/segmav-prx.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now segmav-prx
sudo systemctl status segmav-prx --no-pager
```

---

## Ports & endpoints
- **/dev/telem1** (FTDI) ←→ mavlink-router UART endpoint @ 921600, no HW flow control
- **UDP 14550** → GCS (Mission Planner)
- **UDP 14551** → `mavsegmav_merged.py` (nudges)
- **UDP 14552** → PRX publisher (`scan_to_obstacle_distance.py`)

---

## Notes
- `scan_to_obstacle_distance.py` publishes **OBSTACLE_DISTANCE** at the `/scan` update rate. Consider 10–15 Hz.
- Set `YAW_ALIGN_DEG` inside the script if LiDAR forward isn’t robot forward.
- For high USB stability with Jetson, a powered hub is recommended for LD19.

---

MIT License © 2025
