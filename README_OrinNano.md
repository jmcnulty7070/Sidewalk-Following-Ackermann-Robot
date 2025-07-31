# 🚗 Sidewalk-Following Ackermann Robot — Orin Nano Edition

This guide walks you through installing, wiring, and launching the sidewalk-following robot system on the **NVIDIA Orin Nano** Developer Kit using JetPack 5.1.2+. It includes camera-based semantic navigation, LiDAR obstacle detection, and MAVLink control to the Pixhawk (CrossFlight) flight controller.

---

## ✅ Why Use Orin Nano?

- 🔥 Faster inference (up to 20× faster than Jetson Nano)
- 🧠 Runs Deep Learning segmentation at higher FPS
- 🧰 Supports full ROS Noetic + RealSense + MAVROS
- 🎥 Great for RealSense, USB, or CSI camera input

---

## 🔧 Hardware Setup

| Component                  | Notes                                                   |
|---------------------------|----------------------------------------------------------|
| Orin Nano 4GB or 8GB       | Flash with JetPack 5.1.2 (Ubuntu 20.04)                 |
| RealSense D435 or IMX219  | For camera-based segmentation                            |
| Pixhawk CrossFlight       | Flight controller with MAVLink support (TELEM1)         |
| Logic Level Converter     | Required: Orin = 3.3V, Pixhawk = 5V                      |
| LD LiDAR (or RPLIDAR)     | Publishes `/scan` for obstacle detection                |
| Radiolink M10N GPS        | TELEM2 or GPS/I2C via breakout                           |
| 915MHz Telemetry Radio    | Connect to TELEM1/2 and Mission Planner                 |

---

## 🖥️ JetPack + Software Setup

### 1. Flash JetPack 5.1.2+ on Orin Nano

Use [NVIDIA SDK Manager](https://developer.nvidia.com/embedded/jetpack) or SD image.

### 2. Install ROS Noetic

```bash
sudo apt update
sudo apt install ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. Install MAVROS

```bash
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash install_geographiclib_datasets.sh
```

### 4. Clone and Build Sidewalk-Follower

```bash
cd ~
git clone https://github.com/YOUR_USERNAME/sidewalk-follower.git
cd sidewalk-follower
catkin_make
```

> Replace `YOUR_USERNAME` with your GitHub repo if applicable.

---

## ⚡ UART Wiring to Pixhawk (CrossFlight)

Use a **logic level converter** to safely bridge 3.3V ↔ 5V.

| Jetson Pin | LLC Low Side (3.3V) | LLC High Side (5V) | Pixhawk TELEM1 |
|------------|----------------------|---------------------|----------------|
| Pin 8 (TX) | LV-TX                | HV-RX               | RX             |
| Pin 10 (RX)| LV-RX                | HV-TX               | TX             |
| Pin 6 (GND)| GND                  | GND                 | GND            |
| Pin 1 (3.3V)| LV (ref)           | —                   | —              |
| 5V (from PX)| —                  | HV (ref)            | 5V             |

Pixhawk Setup in Mission Planner:

```text
SERIAL1_PROTOCOL = 2
SERIAL1_BAUD     = 57600
```

---

## 🚀 Launch Order

1. **Start ROS Core**
```bash
roscore
```

2. **Start MAVROS**
```bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyTHS1:57600
```

3. **Start LiDAR Node**
```bash
roslaunch ldlidar_stl ldlidar.launch
```

4. **Start LiDAR → MAVLink bridge**
```bash
rosrun your_package lidar_to_mavlink_bridge.py
```

5. **Start Semantic Navigation (mavsegmav.py)**
```bash
python3 mavsegmav.py --input-flip=rotate-180 --rc=9 --vel=2.5 --device=/dev/ttyTHS1 --baud=57600 csi://0
```

> Replace `csi://0` with `/dev/video0` if using USB/RealSense.

6. **Optional: RViz**
```bash
rviz
```

---

## 🧠 Dynamic Velocity Control

The robot:

- Speeds up when the sidewalk is wide
- Slows when narrow or obscured
- Slows when obstacle detected in `/scan`

Tunable in `mavsegmav.py`:

```python
VELOCITY_BASE = 1.0
VELOCITY_MAX = 2.5
OBSTACLE_RANGE = 0.8
WIDTH_SCALE = 0.4
```

---

## 📡 Adding Telemetry Radio for Mission Planner

To connect Orin Nano + Pixhawk to your PC wirelessly:

| Port         | Use                       |
|--------------|----------------------------|
| TELEM1        | MAVROS / Jetson connection |
| TELEM2        | Radio 915MHz → PC          |
| GPS/I2C       | Radiolink M10N GPS         |

Use a 915MHz SiK radio set on your PC’s Mission Planner and TELEM2 on Pixhawk. Confirm:

```text
SERIAL2_PROTOCOL = 1  (MAVLink)
SERIAL2_BAUD     = 57600
```

---

## 📦 Optional: Autostart with systemd

File: `/etc/systemd/system/segmav.service`

```ini
[Unit]
Description=Start mavsegmav
After=network.target

[Service]
ExecStart=/usr/bin/python3 /home/robot/mavsegmav.py
Restart=always
User=robot

[Install]
WantedBy=multi-user.target
```

Enable it:
```bash
sudo systemctl enable segmav
```

---

## ✅ Confirmed Working on Orin Nano

✔️ Semantic Segmentation  
✔️ MAVROS + MAVLink  
✔️ LiDAR obstacle detection  
✔️ Dynamic steering + speed  
✔️ GPS + Telemetry  
✔️ CSI and USB camera inputs  
✔️ Full ROS Noetic compatibility