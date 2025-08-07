
# 🛴 Sidewalk-Following Robot with GPS + Vision + LiDAR

This guide shows you how to set up your robot so it:
- Follows sidewalks using a camera and AI
- Stays on a path using GPS waypoints (Mission Planner)
- Slows down for obstacles using LiDAR

Works on: Jetson Nano/Orin, Pixhawk or CrossFlight, ROS Noetic, ArduPilot Rover

---

## 📦 What You Need

✅ Jetson Nano or Orin Nano  
✅ Pixhawk or CrossFlight flight controller  
✅ A camera (CSI or USB)  
✅ A LiDAR (like RPLIDAR or YDLIDAR)  
✅ GPS module  
✅ Ubuntu 20.04 with JetPack 5.1.2  
✅ ROS Noetic  
✅ MAVROS  
✅ ArduPilot firmware (Rover)

---
~/                 ← Your Jetson Nano home
├── jetson-inference/
│   ├── build/
│   │   ├── aarch64/bin/
│   │   │   └── segnet.py  ← segmentation executable
│   │   ├── CMakeFiles/
│   ├── data/
│   │   └── networks/
│   │       └── fcn-resnet18-cityscapes-1024x512/
│   │           ├── labels.txt
│   │           ├── fcn_resnet18.onnx
│   │           └── other model files
│   └── python/
│       └── jetson_inference/
│       └── jetson_utils/
│
├── segmav/
│   ├── mavsegmav.py
│   ├── segmav.py
│   ├── segmav.service
│   ├── screenshot.png
│   └── README.md
│
└── catkin_ws/           ← You’ll create this later for ROS
    ├── src/
    └── build/
---
## 🔧 Step-by-Step Setup

### 1. Flash Jetson with JetPack
- Use NVIDIA SDK Manager or flash an SD card with JetPack 5.1.2
- Login and open a terminal

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

---

## 🧠 Clone and Build the Code

Clone the repo into your home folder:

```bash
cd ~
git clone https://github.com/YOUR_USERNAME/sidewalk-follower.git
cd sidewalk-follower
```

### Make it executable:

```bash
chmod +x *.py
```

---

## 🤖 Files in This Project

| File | What it does |
|------|--------------|
| `mavsegmav_merged.py` | Main robot brain: camera + GPS + LiDAR control |
| `segmav.py` | Handles AI (semantic segmentation) |
| `lidar_to_mavlink_bridge.py` | Sends LiDAR data to ArduPilot |
| `segmav.service` | Lets you auto-start the robot script at boot |
| `segmav_bridge.launch` | ROS launch file to run the LiDAR bridge |

---

## 🎯 Running the LiDAR Bridge (ROS)

This sends LiDAR range to `/mavros/distance_sensor/custom`:

```bash
roslaunch segmav_bridge segmav_bridge.launch
```

You must have `lidar_to_mavlink_bridge.py` in a ROS package named `segmav_bridge`.

---

## 🧠 Running the Robot Script

This runs vision + GPS + LiDAR control:

```bash
python3 mavsegmav_merged.py --input=csi://0 --headless
```

You can remove `--headless` if you want to see the debug video overlay.

---

## 🕹️ Switching Modes with RC

Set up a 3-position switch (e.g. Channel 9 or 10):

| RC Position | Mode |
|-------------|------|
| Low (1000)  | Stop |
| Middle (1500) | Record video |
| High (2000) | Navigation (drive using GPS + camera)

---

## 🗺️ Using Waypoints

1. Open **Mission Planner**
2. Switch to `GUIDED` mode
3. Add waypoints along your route
4. Upload the mission
5. The robot will blend GPS headings with the sidewalk AI

> ✅ Your robot will stay on the sidewalk but still follow GPS direction!

---

## 🔄 Auto-Start at Boot (Optional)

Edit and copy `segmav.service`:

```ini
[Unit]
Description=SEGMAV Control Service
After=network.target

[Service]
Environment="LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1"
WorkingDirectory=/home/robot/sidewalk-follower
ExecStart=/usr/bin/python3 mavsegmav_merged.py --input=csi://0 --headless
Restart=on-failure
User=robot

[Install]
WantedBy=multi-user.target
```

Enable it:

```bash
sudo cp segmav.service /etc/systemd/system/
sudo systemctl daemon-reexec
sudo systemctl enable segmav.service
sudo systemctl start segmav.service
```

---

## 🧪 Testing Tips

- Use `--input=file://video.mp4` to test with a saved video
- Use `--output=rtp://192.168.x.x:5400` to stream to another computer
- Set `--targetclass` to your sidewalk class ID (default = 3)

---

## 📍 Final Folder Structure

```
~/sidewalk-follower/
├── mavsegmav_merged.py
├── segmav.py
├── lidar_to_mavlink_bridge.py
├── segmav.service
├── segmav_bridge/
│   ├── launch/
│   │   └── segmav_bridge.launch
│   └── src/
│       └── lidar_to_mavlink_bridge.py
├── README.md
```

---

## 🧠 Credits

- Based on ArduPilot + Jetson AI
- Inspired by: https://www.youtube.com/watch?v=yGB9uLD4dkM&t=183s
