
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
## 📁 FINAL FOLDER STRUCTURE (Jetson)

Here’s what your Jetson Nano home directory should look like when done:
```
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
└── catkin_ws/
    └── src/
        ├── segmav_bridge/
        │   ├── CMakeLists.txt
        │   ├── package.xml
        │   ├── launch/
        │   │   └── segmav_bridge.launch          # Starts lidar_to_mavlink_bridge.py
        │   └── scripts/
        │       └── lidar_to_mavlink_bridge.py    # ROS node that converts LiDAR to MAVROS message
        │
        └── ldlidar_stl_ros/
            ├── CMakeLists.txt
            ├── package.xml
            ├── launch/
            │   └── ld19.launch                   # Launch file for LD LiDAR (can rename to d500.launch if needed)
            └── src/
                └── ...                           # LiDAR driver source files
```
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
# LDROBOT D500 LiDAR (STL‑19P/D500) ROS1 Integration

This guide explains how to set up the **LDROBOT D500 (STL‑19P/D500) LiDAR Kit** using the official ROS 1 driver. This sensor works great for projects like **obstacle avoidance**, **SLAM**, and **Bendy Ruler navigation** on **ROS Noetic**.

---

## ✅ Supported ROS Driver

Use the official **`ldlidar_stl_ros`** package from LDROBOT.

**Supports:**
- Models: LD06, LD19 (D500 is compatible)
- ROS1 and ROS2
- Publishes to `/scan` topic
- Includes launch files and serial communication

GitHub Repository: [ldlidar_stl_ros](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros)

---

## 🛠️ Install Instructions (8th Grade Level)

### 1. Open a Terminal

Open a terminal on your Jetson Nano or computer running ROS Noetic.

### 2. Create a new ROS workspace and download the driver

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
rosdep install --from-paths src --ignore-src -r -y

Save lidar_to_mavlink_bridge.py here

chmod +x src/segmav_bridge/scripts/lidar_to_mavlink_bridge.py

git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros.git
```

### 3. Install missing dependencies

```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Give USB permission to LiDAR port

Substitute the actual port if not `/dev/ttyUSB0`:

```bash
sudo chmod 777 /dev/ttyUSB0
```

### 5. Build the workspace

```bash
catkin_make
source devel/setup.bash
```

### 6. Launch the driver

Use the existing launch file for LD19 (D500 works the same):

```bash
roslaunch ldlidar_stl_ros ld19.launch
```

> ✅ If there’s a `d500.launch`, you can use that instead.

### 7. Confirm LiDAR is working

```bash
rostopic echo /scan
```

You should see live distance readings appear in your terminal.

---

## 💡 Why This Driver Works for D500

- ✅ Official support for LD series (LD19 ≈ D500)
- ✅ ROS1-compatible
- ✅ `/scan` topic is standard for obstacle avoidance or MAVROS integration
- ✅ No need for custom code

---

## 🧩 Integrates With:

- `lidar_to_mavlink_bridge.py`
- `bendy_ruler` obstacle avoidance
- Jetson Nano + MAVROS

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
