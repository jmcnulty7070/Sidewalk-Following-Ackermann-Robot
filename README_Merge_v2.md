# 🛴 Sidewalk-Following Robot with GPS + Vision + LiDAR

This guide shows you how to set up your robot so it:

- Follows sidewalks using a camera and AI
- Stays on a path using GPS waypoints (Mission Planner)
- Slows down for obstacles using LiDAR (LDROBOT D500, ROS Noetic)

**Tested on:** Jetson Nano/Orin Nano, Pixhawk or CrossFlight, ROS Noetic, ArduPilot Rover

---

## 📦 What You Need

- Jetson Nano or Orin Nano  
- Pixhawk or CrossFlight flight controller  
- Camera (CSI or USB, e.g., Jetson IMX219-160, or RealSense)  
- LDROBOT D500 LiDAR (or compatible LD06/LD19 model)  
- GPS module (Yahboom, Radiolink, or similar)  
- Ubuntu 20.04 with JetPack 5.1.2  
- ROS Noetic  
- MAVROS  
- ArduPilot firmware (Rover)

---

## 📁 FINAL FOLDER STRUCTURE (Jetson + ROS)

Here’s what your Jetson Nano/Orin home directory and ROS workspace should look like:

~/ ← Your Jetson home
├── jetson-inference/
│ ├── build/
│ ├── data/
│ └── python/
│
├── sidewalk-follower/
│ ├── mavsegmav_merged.py
│ ├── segmav.py
│ ├── lidar_to_mavlink_bridge.py
│ ├── segmav.service
│ └── README.md
│
└── catkin_ws/
└── src/
├── segmav_bridge/
│ ├── CMakeLists.txt
│ ├── package.xml
│ ├── launch/
│ │ └── segmav_bridge.launch
│ └── scripts/
│ └── lidar_to_mavlink_bridge.py
│
└── ldlidar_stl_ros/
├── CMakeLists.txt
├── package.xml
├── launch/
│ └── ld19.launch
└── src/
└── ... # LiDAR driver files

yaml
Copy
Edit

---

## 🔧 Step-by-Step Setup

### 1. Flash Jetson with JetPack

- Use NVIDIA SDK Manager or SD card image for JetPack 5.1.2
- Login and open a terminal

### 2. Install ROS Noetic

```bash
sudo apt update
sudo apt install ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
3. Install MAVROS
bash
Copy
Edit
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash install_geographiclib_datasets.sh
🧠 Clone and Build the Code
Download and install the sidewalk-follower repo and LiDAR ROS drivers:

bash
Copy
Edit
# 1. Clone your robot code
cd ~
git clone https://github.com/YOUR_USERNAME/sidewalk-follower.git
cd sidewalk-follower

# 2. Prepare ROS workspace and LiDAR driver
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# 3. Clone LiDAR ROS1 driver
git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros.git

# 4. Add your segmav_bridge package with lidar_to_mavlink_bridge.py
# (copy or create it as shown above)
🛠️ Build the ROS workspace
bash
Copy
Edit
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
⚡ Set LiDAR USB Permissions
Substitute your actual device if not /dev/ttyUSB0:

bash
Copy
Edit
sudo chmod 777 /dev/ttyUSB0
🚀 Launch Order
Start LiDAR driver:

bash
Copy
Edit
roslaunch ldlidar_stl_ros ld19.launch
# Or d500.launch if you create one, but ld19.launch works for D500!
Start LiDAR-to-MAVROS bridge:

bash
Copy
Edit
roslaunch segmav_bridge segmav_bridge.launch
Start main robot script (no ROS required):

bash
Copy
Edit
cd ~/sidewalk-follower
python3 mavsegmav_merged.py --input=csi://0 --headless
# Remove --headless to see a debug overlay window
🧩 What Each File Does
File	Purpose
mavsegmav_merged.py	Camera + GPS + LiDAR main controller
segmav.py	Semantic segmentation overlay/testing
lidar_to_mavlink_bridge.py	Bridges /scan → MAVROS distance sensor
segmav.service	Systemd service for robot auto-start
segmav_bridge.launch	Launches the LiDAR → MAVROS bridge

🗺️ Using Waypoints and Mission Planner
Use Mission Planner to upload waypoints along your desired sidewalk/path

Robot will blend GPS heading with sidewalk detection (it tries to stay centered on sidewalk while generally heading to next waypoint)

You stay on the sidewalk but the robot still makes waypoint progress

🕹️ RC Mode Switching
Set up a 3-position switch on your radio (channel 9 or 10 is common):

RC Position	Action
Low (1000)	Stop
Middle	Record video (optional)
High (2000)	Run vision+GPS navigation

🏁 Testing and Debugging
Use rostopic echo /scan to confirm LiDAR driver is running

If needed, edit serial port in ld19.launch

You can test vision-only code with a saved video:
python3 mavsegmav_merged.py --input=file://video.mp4 --headless

For real-time overlay, remove --headless

🚦 Auto-Start at Boot
Edit your segmav.service:

ini
Copy
Edit
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
Enable the service:

bash
Copy
Edit
sudo cp segmav.service /etc/systemd/system/
sudo systemctl daemon-reexec
sudo systemctl enable segmav.service
sudo systemctl start segmav.service
💡 Notes
ld19.launch works for D500 unless you want to clone and edit a new d500.launch.

Your lidar_to_mavlink_bridge.py must be in the segmav_bridge/scripts/ directory for ROS to find it.

Build with catkin_make whenever you update ROS files.

🧠 Credits
Based on ArduPilot + Jetson AI

LDROBOT ROS Drivers

ArduPilot Rover Tuning Process

Inspired by YouTube - ArduPilot AI Video
