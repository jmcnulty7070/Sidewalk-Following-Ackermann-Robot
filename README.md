# 🚗 Sidewalk-Following Ackermann Robot (Jetson + Pixhawk + Camera + LiDAR)

This robot drives on sidewalks by following GPS waypoints, using a camera to stay centered on the sidewalk, and avoiding obstacles with LiDAR (Bendy Ruler). It runs on Jetson Nano, Pixhawk Crossflight, and ROS/MAVROS.

---

## 🧰 Hardware Used

- Jetson Nano (4GB, running Ubuntu 18.04/20.04)
- Pixhawk Crossflight (ArduPilot)
- Yahboom GPS (YB-MVV21-V1.0)
- LDROBOT D500 2D LiDAR
- IMX219-160 or Realsense D435 camera
- 3.3V ↔ 5V Logic Level Converter
- Ackermann-steering chassis + brushless motor

---

## 🔌 Wiring Overview

### 📡 GPS (TELEM2)

| GPS Pin | Pixhawk TELEM2 |
| ------- | ---------------|
| PWR     | 5V             |
| GND     | GND            |
| TX      | RX             |
| RX      | TX (optional)  |

### 🧠 Jetson Nano to Pixhawk (TELEM1, MAVLink)

Use a logic level converter between 3.3V (Jetson) and 5V (Pixhawk):

| Jetson Nano | Logic Converter | Pixhawk TELEM1 |
|-------------|------------------|----------------|
| TX          | LV TX → HV RX    | RX             |
| RX          | LV RX ← HV TX    | TX             |
| 3.3V        | LV REF           |                |
| 5V          | HV REF           |                |
| GND         | GND              | GND            |

---

## 🧠 Semantic Segmentation Installation
---
from: Source code is available from https://github.com/stephendade/segmav
Video: https://www.youtube.com/watch?v=aOMq3tztdVY
```bash
sudo apt-get update
sudo apt-get install python3-pip python3-opencv
pip3 install numpy pymavlink

git clone --recursive https://github.com/dusty-nv/jetson-inference
cd jetson-inference && mkdir build && cd build
cmake ../
make -j$(nproc)
sudo make install
./install-pytorch.sh
```

Download: `fcn-resnet18-cityscapes-1024x512`

---

## 🧠 mavsegmav.py (Blended Sidewalk + GPS Steering)

```python
#!/usr/bin/env python3
import cv2, time, math
import jetson.inference
import jetson.utils
from pymavlink import mavutil

net = jetson.inference.segNet("fcn-resnet18-cityscapes-1024x512", sys.argv)
cam = jetson.utils.videoSource("csi://0")
disp = jetson.utils.videoOutput()

master = mavutil.mavlink_connection('/dev/ttyTHS1', baud=57600)
master.wait_heartbeat()

def send_steering_speed(steer, speed):
    master.mav.manual_control_send(
        master.target_system, int(speed*100), 0, 500, int(steer*100), 0)

while True:
    img = cam.Capture()
    seg = net.Process(img)

    width = seg.width
    centerline = int(width / 2)
    pixels = seg.GetClassColors()
    mask = jetson.utils.cudaAllocMapped(width * seg.height * 4)
    net.Mask(seg, mask)

    sidewalk_x = []
    for x in range(width):
        pixel_class = pixels[x + seg.height//2 * width]
        if pixel_class[0] == 128:  # Sidewalk class color
            sidewalk_x.append(x)

    if len(sidewalk_x) > 0:
        mid = sum(sidewalk_x) / len(sidewalk_x)
        offset = (mid - centerline) / centerline
        steer = -offset  # Negative to steer toward center
    else:
        steer = 0  # Fail-safe

    # GPS blending
    gps_steer = 0.2  # Assume slight right turn for example
    alpha = 0.6
    final_steer = alpha * steer + (1 - alpha) * gps_steer

    send_steering_speed(final_steer, 0.6)
    disp.Render(img)
```

---

## 📍 Mission Planner Settings

| Parameter         | Value   |
|-------------------|---------|
| SERIAL1_PROTOCOL  | 2       |
| SERIAL1_BAUD      | 57600   |
| SERIAL2_PROTOCOL  | 5 (GPS) |
| SERIAL2_BAUD      | 57600   |
| COMPASS_ENABLE    | 1       |
| AVOID_ENABLE      | 7       |
| RC9_OPTION        | 300     |

---

## 🛑 LiDAR with MAVROS for Bendy Ruler

Install ROS Noetic:

```bash
sudo apt install ros-noetic-desktop-full
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash install_geographiclib_datasets.sh
```

### ROS Node: lidar_to_mavlink_bridge.py

```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from mavros_msgs.msg import DistanceSensor

def callback(data):
    ds = DistanceSensor()
    ds.header.stamp = rospy.Time.now()
    ds.min_distance = 0.1
    ds.max_distance = 12.0
    ds.current_distance = min(data.ranges)
    ds.type = 1
    ds.id = 0
    ds.orientation = 0
    pub.publish(ds)

rospy.init_node('lidar_bridge')
pub = rospy.Publisher('/mavros/distance_sensor/custom', DistanceSensor, queue_size=10)
rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()
```

---

## 🔁 Autostart on Boot

Create `/etc/systemd/system/segmav.service`

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

```bash
sudo systemctl enable segmav
```

# ✅ Sidewalk-Following Robot — Launch Instructions (Jetson + Pixhawk + Camera + LiDAR)

This guide explains how to launch your robot using ROS, MAVROS, LiDAR, and semantic vision control. It blends GPS waypoint navigation, sidewalk-following using AI, and obstacle avoidance via LiDAR and Bendy Ruler.

---

## ✅ LAUNCH ORDER — Step by Step

🔋 **Make sure Jetson Nano and Pixhawk are powered on**, GPS lock is acquired, and camera & LiDAR are plugged in.

---

### 1. Start ROS Core

This is the master node. Keep this terminal open:

```bash
roscore
```

---

### 2. Start MAVROS (Jetson ↔ Pixhawk Bridge)

> Replace `/dev/ttyTHS1` with your Jetson's serial port (like `/dev/ttyUSB0` if using a USB adapter).

```bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyTHS1:57600
```

---

### 3. Start LiDAR Node

If you're using the **LDROBOT D500**, install their ROS driver. Then:

```bash
roslaunch ldlidar_stl ldlidar.launch
```

This publishes `/scan` — the laser data used for Bendy Ruler.

---

### 4. Start LIDAR-to-MAVLink Bridge

This bridges `/scan` to the MAVROS topic used by ArduPilot for obstacle avoidance.

```bash
rosrun your_package lidar_to_mavlink_bridge.py
```

Make sure `lidar_to_mavlink_bridge.py` exists and is executable in your ROS package.

---

### 5. Start Semantic Segmentation Vision Control

This runs the vision-based sidewalk following using Jetson and a camera.

#### If using Jetson IMX219 CSI Camera:

```bash
python3 mavsegmav.py --input-flip=rotate-180 --rc=9 --vel=2.5 --device=/dev/ttyTHS1 --baud=57600 csi://0
```

#### If using RealSense D435 USB Camera:

```bash
python3 mavsegmav.py --input-flip=none --rc=9 --vel=2.5 --device=/dev/ttyTHS1 --baud=57600 /dev/video0
```

- The robot will **only follow vision** if RC9 switch is set HIGH.
- Otherwise, it defaults to GPS + LiDAR navigation.

---

## 🧠 Optional Autostart with systemd

You can make semantic vision autostart on boot:

```bash
sudo systemctl enable segmav
```

This assumes you created a `segmav.service` unit in `/etc/systemd/system/`.

---

## 📦 Optional: RealSense Camera Node

If other ROS tools (like RViz or bag recording) need access to the RealSense:

```bash
roslaunch realsense2_camera rs_camera.launch
```

---

## 🛰️ Optional: RViz for Debug

To view what the robot is seeing in real-time:

```bash
rviz
```

Add these displays:

- `/scan` (LiDAR scan)
- `/camera/image_raw` (camera feed)
- `/mavros/local_position/pose`, `/mavros/distance_sensor/custom`, etc. for MAVLink info

---

---

## 🧪 Testing Checklist

- [ ] GPS lock in Mission Planner
- [ ] Compass calibration complete
- [ ] LiDAR publishes `/scan`
- [ ] MAVROS publishes `/mavros/distance_sensor/custom`
- [ ] Camera segmenting sidewalk
- [ ] RC9 toggles control mode
- [ ] Robot stays on sidewalk + obeys waypoints
