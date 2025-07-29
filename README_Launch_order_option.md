
# ✅ LAUNCH ORDER — Step by Step

🔋 Make sure Jetson Nano and Pixhawk are powered on, GPS lock is acquired, and camera/LiDAR are plugged in.

1. **Start ROS Core**
```bash
roscore
```

2. **Start MAVROS (Jetson ↔ Pixhawk Bridge)**
```bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyTHS1:57600
```

3. **Start LiDAR Node (LDROBOT D500)**
```bash
roslaunch ldlidar_stl ldlidar.launch
```

4. **Start LIDAR-to-MAVLink Bridge Node**
```bash
rosrun your_package lidar_to_mavlink_bridge.py
```

5. **Start Semantic Segmentation Vision Control**
```bash
python3 mavsegmav.py --input-flip=rotate-180 --rc=9 --vel=2.5 --device=/dev/ttyTHS1 --baud=57600 csi://0
```

Or for RealSense:
```bash
python3 mavsegmav.py --input-flip=none --rc=9 --vel=2.5 --device=/dev/ttyTHS1 --baud=57600 /dev/video0
```

🧠 Optional Autostart
You can also have this run at boot:

```bash
sudo systemctl enable segmav
```

📦 Optional: RealSense Camera Node
```bash
roslaunch realsense2_camera rs_camera.launch
```

🛰️ Optional: RViz for Debug
```bash
rviz
```

---

## 🔁 Autostart with systemd (Optional Deployment Mode)

To have `mavsegmav.py` launch automatically on boot (no keyboard/screen needed), create this file:

```bash
sudo nano /etc/systemd/system/segmav.service
```

Paste the following:

```ini
[Unit]
Description=Start mavsegmav.py for semantic sidewalk following
After=network.target

[Service]
ExecStart=/usr/bin/python3 /home/robot/mavsegmav.py --input-flip=rotate-180 --rc=9 --vel=2.5 --device=/dev/ttyTHS1 --baud=57600 csi://0
Restart=always
User=robot

[Install]
WantedBy=multi-user.target
```

Then run:
```bash
sudo systemctl daemon-reexec
sudo systemctl enable segmav
sudo systemctl start segmav
```

🧠 Tip: Do not launch `mavsegmav.py` in ROS if you're using systemd. Keep it separate from your `.launch` files.
