
# 🚗 Dynamic Velocity Control for Sidewalk-Following Robot

This module handles **dynamic velocity control** in the Sidewalk-Following Ackermann Robot system using camera-based sidewalk detection and LiDAR-based obstacle avoidance.

---

## 🎯 How It Works

1. **Semantic Segmentation** detects sidewalk area using models like `fcn-resnet18-cityscapes-1024x512`.
2. The system calculates **sidewalk width ratio** by checking how much of the lower part of the image is labeled as sidewalk.
3. The **velocity is increased** if the sidewalk is wide, and **decreased** if the sidewalk is narrow or lost.
4. **LiDAR scans** are monitored; if an obstacle is detected within a defined range, speed is **drastically reduced**.
5. The calculated velocity is sent to the Pixhawk via **MAVLink** as throttle control.

---

## 🧮 Speed Calculation

```python
sidewalk_width_ratio = sidewalk_pixels / image_width
velocity = VELOCITY_BASE + (VELOCITY_MAX - VELOCITY_BASE) * sidewalk_width_ratio
```

If an obstacle is nearby:

```python
if lidar_min < OBSTACLE_RANGE:
    velocity *= 0.3  # Reduce speed significantly
```

---

## 🔧 Tunable Parameters in `mavsegmav.py`

| Parameter        | Description                                | Example     |
|------------------|--------------------------------------------|-------------|
| `VELOCITY_BASE`  | Minimum speed even if sidewalk is narrow   | `1.0` m/s   |
| `VELOCITY_MAX`   | Max speed when sidewalk is wide            | `2.5` m/s   |
| `WIDTH_SCALE`    | How aggressively to scale speed by width   | `0.4`       |
| `OBSTACLE_RANGE` | Distance (in meters) for obstacle slowdown | `0.8` m     |

---

## 📈 Behavior Summary

| Situation                   | Robot Speed Behavior             |
|-----------------------------|----------------------------------|
| Wide, clear sidewalk        | Faster speed (up to max)         |
| Narrow sidewalk or edge     | Slower speed                     |
| Obstacle in LiDAR range     | Speed reduced significantly      |
| No sidewalk detected        | Moves at base speed (slow)       |

---
## 🧠 Full `mavsegmav.py` with Comments

```python
#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
import jetson.inference
import jetson.utils
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
from pymavlink import mavutil
import serial
import argparse
import time
import math

# Target class to follow (from segmentation)
TARGET_CLASS = "road"

# Velocity logic
WIDTH_SCALE = 0.4
VELOCITY_BASE = 1.0
VELOCITY_MAX = 2.5
OBSTACLE_RANGE = 0.8

# Parse input arguments
parser = argparse.ArgumentParser()
parser.add_argument('--device', default='/dev/ttyTHS1')
parser.add_argument('--baud', type=int, default=57600)
parser.add_argument('--input-flip', default='rotate-180')
parser.add_argument('--vel', type=float, default=2.5)
parser.add_argument('--rc', type=int, default=9)
parser.add_argument('input', nargs='?', default='csi://0')
args = parser.parse_args()

# MAVLink connection to Pixhawk
master = mavutil.mavlink_connection(args.device, baud=args.baud)
master.wait_heartbeat()

# Jetson segmentation model
net = jetson.inference.segNet("fcn-resnet18-cityscapes-1024x512", sys.argv)
input = jetson.utils.videoSource(args.input, argv=sys.argv)
bridge = CvBridge()

# ROS node and LiDAR subscription
rospy.init_node('mavsegmav', anonymous=True)
lidar_min = float('inf')

def lidar_callback(data):
    global lidar_min
    valid_ranges = [r for r in data.ranges if not math.isnan(r) and r > 0.05]
    lidar_min = min(valid_ranges) if valid_ranges else 99.0

rospy.Subscriber('/scan', LaserScan, lidar_callback)

# Compute blended vision+GPS angle
def blend_angles(vision_angle, gps_angle, gps_weight=0.3):
    if gps_angle is None:
        return vision_angle
    return (1 - gps_weight) * vision_angle + gps_weight * gps_angle

# Calculate vision-based steering from mask
def compute_steering_from_mask(mask):
    h, w = mask.shape
    bottom_half = mask[int(h/2):, :]
    moments = cv2.moments(bottom_half)
    if moments['m00'] == 0:
        return 0.0
    cx = int(moments['m10'] / moments['m00'])
    error = (cx - w / 2) / (w / 2)
    return -error

# Compute dynamic speed
def compute_velocity(mask):
    h, w = mask.shape
    sidewalk_width = np.sum(mask[int(0.6*h):, :] > 0) / h
    ratio = min(1.0, WIDTH_SCALE * sidewalk_width / w)
    return VELOCITY_BASE + (VELOCITY_MAX - VELOCITY_BASE) * ratio

# Send RC override to Pixhawk
def send_rc_override(steering, velocity):
    pwm_steer = int(1500 + steering * 300)
    pwm_throttle = int(1500 + velocity * 100)
    rc_channels = [0]*18
    rc_channels[0] = pwm_steer
    rc_channels[2] = pwm_throttle

    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *rc_channels
    )

# Main loop
gps_heading = None
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    img = input.Capture()
    if img is None:
        continue

    seg = net.Segmentation(img)
    mask = jetson.utils.cudaToNumpy(seg, format="gray8")
    mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    gray = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)

    vision_angle = compute_steering_from_mask(binary)
    angle = blend_angles(vision_angle, gps_heading)
    velocity = compute_velocity(binary)

    if lidar_min < OBSTACLE_RANGE:
        velocity *= 0.3

    send_rc_override(angle, velocity)
    rate.sleep()
```

---

## ✅ Summary

This script lets the robot safely and intelligently change its speed:

- Faster when the sidewalk is wide and clear
- Slower when narrow or when obstacles are close
- Blends camera vision with GPS heading

---

