
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

