# Sidewalk GPS Follower - File Structure

This document describes the **file structure** of the Sidewalk GPS
Follower system.\
It organizes all code, configuration files, services, and documentation
for easy navigation.

------------------------------------------------------------------------

## 📁 Repository Root Layout

    ~/sidewalk-follower/
    ├── README.md                    # Main project overview
    ├── README_TUNING.md              # Ackermann tuning guide for waypoints and turns
    ├── README_communication_plan.md  # Communication flow (MAVLink, ROS, Jetson, radios)
    ├── README_file_structure.md      # This document
    ├── segmav/                       # Segmentation + MAVLink control logic
    │   ├── guided_mission.py         # GPS follower 
    │   ├── mavsegmav_merged.py       # Advanced merged script (GPS + Segmentation + LiDAR)
    │   ├── segmav.py                 # Segmentation logic (SegThread, VideoThread)
    │   ├── __init__.py
    │   └── utils/                    # Helper functions
    │       └── vision_helpers.py
    ├── launch/                       # ROS and non-ROS launch/config files
    │   ├── sensors.launch.xml        # LiDAR, IMU, and Camera nodes
    │   └── mavros.launch             # MAVROS bridge to Pixhawk/CrossFlight
    ├── config/                       # Parameter configs
    │   ├── segmav_params.yaml        # Segmentation tuning
    │   ├── mavros_params.yaml        # MAVROS params
    │   ├── cartographer.lua          # Cartographer SLAM tuning
    │   └── BO_range.yaml             # Parameter optimizer ranges
    ├── services/                     # Systemd auto-start files
    │   ├── segmav.service            # Runs mavsegmav_merged.py at boot
    │   ├── mavlink-router.service    # Starts MAVLink Router automatically
    │   └── rviz.service              # Optional RViz visualization
    ├── udev/                         # Persistent device naming rules
    │   ├── 99-astra.rules            # Jetson camera (/dev/astra_rgb)
    │   ├── 99-ldlidar.rules          # LiDAR (/dev/ldlidar)
    │   ├── 99-telem1.rules           # FTDI cable for Pixhawk (/dev/telem1 for MAVLink)
    │  
    ├── docs/                         # Documentation & diagrams
    │   ├── system_overview.png       # High-level block diagram
    │   ├── communication_flow.png    # Network/UDP/ROS/MAVLink layout
    │   └── tuning_notes.pdf          # Exported tuning doc
    └── data/                         # Logs and bags
        ├── rosbag/                   # Collected ROS bag files
        ├── mission_planner/          # Waypoint/mission files (.waypoints)
        └── lidar_logs/               # Recorded LiDAR scans

------------------------------------------------------------------------

## 🔑 Notes

-   `segmav/` holds the **core Python control scripts**.
-   `launch/` defines how sensors, mapping, and MAVROS start.
-   `config/` keeps YAML and Lua parameter files.
-   `services/` makes sure everything **auto-starts on boot**.
-   `udev/` rules ensure **persistent device naming** (so `/dev/ldlidar`
    is always the same LiDAR).
-   `docs/` contains diagrams and additional documentation.
-   `data/` is for runtime logs, ROS bags, and recorded missions.

✅ This structure keeps system code, configs, and runtime data
**organized and modular**.
