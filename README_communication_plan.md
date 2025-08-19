# Communication Plan for Ackermann Car (Sidewalk Follower with MAVLink + ROS)

This document explains how commands and information flow between
**Mission Planner**, **Crossflight flight controller**, **Jetson
companion computer**, **LiDAR**, and the **SiK telemetry radio**. It
focuses on how UDP, MAVLink, and ROS are used together.

------------------------------------------------------------------------

## 1. Key Communication Channels

### UDP (User Datagram Protocol)

-   **What it is:** A fast, connectionless way of sending messages
    across a network.\
-   **Why used:** MAVLink prefers UDP because it is lightweight and
    real-time (small delays, less overhead).\
-   **How it works in our setup:**
    -   MAVLink messages (telemetry, commands) are wrapped in UDP
        packets.\
    -   Multiple systems can listen on the same UDP port (e.g., Jetson
        and Mission Planner both listen on `14550`).

------------------------------------------------------------------------

## 2. Communication Devices & Interfaces

### a) Mission Planner on PC

-   Connects via **SiK Radio** (USB ground unit).\
-   Sends waypoints, mode changes, and parameters to the flight
    controller (Crossflight).\
-   Listens to telemetry (battery, GPS, mode, velocity) from the flight
    controller.\
-   **IP/Port:** Typically `COMx` on Windows → forwarded into MAVLink.

### b) SiK Telemetry Radio (Air & Ground)

-   Provides wireless MAVLink link between **Mission Planner (PC)** and
    **Crossflight FC**.\
-   Ground SiK (USB dongle) ↔ Air SiK (TELEM1 port on FC).\
-   Baud rate: commonly 57600 or 115200.\
-   Used for **manual mission upload** and **telemetry viewing**.

### c) Crossflight Flight Controller (FC)

-   Runs ArduPilot Rover firmware.\
-   Interfaces:
    -   **TELEM1:** SiK radio (to PC Mission Planner).\
    -   **TELEM2:** Companion computer (Jetson) via MAVLink Router.\
-   Tasks:
    -   Executes waypoint missions from Mission Planner.\
    -   Accepts velocity commands from Jetson (SegMAV).\
    -   Processes obstacle avoidance using LiDAR data.

### d) Jetson Companion Computer

-   Runs `mavsegmav_merged.py`.\
-   Subscribes to MAVLink heartbeat and mission data from FC.\
-   Publishes velocity commands (`SET_POSITION_TARGET_LOCAL_NED` or
    `SET_ATTITUDE_TARGET`).\
-   Runs ROS drivers for LiDAR (e.g., `/scan` topic).\
-   Uses MAVROS to forward obstacle messages into MAVLink.\
-   **UDP device string:**
    -   `udpin:0.0.0.0:14550` → listen for FC heartbeat.\
    -   `udpout:127.0.0.1:14555` → forward to local processes.

### e) LiDAR (LD19)

-   Connected to Jetson via USB/FTDI.\
-   ROS driver publishes `/scan`.\
-   SegMAV script reads `/scan` → computes free path → generates
    velocity command → sends via MAVLink to FC.

------------------------------------------------------------------------

## 3. Information Flow

### Mission Planner → SiK → Crossflight → Jetson

1.  Operator uploads waypoints from Mission Planner (PC).\
2.  Waypoints transmitted via SiK radio → TELEM1 (FC).\
3.  FC executes waypoint navigation.\
4.  FC forwards telemetry and mission state to Jetson (via MAVLink
    Router over TELEM2).

### Jetson (SegMAV + ROS) → Crossflight

1.  SegNet finds sidewalk center from camera.\
2.  LiDAR (`/scan`) checks obstacles.\
3.  Jetson blends vision + LiDAR → velocity command.\
4.  Command sent over MAVLink UDP → TELEM2 → Crossflight.

### Crossflight → Mission Planner (via SiK)

1.  Crossflight sends live telemetry (GPS, battery, mode).\
2.  Data sent via TELEM1 → air SiK → ground SiK → Mission Planner.\
3.  Operator can monitor real-time vehicle status.

------------------------------------------------------------------------

## 4. Example Network Map

    [ Mission Planner PC ] 
            | USB
       [ SiK Ground Radio ]
            | Wireless 915MHz
       [ SiK Air Radio ] -- TELEM1 --> [ Crossflight FC ] <-- TELEM2 --> [ Jetson Nano ]
                                                                             |
                                                                        USB/FTDI
                                                                         [ LiDAR ]

------------------------------------------------------------------------

## 5. Notes on UDP Sharing

-   Multiple apps (Mission Planner, MAVROS, SegMAV script) can all
    listen to the same UDP port (`14550`).\
-   MAVLink Router running on Jetson duplicates streams:
    -   One for local apps (SegMAV, MAVROS).\
    -   One for forwarding back to GCS (Mission Planner).

------------------------------------------------------------------------

✅ This ensures **Mission Planner** can upload waypoints and monitor,
while the **Jetson** can apply vision/LiDAR logic and override velocity
in real time.
