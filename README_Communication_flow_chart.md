                     (A) Hardwired path via Jetson  (UDP fan-out)

 Pixhawk/CrossFlight  UART@57600      Jetson (mavlink-routerd)
 ┌──────────────────┐  /dev/telem1  ┌────────────────────────────┐
 │   ArduRover FC   │<──────────────│ Reads: /dev/telem1:57600   │
 │  TELEM1 (57600)  │               │ Sends UDP:                 │
 └──────────────────┘               │   14550 → Mission Planner  │
          │                         │   14551 → MAVROS/SegMAV    │
          │                         └────────────────────────────┘
          │                                           │
          │                             14551 (listen)│
          │                                           v
          │                               +--------------------+
          │                               |  MAVROS + SegMAV   |
          │                               | (udpin:127.0.0.1:  |
          │                               |    14551)          |
          │                               +--------------------+

                     (B) Wireless path via SiK radio (serial link)
 Pixhawk/CrossFlight  TELEM2@57600
 ┌──────────────────┐      │
 │   ArduRover FC   │      │  (MAVLink over air, 915/433 MHz)
 │  TELEM2 (57600)  │      │
 └──────────────────┘      v
                      ┌──────────────┐   USB/COM@57600   ┌──────────────────┐
                      │  SiK Air     │<──────────────────│  SiK Ground USB  │
                      └──────────────┘                   └──────────────────┘
                                                           │
                                                           v
                                                Mission Planner (COM)



------------------------------------------------------------------------------------

[Pixhawk/CrossFlight UART @ 57600]
            |
            |  /dev/telem1  (FTDI)
            v
+-------------------------------+
|         MAVLink Router        |
|   (mavlink-routerd on Jetson) |
|   Reads: /dev/telem1:57600    |
|   Fans out via UDP            |
+-------------------------------+
        |                 |                  |
        | UDP 14550       | UDP 14551        | (optional extra, e.g., 14555)
        v                 v                  v
+----------------+   +----------------+   +----------------------+
| MissionPlanner |   |    MAVROS      |   |    Other Consumers   |
| (Laptop)       |   | (Jetson)       |   | (e.g., MAVProxy,     |
| udp:JETSON:14550|  | fcu_url:=      |   |   another GCS, etc.) |
|                |   |  udp://0.0.0.0:14551@                     |
+----------------+   +----------------+   +----------------------+

                        (inside Jetson ROS)
                             |
                             | TCPROS topics (not UDP)
                             v
                      +------------------+
                      |  SegMAV script   |
                      | (mavsegmav_... ) |
                      | --device=udpin:  |
                      |   127.0.0.1:14551|
                      +------------------+

                      +------------------+
                      |  LiDAR driver    |
                      |  /scan (ROS)     |
                      +------------------+
                              |
                              v
                  topic_tools relay /scan -> /mavros/obstacle/send
