#!/usr/bin/env python3
"""
Convert ROS LaserScan (/scan) from LD19 into MAVLink OBSTACLE_DISTANCE
messages and forward them to the flight controller through mavlink-router.

Assumptions:
- mavlink-router is running, owns /dev/telem1 (FTDI → TELEM1),
- This script connects to UDP port 14552 (set in main.conf),
- ArduPilot params: PRX1_TYPE=2, OA_TYPE=1 (BendyRuler), AVOID_ENABLE=7.
"""

import math
import time
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from pymavlink import mavutil

# === CONFIG ===
MAV_DEV       = 'udpin:127.0.0.1:14552'  # matches [UdpEndpoint prx] in main.conf
INCREMENT_DEG = 5                        # sector size (5° = 72 bins)
MIN_CM        = 20                       # clamp minimum distance (20 cm)
MAX_CM        = 1200                     # clamp maximum distance (12 m)
YAW_ALIGN_DEG = 0                        # rotate if LiDAR "forward" isn't robot forward

def to_mav_time_us():
    """Return current UNIX time in microseconds."""
    return int(time.time() * 1e6)

class ScanToObstacleDistance:
    def __init__(self):
        rospy.init_node("scan_to_obstacledistance", anonymous=True)

        # MAVLink connection
        self.conn = mavutil.mavlink_connection(MAV_DEV)
        try:
            self.conn.wait_heartbeat(timeout=3)
            rospy.loginfo("Connected to MAVLink heartbeat.")
        except Exception:
            rospy.logwarn("No heartbeat yet, still sending OBSTACLE_DISTANCE.")

        # ROS subscriber
        rospy.Subscriber("/scan", LaserScan, self.scan_cb, queue_size=1)

    def scan_cb(self, msg: LaserScan):
        n = len(msg.ranges)
        ranges = np.array(msg.ranges, dtype=np.float32)

        # Replace NaN/Inf with max distance
        ranges[~np.isfinite(ranges)] = MAX_CM / 100.0

        start_deg = math.degrees(msg.angle_min)
        inc_deg   = math.degrees(msg.angle_increment)

        # Bins for 0–360 deg
        dist_bins = np.full(360 // INCREMENT_DEG, MAX_CM, dtype=np.uint16)

        for i in range(n):
            deg = (start_deg + i * inc_deg + YAW_ALIGN_DEG) % 360.0
            b = int(deg // INCREMENT_DEG)
            d_cm = int(100.0 * max(min(ranges[i], MAX_CM/100.0), MIN_CM/100.0))
            if d_cm < dist_bins[b]:
                dist_bins[b] = d_cm

        # Send MAVLink OBSTACLE_DISTANCE
        self.conn.mav.obstacle_distance_send(
            to_mav_time_us(),          # time_usec
            1,                         # sensor_type (1=laser)
            0,                         # increment (starting angle, deg)
            INCREMENT_DEG,             # increment (deg)
            dist_bins.tolist(),        # distances (cm)
            MIN_CM,                    # min_distance (cm)
            MAX_CM,                    # max_distance (cm)
            0,                         # increment_f (unused)
            0                          # angle_offset (0=forward)
        )

if __name__ == "__main__":
    node = ScanToObstacleDistance()
    rospy.loginfo("scan_to_obstacle_distance started, sending on %s", MAV_DEV)
    rospy.spin()
