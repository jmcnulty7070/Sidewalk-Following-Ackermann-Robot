#!/usr/bin/env python3
# ~/segmav/scan_to_obstacle_pymav.py
# ROS LiDAR → MAVLink OBSTACLE_DISTANCE (no MAVROS).
#
# Example:
#   python3 ~/segmav/scan_to_obstacle_pymav.py \
#     --hub udpout:127.0.0.1:14553 --sectors 72 --min 0.20 --max 12.0 --yaw_offset_deg 0

import math, time, argparse
import rospy
from sensor_msgs.msg import LaserScan
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2
mavutil.mavlink = mavlink2

def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v

class ObstacleBridge:
    def __init__(self, m, sectors, rmin_m, rmax_m, yaw_offset_deg, sensor_type=1, frame=None):
        self.m = m
        self.N = int(sectors)
        self.rmin_m = float(rmin_m)
        self.rmax_m = float(rmax_m)
        self.min_cm = int(round(self.rmin_m * 100.0))
        self.max_cm = int(round(self.rmax_m * 100.0))
        self.offset_deg = float(yaw_offset_deg)
        self.inc_deg = 360.0 / self.N
        self.inc_u8 = int(round(self.inc_deg))  # must be 0..255
        self.sensor_type = int(sensor_type)     # 1 = LASER
        # Default to body-forward-right-down frame if available
        self.frame = frame if frame is not None else getattr(mavutil.mavlink, 'MAV_FRAME_BODY_FRD', 12)

        rospy.Subscriber("/scan", LaserScan, self.scan_cb, queue_size=1)
        rospy.loginfo("scan_to_obstacle_pymav: streaming OBSTACLE_DISTANCE to %s (sectors=%d, min=%.2fm, max=%.2fm, yaw_offset=%.1f°)",
                      m.address, self.N, self.rmin_m, self.rmax_m, self.offset_deg)

    def scan_cb(self, msg: LaserScan):
        # Build N bins of uint16 centimeters; 0 = unknown
        dists = [0] * self.N

        if msg.angle_increment == 0.0:
            return
        a = msg.angle_min
        for r in msg.ranges:
            rng = float(r)
            ang = a
            a += msg.angle_increment

            if not math.isfinite(rng):
                continue
            if rng < self.rmin_m or rng > self.rmax_m:
                continue

            # Angle in degrees [0,360), apply yaw offset (deg)
            deg = math.degrees(ang) + self.offset_deg
            deg = (deg % 360.0 + 360.0) % 360.0

            idx = int(deg // self.inc_deg)
            if idx < 0 or idx >= self.N:
                continue

            cm = int(round(rng * 100.0))
            cm = clamp(cm, self.min_cm, self.max_cm)

            # Keep the NEAREST reading per sector
            if dists[idx] == 0 or cm < dists[idx]:
                dists[idx] = cm

        # Send message (types must match dialect)
        try:
            self.m.mav.obstacle_distance_send(
                int(time.time() * 1e6),      # time_usec (uint64)
                self.sensor_type,            # sensor_type (uint8)
                dists,                       # distances (uint16[72]) -> length must equal sectors
                self.inc_u8,                 # increment (uint8, degrees per sector)
                self.min_cm,                 # min_distance (uint16, cm)
                self.max_cm,                 # max_distance (uint16, cm)
                float(self.inc_deg),         # increment_f (float)
                float(self.offset_deg),      # angle_offset (float, degrees)
                int(self.frame)              # frame (uint8)
            )
        except Exception as e:
            rospy.logerr("OBSTACLE_DISTANCE send failed: %s", e)

def main():
    ap = argparse.ArgumentParser(description="ROS /scan → MAVLink OBSTACLE_DISTANCE (pymavlink)")
    ap.add_argument("--hub", default="udpout:127.0.0.1:14553", help="mavlink-router hub")
    ap.add_argument("--sectors", type=int, default=72, help="number of sectors (e.g., 72 => 5° bins)")
    ap.add_argument("--min", type=float, default=0.20, help="min valid range (m)")
    ap.add_argument("--max", type=float, default=12.0, help="max valid range (m)")
    ap.add_argument("--yaw_offset_deg", type=float, default=0.0, help="+deg rotates ring clockwise")
    args = ap.parse_args()

    rospy.init_node("scan_to_obstacle_pymav", anonymous=True)

    m = mavutil.mavlink_connection(args.hub, source_system=202, source_component=191)
    # announce & keepalive so router learns our UDP peer quickly
    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0,0,mavutil.mavlink.MAV_STATE_ACTIVE)
    try:
        hb = m.wait_heartbeat(timeout=5)
    except Exception:
        hb = None
    m.address = args.hub  # for logging

    ObstacleBridge(m, args.sectors, args.min, args.max, args.yaw_offset_deg)
    rospy.spin()

if __name__ == "__main__":
    main()
