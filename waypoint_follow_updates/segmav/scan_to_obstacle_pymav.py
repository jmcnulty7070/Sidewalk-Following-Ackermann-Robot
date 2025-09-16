#!/usr/bin/env python3
# ROS LiDAR → MAVLink OBSTACLE_DISTANCE (pymavlink)
# Adds: front FOV mask and short max-range to calm BendyRuler

import math, time, argparse
import rospy
from sensor_msgs.msg import LaserScan
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2
mavutil.mavlink = mavlink2

def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v

def ang_diff_deg(a, b):
    """smallest signed difference a-b in degrees (-180..+180]"""
    d = (a - b + 180.0) % 360.0 - 180.0
    return d

class ObstacleBridge:
    def __init__(self, m, sectors, rmin_m, rmax_m, yaw_offset_deg, fov_deg, sensor_type=1, frame=None):
        self.m = m
        self.N = int(sectors)
        self.rmin_m = float(rmin_m)
        self.rmax_m = float(rmax_m)
        self.min_cm = int(round(self.rmin_m * 100.0))
        self.max_cm = int(round(self.rmax_m * 100.0))
        self.offset_deg = float(yaw_offset_deg)
        self.fov_deg = float(fov_deg)     # e.g., 120 => only +/-60° around FRONT
        self.inc_deg = 360.0 / self.N
        self.inc_u8 = int(round(self.inc_deg))  # 0..255
        self.sensor_type = int(sensor_type)     # 1 = LASER
        self.frame = frame if frame is not None else getattr(mavutil.mavlink, 'MAV_FRAME_BODY_FRD', 12)

        rospy.Subscriber("/scan", LaserScan, self.scan_cb, queue_size=1)
        rospy.loginfo("OBSTACLE_DISTANCE → %s | sectors=%d min=%.2fm max=%.2fm yaw_offset=%.1f° fov=%.0f°",
                      m.address, self.N, self.rmin_m, self.rmax_m, self.offset_deg, self.fov_deg)

    def scan_cb(self, msg: LaserScan):
        dists = [0] * self.N        # uint16 cm; 0 = unknown/ignored

        if msg.angle_increment == 0.0:
            return
        a = msg.angle_min
        half_fov = self.fov_deg * 0.5

        for r in msg.ranges:
            rng = float(r)
            ang = a
            a += msg.angle_increment

            if not math.isfinite(rng):
                continue
            if rng < self.rmin_m or rng > self.rmax_m:
                continue

            # LiDAR angle → body-forward degrees; 0° = FRONT
            deg = (math.degrees(ang) + self.offset_deg) % 360.0

            # FRONT-arc mask: keep only within ±FOV/2 around 0°
            if self.fov_deg < 360.0:
                if abs(ang_diff_deg(deg, 0.0)) > half_fov:
                    continue

            idx = int(deg // self.inc_deg)
            if idx < 0 or idx >= self.N:
                continue

            cm = int(round(rng * 100.0))
            cm = clamp(cm, self.min_cm, self.max_cm)

            # keep nearest per sector
            if dists[idx] == 0 or cm < dists[idx]:
                dists[idx] = cm

        try:
            self.m.mav.obstacle_distance_send(
                int(time.time() * 1e6),      # time_usec (uint64)
                self.sensor_type,            # sensor_type (uint8)
                dists,                       # uint16[sectors]
                self.inc_u8,                 # increment (uint8, deg/sector)
                self.min_cm,                 # min_distance (uint16, cm)
                self.max_cm,                 # max_distance (uint16, cm)
                float(self.inc_deg),         # increment_f (float)
                float(self.offset_deg),      # angle_offset (float, deg)
                int(self.frame)              # frame (uint8)
            )
        except Exception as e:
            rospy.logerr("OBSTACLE_DISTANCE send failed: %s", e)

def main():
    ap = argparse.ArgumentParser(description="ROS /scan → MAVLink OBSTACLE_DISTANCE (front-arc, short range)")
    ap.add_argument("--hub", default="udpout:127.0.0.1:14553", help="mavlink-router hub")
    ap.add_argument("--sectors", type=int, default=72, help="number of sectors (e.g., 72 => 5° bins)")
    ap.add_argument("--min", type=float, default=0.20, help="min valid range (m)")
    ap.add_argument("--max", type=float, default=0.61, help="**max** valid range (m) (≈2 ft)")
    ap.add_argument("--yaw_offset_deg", type=float, default=0.0, help="+deg rotates ring clockwise")
    ap.add_argument("--fov_deg", type=float, default=120.0, help="front field-of-view in degrees (e.g., 120)")
    args = ap.parse_args()

    rospy.init_node("scan_to_obstacle_pymav", anonymous=True)

    m = mavutil.mavlink_connection(args.hub, source_system=202, source_component=191)
    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0,0,mavutil.mavlink.MAV_STATE_ACTIVE)
    try:
        _ = m.wait_heartbeat(timeout=5)
    except Exception:
        pass
    m.address = args.hub

    ObstacleBridge(m, args.sectors, args.min, args.max, args.yaw_offset_deg, args.fov_deg)
    rospy.spin()

if __name__ == "__main__":
    main()
