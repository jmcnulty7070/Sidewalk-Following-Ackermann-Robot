#!/usr/bin/env python3
import math, time, numpy as np, rospy
from sensor_msgs.msg import LaserScan
from pymavlink import mavutil

MAV_DEV       = 'udpin:127.0.0.1:14552'
INCREMENT_DEG = 5
MIN_CM        = 20
MAX_CM        = 1200
YAW_ALIGN_DEG = 0

def to_mav_time_us():
    return int(time.time() * 1e6)

class ScanToObstacleDistance:
    def __init__(self):
        rospy.init_node("scan_to_obstacledistance", anonymous=True)
        self.conn = mavutil.mavlink_connection(MAV_DEV)
        try:
            self.conn.wait_heartbeat(timeout=3)
            rospy.loginfo("Connected to MAVLink heartbeat.")
        except Exception:
            rospy.logwarn("No heartbeat yet, still sending OBSTACLE_DISTANCE.")
        rospy.Subscriber("/scan", LaserScan, self.scan_cb, queue_size=1)

    def scan_cb(self, msg):
        n = len(msg.ranges)
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges[~np.isfinite(ranges)] = MAX_CM / 100.0
        start_deg = math.degrees(msg.angle_min)
        inc_deg   = math.degrees(msg.angle_increment)
        dist_bins = np.full(360 // INCREMENT_DEG, MAX_CM, dtype=np.uint16)
        for i in range(n):
            deg = (start_deg + i * inc_deg + YAW_ALIGN_DEG) % 360.0
            b = int(deg // INCREMENT_DEG)
            d_cm = int(100.0 * max(min(ranges[i], MAX_CM/100.0), MIN_CM/100.0))
            if d_cm < dist_bins[b]:
                dist_bins[b] = d_cm
        self.conn.mav.obstacle_distance_send(
            to_mav_time_us(),
            1, 0, INCREMENT_DEG,
            dist_bins.tolist(),
            MIN_CM, MAX_CM, 0, 0
        )

if __name__ == "__main__":
    node = ScanToObstacleDistance()
    rospy.loginfo("scan_to_obstacle_distance started, sending on %s", MAV_DEV)
    rospy.spin()
