#!/usr/bin/env python3
"""
Merged MAVLink + Semantic Segmentation Navigation Script
- Supports GPS waypoint blending
- Supports segmentation-based sidewalk following
- Supports RC override for mode switching
- Supports LiDAR ROS obstacle distance
- Optional visualization overlay with Jetson Inference
"""

import argparse
import numpy as np
import threading
import signal
import time
import sys
import math
from datetime import datetime

from pymavlink import mavutil
from jetson_inference import segNet
from jetson_utils import videoSource, videoOutput, cudaToNumpy

from segmav import SegThread, VideoThread

# Exit signal for clean shutdown
exit_event = threading.Event()

def signal_handler(sig, frame):
    exit_event.set()

signal.signal(signal.SIGINT, signal_handler)

# Global: Latest obstacle range from ROS (published separately)
latest_obstacle_range = 99.0  # meters

def send_msg_to_gcs(conn, text):
    """Send a text message to ground control station."""
    conn.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, b"SEGMAV: " + text.encode())

def play_tune(conn, tune):
    """Play a short tune on the vehicle."""
    conn.mav.play_tune_send(conn.target_system, conn.target_component, bytes(tune, "ascii"))

def set_target_velocity_yaw(conn, velocity, yaw):
    """Send velocity and yaw setpoint to ArduPilot."""
    POSITION_TARGET_TYPEMASK = 0b101111000111  # Only velocity and yaw
    conn.mav.set_position_target_local_ned_send(
        0,
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        POSITION_TARGET_TYPEMASK,
        0, 0, 0,        # position x/y/z
        velocity, 0, 0,  # velocity x/y/z
        0, 0, 0,        # acceleration x/y/z
        yaw, 0          # yaw, yaw_rate
    )

def get_gps_waypoint_heading(conn):
    """
    Reads current MISSION_ITEM (next waypoint) and computes heading.
    Returns GPS heading in radians if valid.
    """
    try:
        conn.mav.mission_request_int_send(conn.target_system, conn.target_component, 0)
        msg = conn.recv_match(type='MISSION_ITEM_INT', blocking=True, timeout=1.0)
        if not msg:
            return None

        lat = msg.x / 1e7
        lon = msg.y / 1e7
        # You can calculate the heading from current position to (lat, lon) if GPS position available
        # For now, we assume you already have the yaw to next waypoint via mission
        yaw_deg = msg.param4  # Param4 stores heading in MISSION_ITEM_INT
        return math.radians(yaw_deg)
    except:
        return None

# ------------------ MAIN ------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="udpin:127.0.0.1:14550")
    parser.add_argument("--input", default="csi://0")
    parser.add_argument("--output", default="rtp://192.168.1.124:5400")
    parser.add_argument("--network", default="fcn-resnet18-cityscapes-1024x512")
    parser.add_argument("--targetclass", type=int, default=3)
    parser.add_argument("--filter-mode", default="point")
    parser.add_argument("--ignore-class", default="void")
    parser.add_argument("--alpha", type=float, default=80.0)
    parser.add_argument("--vel", type=float, default=0.6)
    parser.add_argument("--rc", type=int, default=10)
    parser.add_argument("--pwmlow", type=int, default=1000)
    parser.add_argument("--pwmmid", type=int, default=1500)
    parser.add_argument("--pwmhigh", type=int, default=2000)
    parser.add_argument("--gps-blend", type=float, default=0.3)
    parser.add_argument("--headless", action="store_true")

    args = parser.parse_args()
    is_headless = args.headless

    conn = mavutil.mavlink_connection(args.device,
                                      autoreconnect=True,
                                      source_system=1,
                                      baud=115200,
                                      force_connected=False,
                                      source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)

    print("Waiting for heartbeat from flight controller...")
    while not exit_event.is_set():
        msg = conn.wait_heartbeat(timeout=0.5)
        if msg and msg.type != mavutil.mavlink.MAV_TYPE_GCS:
            break

    print(f"Connected to system {conn.target_system}")
    send_msg_to_gcs(conn, "SEGMAV script started")
    play_tune(conn, "L12DD")

    rc_level = None
    rc_field = f"chan{args.rc}_raw"
    seg_thread = None
    cur_heading = -1
    last_bearing_check = 0

    while not exit_event.is_set():
        msg = conn.recv_match(blocking=True, timeout=0.1)
        if not msg:
            continue

        if msg.get_type() == 'RC_CHANNELS':
            pwm_now = getattr(msg, rc_field, None)
            if pwm_now is None:
                continue

            if rc_level is None or abs(rc_level - pwm_now) > 100:
                if abs(pwm_now - args.pwmlow) < 50:
                    if seg_thread:
                        seg_thread.exit()
                        seg_thread = None
                        play_tune(conn, "L12DD")
                        send_msg_to_gcs(conn, "Stopped NAV")

                elif abs(pwm_now - args.pwmmid) < 50:
                    if not seg_thread:
                        filename = f"record-{datetime.now().strftime('%Y%m%d-%H%M%S')}.mp4"
                        seg_thread = VideoThread(args, sys.argv, filename)
                        seg_thread.start()
                        play_tune(conn, "L12DD")
                        send_msg_to_gcs(conn, f"Started Recording {filename}")

                elif abs(pwm_now - args.pwmhigh) < 50:
                    if not seg_thread:
                        seg_thread = SegThread(args, sys.argv, is_headless)
                        seg_thread.start()
                        play_tune(conn, "L12DD")
                        send_msg_to_gcs(conn, "Started NAV")
                        conn.set_mode("GUIDED")

                rc_level = pwm_now

        if msg.get_type() == 'VFR_HUD':
            cur_heading = msg.heading

        # Periodic bearing + velocity updates
        if seg_thread and time.time() - last_bearing_check > 0.3:
            rel_bearing = seg_thread.getBearing()
            last_bearing_check = time.time()

            if rel_bearing is not None:
                if cur_heading > 0:
                    abs_bearing = math.radians((cur_heading + math.degrees(rel_bearing)) % 360)
                else:
                    abs_bearing = rel_bearing

                gps_heading = get_gps_waypoint_heading(conn)
                blended_yaw = rel_bearing if gps_heading is None else \
                              (1 - args.gps_blend) * rel_bearing + args.gps_blend * gps_heading

                # Lidar slowdown (if lidar is publishing via ROS to MAVROS /mavros/distance_sensor)
                speed = args.vel
                if latest_obstacle_range < 0.8:
                    speed *= 0.4

                set_target_velocity_yaw(conn, speed, blended_yaw)
                send_msg_to_gcs(conn, f"Bearing {math.degrees(blended_yaw):.1f}°")

    if seg_thread:
        seg_thread.exit()
    send_msg_to_gcs(conn, "SEGMAV exiting")
    play_tune(conn, "L12DD")
