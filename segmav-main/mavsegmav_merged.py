#!/usr/bin/env python3
"""
Main SEGMAV script for Jetson-based sidewalk following with GPS waypoint blending
- Uses MAVLink over UDP to control ArduPilot-based rover
- Jetson Inference for real-time semantic segmentation
- RC override channel to switch between modes: OFF, RECORD, NAV
- Optional overlay mode for visualization
"""

import argparse
import threading
import signal
import sys
import time
from datetime import datetime
from pymavlink import mavutil
import numpy as np

from jetson_inference import segNet
from jetson_utils import videoSource, videoOutput, cudaToNumpy

from segmav import SegThread, VideoThread

exit_event = threading.Event()
waypoints = []
current_wp_index = 0


def signal_handler(signum, frame):
    exit_event.set()


def send_msg_to_gcs(conn, text_to_be_sent):
    """
    Send a message to Ground Control Station
    """
    msg = 'SEGMAV: ' + text_to_be_sent
    conn.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, msg.encode())


def play_tune(conn, tune):
    """
    Play a tune via the onboard buzzer
    """
    conn.mav.play_tune_send(conn.target_system, conn.target_component, bytes(tune, "ascii"))


def set_target(conn, speed, yaw):
    """
    Send velocity + heading command in BODY_NED frame
    """
    TYPEMASK = 0b101111000111  # Velocity + yaw only
    conn.mav.set_position_target_local_ned_send(
        0, conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        TYPEMASK, 0, 0, 0, speed, 0, 0, 0, 0, 0, yaw, 0
    )


def get_next_waypoint_heading(my_lat, my_lon, wp_lat, wp_lon):
    """
    Compute relative bearing to the next waypoint in degrees
    """
    delta_lon = np.radians(wp_lon - my_lon)
    lat1 = np.radians(my_lat)
    lat2 = np.radians(wp_lat)

    x = np.sin(delta_lon) * np.cos(lat2)
    y = np.cos(lat1) * np.sin(lat2) - np.sin(lat1) * np.cos(lat2) * np.cos(delta_lon)
    bearing = np.arctan2(x, y)
    bearing_deg = (np.degrees(bearing) + 360) % 360
    return bearing_deg


def blend_heading(vision_bearing, gps_bearing, weight=0.4):
    """
    Weighted average between GPS heading and vision-based heading
    """
    if gps_bearing is None:
        return vision_bearing
    return (1 - weight) * vision_bearing + weight * gps_bearing


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="MAVLink + Semantic Segmentation Blended Navigation")
    parser.add_argument("input", type=str, default="csi://0", nargs='?',
                        help="Video input stream URI")
    parser.add_argument("--device", type=str, default="udpin:127.0.0.1:14550",
                        help="MAVLink device (UDP or serial)")
    parser.add_argument("--rc", type=int, default=10,
                        help="RC channel for mode control (3-position switch)")
    parser.add_argument("--pwmlow", type=int, default=1000,
                        help="PWM for OFF mode")
    parser.add_argument("--pwmmid", type=int, default=1500,
                        help="PWM for RECORD mode")
    parser.add_argument("--pwmhigh", type=int, default=2000,
                        help="PWM for NAV mode")
    parser.add_argument("--model", type=str, default="fcn-resnet18-cityscapes-1024x512",
                        help="Jetson segmentation model name")
    parser.add_argument("--targetclass", type=int, default=3,
                        help="Segment class ID to follow (e.g. 'road')")
    parser.add_argument("--vel", type=float, default=0.8,
                        help="Base forward velocity")
    parser.add_argument("--overlay", action="store_true",
                        help="Enable RGB + segmentation overlay preview (Jetson GUI)")

    args = parser.parse_args()
    is_headless = not args.overlay

    signal.signal(signal.SIGINT, signal_handler)

    # Start MAVLink
    conn = mavutil.mavlink_connection(args.device, autoreconnect=True,
                                      source_system=1,
                                      source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)
    print("Waiting for MAVLink heartbeat...")
    conn.wait_heartbeat()
    print(f"✅ MAVLink connection established to system {conn.target_system}")

    # Load waypoints from Mission Planner (MISSION_ITEM stream)
    waypoints = []
    while True:
        msg = conn.recv_match(type='MISSION_ITEM', blocking=True, timeout=1.0)
        if msg:
            waypoints.append((msg.x, msg.y))  # latitude, longitude
        elif len(waypoints) > 0:
            print(f"Loaded {len(waypoints)} waypoints.")
            break

    # Init camera + segmentation
    cam = videoSource(args.input, argv=sys.argv)
    net = segNet(args.model, sys.argv)
    curThread = None
    rc_level = None
    fieldname = f"chan{args.rc}_raw"
    gps_heading = None
    time_of_last_bearing_check = 0

    while True:
        if exit_event.is_set():
            if curThread:
                curThread.exit()
            break

        msg = conn.recv_match(blocking=False)
        if msg:
            if msg.get_type() == "RC_CHANNELS":
                pwm_now = getattr(msg, fieldname, None)
                if pwm_now and rc_level is None:
                    rc_level = pwm_now
                if pwm_now and abs(rc_level - pwm_now) > 100:
                    if abs(args.pwmlow - pwm_now) < 50:
                        if curThread:
                            curThread.exit()
                            curThread = None
                        send_msg_to_gcs(conn, "Stopped all SEGMAV")
                        play_tune(conn, "L12DD")
                    elif abs(args.pwmmid - pwm_now) < 50:
                        if not curThread:
                            filename = f"record-{datetime.now().strftime('%Y%m%d-%H%M%S')}.mp4"
                            curThread = VideoThread(args, sys.argv, filename)
                            curThread.start()
                            send_msg_to_gcs(conn, "Recording started")
                            play_tune(conn, "L12DD")
                    elif abs(args.pwmhigh - pwm_now) < 50:
                        if not curThread:
                            curThread = SegThread(args, sys.argv, is_headless)
                            curThread.start()
                            conn.set_mode("GUIDED")
                            send_msg_to_gcs(conn, "Started Navigation")
                            play_tune(conn, "L12DD")
                            time_of_last_bearing_check = time.time()
                    rc_level = pwm_now

            elif msg.get_type() == "GLOBAL_POSITION_INT":
                # ArduPilot sends lat/lon in 1E7 format
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                if current_wp_index < len(waypoints):
                    wp_lat, wp_lon = waypoints[current_wp_index]
                    gps_heading = get_next_waypoint_heading(lat, lon, wp_lat, wp_lon)

        # Periodic vision computation and steering update
        if curThread and time.time() - time_of_last_bearing_check > 0.3:
            time_of_last_bearing_check = time.time()
            bearing = curThread.getBearing()
            if bearing is not None:
                blended = blend_heading(bearing, gps_heading)
                send_msg_to_gcs(conn, f"Bearing: Vision={bearing:.1f}°, GPS={gps_heading:.1f}°, Blended={blended:.1f}°")
                set_target(conn, args.vel, np.deg2rad(blended))

    time.sleep(2)
    send_msg_to_gcs(conn, "Exiting SEGMAV")
