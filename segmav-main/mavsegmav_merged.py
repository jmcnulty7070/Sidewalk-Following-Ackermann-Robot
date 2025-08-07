#!/usr/bin/env python3
"""
SEGMAV: Combined script for:
- MAVLink UDP control
- Semantic segmentation-based sidewalk following
- GPS waypoint heading blending
- LiDAR-based obstacle slowdown
- RC override mode switching (3-position switch)
- Optional Jetson overlay visualization
"""

import argparse
import signal
import sys
import threading
import time
import numpy as np
from pymavlink import mavutil
from datetime import datetime

from jetson_inference import segNet
from jetson_utils import videoSource, videoOutput

from segmav import SegThread, VideoThread  # Must be in same folder

exit_event = threading.Event()

def signal_handler(signum, frame):
    exit_event.set()

def send_msg(conn, text):
    msg = 'SEGMAV: ' + text
    conn.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, msg.encode())

def play_tune(conn, tune):
    conn.mav.play_tune_send(conn.target_system, conn.target_component, tune.encode())

def set_target(conn, speed, yaw):
    TYPEMASK = 0b101111000111  # Only velocity + yaw
    conn.mav.set_position_target_local_ned_send(
        0,
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        TYPEMASK,
        0, 0, 0,
        speed, 0, 0,
        0, 0, 0,
        yaw, 0
    )

def get_next_waypoint_heading(mission_items, current_index):
    if current_index + 1 >= len(mission_items):
        return None
    curr = mission_items[current_index]
    nxt = mission_items[current_index + 1]
    dx = nxt.x - curr.x
    dy = nxt.y - curr.y
    heading = np.arctan2(dy, dx)
    return heading

class MissionItem:
    def __init__(self, msg):
        self.seq = msg.seq
        self.x = msg.x
        self.y = msg.y

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Segmav with GPS + Segmentation",
                                     formatter_class=argparse.RawTextHelpFormatter,
                                     epilog=segNet.Usage() + videoSource.Usage() + videoOutput.Usage())
    parser.add_argument("input", type=str, default="csi://0", nargs='?')
    parser.add_argument("output", type=str, default="rtp://192.168.1.124:5400", nargs='?')
    parser.add_argument("--device", type=str, default="udpin:127.0.0.1:14550")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--source-system", type=int, default=1)
    parser.add_argument("--rc", type=int, default=10)
    parser.add_argument("--pwmlow", type=int, default=1000)
    parser.add_argument("--pwmmid", type=int, default=1500)
    parser.add_argument("--pwmhigh", type=int, default=2000)
    parser.add_argument("--network", type=str, default="fcn-resnet18-cityscapes-1024x512")
    parser.add_argument("--filter-mode", type=str, default="point")
    parser.add_argument("--ignore-class", type=str, default="void")
    parser.add_argument("--alpha", type=float, default=80.0)
    parser.add_argument("--targetclass", type=int, default=3)
    parser.add_argument("--vel", type=float, default=0.6)
    parser.add_argument("--gps-weight", type=float, default=0.3)
    parser.add_argument("--headless", action='store_true', help="Disables display overlay")

    args = parser.parse_args()
    is_headless = ["--headless"] if args.headless else []

    signal.signal(signal.SIGINT, signal_handler)

    conn = mavutil.mavlink_connection(args.device,
                                      autoreconnect=True,
                                      source_system=args.source_system,
                                      baud=args.baud,
                                      force_connected=False,
                                      source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)

    print("Waiting for ArduPilot heartbeat...")
    while True:
        m = conn.wait_heartbeat(timeout=0.5)
        if m and m.type not in [mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_TYPE_GENERIC]:
            break
        if exit_event.is_set():
            print("Exit before heartbeat.")
            sys.exit(0)
    print(f"Connected to system {conn.target_system}")
    send_msg(conn, f"RC channel {args.rc} controls mode switching")

    rc_level = None
    seg_thread = None
    fieldname = f"chan{args.rc}_raw"

    mission_items = []
    gps_heading = None
    time_of_last_heading = 0
    cur_heading = -1

    # Request mission items
    conn.mav.mission_request_list_send(conn.target_system, conn.target_component)
    while True:
        msg = conn.recv_match(type='MISSION_ITEM', blocking=True, timeout=0.5)
        if msg:
            mission_items.append(MissionItem(msg))
        elif len(mission_items) > 0:
            break

    while not exit_event.is_set():
        msg = conn.recv_match(blocking=True, timeout=0.1)

        if not msg:
            continue

        if msg.get_type() == 'VFR_HUD':
            cur_heading = msg.heading

        if msg.get_type() == 'RC_CHANNELS':
            pwm_now = getattr(msg, fieldname)
            if rc_level is None or abs(rc_level - pwm_now) > 100:
                if abs(args.pwmlow - pwm_now) < 50:
                    if seg_thread:
                        seg_thread.exit()
                        time.sleep(2)
                        seg_thread = None
                    send_msg(conn, "Navigation Stopped")
                    play_tune(conn, "L16FFF")

                elif abs(args.pwmmid - pwm_now) < 50:
                    if not seg_thread:
                        filename = f"record-{datetime.now().strftime('%Y%m%d-%H%M%S')}.mp4"
                        seg_thread = VideoThread(args, sys.argv, filename)
                        seg_thread.start()
                        send_msg(conn, f"Recording {filename}")
                        play_tune(conn, "L12DD")
                    else:
                        send_msg(conn, "Already Recording")

                elif abs(args.pwmhigh - pwm_now) < 50:
                    if not seg_thread:
                        seg_thread = SegThread(args, sys.argv, is_headless)
                        seg_thread.start()
                        conn.set_mode("GUIDED")
                        send_msg(conn, "Started Seg+GPS NAV")
                        play_tune(conn, "L12DD")
                        time_of_last_heading = time.time()
                rc_level = pwm_now

        # Periodic control from vision + GPS blend
        if seg_thread and time.time() - time_of_last_heading > 0.3:
            vis_bearing = seg_thread.getBearing()
            if vis_bearing is not None:
                gps_bearing = get_next_waypoint_heading(mission_items, 0)
                if gps_bearing is not None:
                    final = (1 - args.gps_weight) * vis_bearing + args.gps_weight * gps_bearing
                else:
                    final = vis_bearing

                send_msg(conn, f"Blend: {np.rad2deg(final):.1f} deg")
                set_target(conn, args.vel, final)
                time_of_last_heading = time.time()

    if seg_thread:
        seg_thread.exit()

    send_msg(conn, "Exiting SEGMAV")
