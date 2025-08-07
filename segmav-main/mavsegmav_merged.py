#!/usr/bin/env python3
"""
Merged script for:
- Semantic segmentation steering
- MAVLink navigation via RC override
- GPS + sidewalk heading blending
- Optional Jetson visualization
- RC channel triggers (record/navigate)
"""

import argparse
import threading
import signal
import sys
import time
import numpy as np
from datetime import datetime
from pymavlink import mavutil
from jetson_inference import segNet
from jetson_utils import videoSource, videoOutput
from segmav import SegThread, VideoThread  # Required local module

exit_event = threading.Event()

def signal_handler(sig, frame):
    exit_event.set()

def send_msg_to_gcs(conn, text):
    msg = f"SEGMAV: {text}"
    conn.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, msg.encode())

def play_tune(conn, tune):
    conn.mav.play_tune_send(conn.target_system, conn.target_component, bytes(tune, "ascii"))

def set_target(conn, speed, yaw):
    TYPE_MASK = 0b101111000111  # Only vx, vy, yaw used
    conn.mav.set_position_target_local_ned_send(
        0, conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED, TYPE_MASK,
        0, 0, 0, speed, 0, 0, 0, 0, 0, yaw, 0
    )

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Merged GPS+vision follower",
        epilog=segNet.Usage() + videoSource.Usage() + videoOutput.Usage(),
        formatter_class=argparse.RawTextHelpFormatter)

    # Inputs
    parser.add_argument("input", nargs='?', default="csi://0")
    parser.add_argument("output", nargs='?', default="rtp://192.168.1.124:5400")

    # MAVLink
    parser.add_argument("--device", default="udpin:127.0.0.1:14550")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--source-system", type=int, default=1)

    # RC control logic
    parser.add_argument("--rc", type=int, default=10)
    parser.add_argument("--pwmlow", type=int, default=1000)
    parser.add_argument("--pwmmid", type=int, default=1500)
    parser.add_argument("--pwmhigh", type=int, default=2000)

    # Segmentation
    parser.add_argument("--network", default="fcn-resnet18-cityscapes-1024x512")
    parser.add_argument("--filter-mode", default="point", choices=["point", "linear"])
    parser.add_argument("--ignore-class", default="void")
    parser.add_argument("--alpha", type=float, default=80.0)
    parser.add_argument("--targetclass", type=int, default=3)

    # Navigation
    parser.add_argument("--vel", type=float, default=0.6)
    parser.add_argument("--gps-weight", type=float, default=0.3)
    parser.add_argument("--headless", action="store_true")

    args = parser.parse_args()
    signal.signal(signal.SIGINT, signal_handler)

    # MAVLink connection
    conn = mavutil.mavlink_connection(args.device,
        source_system=args.source_system,
        baud=args.baud,
        autoreconnect=True,
        force_connected=False,
        source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)

    print("Waiting for heartbeat...")
    while not exit_event.is_set():
        hb = conn.wait_heartbeat(timeout=0.5)
        if hb and hb.type not in [mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_TYPE_GENERIC]:
            break

    print(f"Connected to system {conn.target_system}")
    send_msg_to_gcs(conn, "SegMAV merged script started")
    play_tune(conn, "L12DD")

    # State
    curHeading = -1
    rc_level = None
    fieldname = f"chan{args.rc}_raw"
    segThread = None
    videoThread = None
    last_bearing_check = 0

    # Main loop
    while not exit_event.is_set():
        msg = conn.recv_match(blocking=True, timeout=0.1)

        if msg:
            if msg.get_type() == "RC_CHANNELS":
                pwm_now = getattr(msg, fieldname)
                if rc_level is None or abs(pwm_now - rc_level) > 100:
                    if abs(pwm_now - args.pwmlow) < 50:
                        if segThread:
                            segThread.exit()
                            segThread = None
                        if videoThread:
                            videoThread.exit()
                            videoThread = None
                        send_msg_to_gcs(conn, "STOPPED")
                        play_tune(conn, "L12DD")

                    elif abs(pwm_now - args.pwmmid) < 50:
                        if not videoThread:
                            filename = f"record-{datetime.now().strftime('%Y%m%d-%H%M%S')}.mp4"
                            videoThread = VideoThread(args, sys.argv, filename)
                            videoThread.start()
                            send_msg_to_gcs(conn, f"Recording {filename}")
                            play_tune(conn, "L12DD")
                        else:
                            send_msg_to_gcs(conn, "Recording already active")

                    elif abs(pwm_now - args.pwmhigh) < 50:
                        if not segThread:
                            is_headless = ["--headless"] if args.headless else []
                            segThread = SegThread(args, sys.argv, is_headless)
                            segThread.start()
                            send_msg_to_gcs(conn, "Started NAV")
                            conn.set_mode("GUIDED")
                            play_tune(conn, "L12DD")
                        else:
                            send_msg_to_gcs(conn, "Navigation already active")

                    rc_level = pwm_now

            elif msg.get_type() == "VFR_HUD":
                curHeading = msg.heading

        # Path update
        if segThread and time.time() - last_bearing_check > 0.3:
            bearing = segThread.getBearing()
            if bearing is not None:
                if curHeading > 0:
                    target_bearing = (curHeading + np.rad2deg(bearing)) % 360
                else:
                    target_bearing = np.rad2deg(bearing)
                set_target(conn, args.vel, np.deg2rad(bearing))
                send_msg_to_gcs(conn, f"Rel {np.rad2deg(bearing):.1f}°, Cur {curHeading:.1f}°, Target {target_bearing:.1f}°")
                last_bearing_check = time.time()

    if segThread:
        segThread.exit()
    if videoThread:
        videoThread.exit()

    send_msg_to_gcs(conn, "SegMAV exiting")
    play_tune(conn, "L12DD")
