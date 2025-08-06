#!/usr/bin/env python3
# Combined MAVLink UDP control and segmentation-based navigation

import argparse
from pymavlink import mavutil
from datetime import datetime
import threading
import signal
import sys
import time
from jetson_inference import segNet  # Requires Jetson Inference
from jetson_utils import videoSource, videoOutput
import numpy as np

from segmav import SegThread, VideoThread  # You must provide these classes

exit_event = threading.Event()

def signal_handler(signum, frame):
    exit_event.set()

def send_msg_to_gcs(conn, text_to_be_sent):
    text_msg = 'SEGMAV: ' + text_to_be_sent
    conn.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text_msg.encode())

def play_tune(conn, tune):
    conn.mav.play_tune_send(conn.target_system, conn.target_component, bytes(tune, "ascii"))

def set_target(conn, speed, yaw):
    POSITION_TARGET_TYPEMASK = 0b101111000111  # Only target velocity and yaw
    conn.mav.set_position_target_local_ned_send(
        0,
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        POSITION_TARGET_TYPEMASK,
        0, 0, 0,         # x, y, z positions
        speed, 0, 0,     # vx, vy, vz
        0, 0, 0,         # afx, afy, afz
        yaw, 0           # yaw, yaw_rate
    )

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Segmentation-based navigation for Ardupilot",
                                     formatter_class=argparse.RawTextHelpFormatter,
                                     epilog=segNet.Usage() + videoSource.Usage() + videoOutput.Usage())
    parser.add_argument("input", type=str, default="csi://0", nargs='?',
                        help="URI of the input stream")
    parser.add_argument("output", type=str, default="rtp://192.168.1.124:5400", nargs='?',
                        help="URI of the output stream")
    parser.add_argument("--device", type=str, default="udpin:127.0.0.1:14550",
                        help="MAVLink connection string")
    parser.add_argument("--baud", type=int, default=115200,
                        help="MAVLink baud rate if using serial")
    parser.add_argument("--source-system", type=int, default=1,
                        help="MAVLink Source system ID")
    parser.add_argument("--rc", type=int, default=10,
                        help="3-position RC channel to control script")
    parser.add_argument("--pwmlow", type=int, default=1000,
                        help="RC PWM low value")
    parser.add_argument("--pwmmid", type=int, default=1500,
                        help="RC PWM mid value")
    parser.add_argument("--pwmhigh", type=int, default=2000,
                        help="RC PWM high value")
    parser.add_argument("--network", type=str, default="fcn-resnet18-cityscapes-1024x512",
                        help="Pre-trained model to load")
    parser.add_argument("--filter-mode", type=str, default="point",
                        choices=["point", "linear"])
    parser.add_argument("--ignore-class", type=str, default="void",
                        help="Optional name of class to ignore in output")
    parser.add_argument("--alpha", type=float, default=80.0,
                        help="Alpha blending overlay value (0.0 to 255.0)")
    parser.add_argument("--targetclass", type=int, default=3,
                        help="The item class to track")
    parser.add_argument("--vel", type=float, default=0.6,
                        help="Forward velocity setpoint")

    try:
        args = parser.parse_known_args()[0]
    except:
        parser.print_help()
        sys.exit(0)

    is_headless = ["--headless"] if sys.argv[0].find('console.py') != -1 else [""]

    curThread = None
    signal.signal(signal.SIGINT, signal_handler)

    # Setup MAVLink UDP connection
    conn = mavutil.mavlink_connection(args.device,
                                      autoreconnect=True,
                                      source_system=args.source_system,
                                      baud=args.baud,
                                      force_connected=False,
                                      source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)

    print("Waiting for Heartbeat from ArduPilot...")
    while True:
        m = conn.wait_heartbeat(timeout=0.5)
        if m and m.type not in [mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_TYPE_GENERIC]:
            break
        if exit_event.is_set():
            print("Exiting before heartbeat received")
            sys.exit(0)

    print(f"Got heartbeat from ArduPilot (system {conn.target_system})")

    rc_level = None
    fieldname = f"chan{args.rc}_raw"
    curHeading = -1
    time_of_last_bearing_check = 0

    send_msg_to_gcs(conn, f"Monitoring RC channel {args.rc}")

    while True:
        msg = conn.recv_match(blocking=True, timeout=0.1)
        if msg:
            if msg.get_type() == 'VFR_HUD':
                curHeading = msg.heading
            if msg.get_type() == 'RC_CHANNELS':
                if not rc_level:
                    rc_level = getattr(msg, fieldname)
                pwm_now = getattr(msg, fieldname)

                if abs(rc_level - pwm_now) > 100:
                    if abs(args.pwmlow - pwm_now) < 50:
                        if curThread:
                            curThread.exit()
                            time.sleep(2)
                            curThread = None
                        send_msg_to_gcs(conn, "Stopped Record/NAV")
                        time_of_last_bearing_check = 0
                        play_tune(conn, "L12DD")

                    elif abs(args.pwmmid - pwm_now) < 50:
                        if not curThread:
                            filename = f"record-{datetime.now().strftime('%Y%m%d-%H%M%S')}.mp4"
                            print(f"Recording to: {filename}")
                            try:
                                curThread = VideoThread(args, sys.argv, filename)
                                curThread.start()
                                send_msg_to_gcs(conn, f"Started recording {filename}")
                                play_tune(conn, "L12DD")
                            except Exception as ex:
                                print(f"Recording failed: {ex}")
                                play_tune(conn, "L16FFF")
                        else:
                            send_msg_to_gcs(conn, "Recording already active")
                            play_tune(conn, "L16FFF")

                    elif abs(args.pwmhigh - pwm_now) < 50:
                        if not curThread:
                            curThread = SegThread(args, sys.argv, is_headless)
                            curThread.start()
                            send_msg_to_gcs(conn, "Started Navigation")
                            conn.set_mode("GUIDED")
                            time_of_last_bearing_check = time.time()
                            play_tune(conn, "L12DD")
                        else:
                            send_msg_to_gcs(conn, "Navigation already active")
                            play_tune(conn, "L16FFF")

                    rc_level = pwm_now

        if exit_event.is_set():
            if curThread:
                curThread.exit()
            break

        if time_of_last_bearing_check > 0 and time.time() - time_of_last_bearing_check > 0.3 and curThread:
            bearing = curThread.getBearing()
            time_of_last_bearing_check = time.time()
            if bearing is not None:
                if curHeading > 0:
                    target_bearing = (curHeading + bearing) % 360
                else:
                    target_bearing = -1
                send_msg_to_gcs(conn, f"Rel {bearing:.0f}°, Cur {curHeading:.0f}°, Target {target_bearing:.0f}°")
                set_target(conn, args.vel, np.deg2rad(bearing))

    time.sleep(2)
    send_msg_to_gcs(conn, "Exiting SEGMAV")
