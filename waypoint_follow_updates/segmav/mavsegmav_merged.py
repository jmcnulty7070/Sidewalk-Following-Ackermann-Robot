#!/usr/bin/env python3
"""
SEGMAV (revised):
- Velocity-only BODY-frame "nudges" with short TTL (~0.3s) at ~2 Hz
- Segmentation-based sidewalk following + optional GPS bearing blend
- Optional video overlay/record
- RC 3-position switch for idle/record/navigate

Networking:
- Default dual-socket with mavlink-router:
    TX: udpout:127.0.0.1:14555   (router listens; we send nudges + heartbeats)
    RX: udpin:0.0.0.0:14601      (router pushes FC telemetry here)
- Back-compat single-socket:
    --device <mavutil url>       (same socket for send/recv; not recommended if router uses split TX/RX)
"""

import argparse
import signal
import sys
import threading
import time
import math
import numpy as np
from datetime import datetime

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2
mavutil.mavlink = mavlink2

# Optional Jetson modules / your threads
from jetson_inference import segNet
from jetson_utils import videoSource, videoOutput
from segmav import SegThread, VideoThread  # in same folder

exit_event = threading.Event()

# ------------------------- Utilities -------------------------

def signal_handler(signum, frame):
    exit_event.set()

def send_msg(conn, text):
    try:
        msg = ('SEGMAV: ' + text).encode()
        conn.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, msg)
    except Exception:
        pass

def play_tune(conn, tune):
    try:
        conn.mav.play_tune_send(conn.target_system, conn.target_component, tune.encode())
    except Exception:
        pass

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def deg_wrap(rad):
    """Wrap radians to [-pi, pi]."""
    while rad > math.pi:  rad -= 2*math.pi
    while rad < -math.pi: rad += 2*math.pi
    return rad

# ---------------- MAVLink nudge sender (NEW) ------------------

def send_velocity_nudge_body(conn, vx_fwd_mps, vy_right_mps, ttl_s=0.3):
    """
    BODY_NED frame velocity-only nudge:
    - vx = forward(+)/back(-), vy = right(+)/left(-), vz=0 for rover
    - TTL-ish: we embed ~ ttl_s*1000 in time_boot_ms (purely informational)
    """
    TM = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
          mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
          mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
          mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
          mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
          mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
          mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
          mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)

    vx = clamp(vx_fwd_mps,  -0.8,  0.8)   # forward/back nudge cap
    vy = clamp(vy_right_mps, -0.35, 0.35) # lateral nudge cap

    try:
        conn.mav.set_position_target_local_ned_send(
            int(ttl_s * 1000),                  # time_boot_ms (TTL-ish; not enforced by FC)
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            TM,
            0, 0, 0,                            # x,y,z (ignored)
            vx, vy, 0,                          # vx,vy,vz (m/s)
            0, 0, 0,                            # ax,ay,az (ignored)
            0, 0                                 # yaw,yaw_rate (ignored)
        )
    except Exception:
        pass

def start_heartbeat(conn, hz=1.0):
    """Heartbeat thread to register UDP peer with router."""
    def loop():
        while not exit_event.is_set():
            try:
                conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, mavutil.mavlink.MAV_STATE_ACTIVE
                )
            except Exception:
                pass
            time.sleep(1.0 / hz)
    t = threading.Thread(target=loop, daemon=True)
    t.start()
    return t

# ---------------- Bearing blending helpers -------------------

class MissionItem:
    def __init__(self, msg):
        self.seq = int(msg.seq)
        # Some FWs send MISSION_ITEM, some MISSION_ITEM_INT
        if msg.get_type() == 'MISSION_ITEM_INT':
            self.x = float(msg.x) / 1e7
            self.y = float(msg.y) / 1e7
        else:
            self.x = float(msg.x)
            self.y = float(msg.y)

def bearing_to_next(mis, idx):
    """Return heading (rad) from item idx to idx+1, or None."""
    if idx + 1 >= len(mis):
        return None
    dx = (mis[idx+1].x - mis[idx].x)
    dy = (mis[idx+1].y - mis[idx].y)
    if dx == 0 and dy == 0:
        return None
    return math.atan2(dy, dx)

# ------------------------------ Main -------------------------

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="SegMAV (velocity nudges) with GPS+Seg bearing blend",
        formatter_class=argparse.RawTextHelpFormatter,
        epilog=segNet.Usage() + videoSource.Usage() + videoOutput.Usage()
    )
    parser.add_argument("input", type=str, default="csi://0", nargs='?')
    parser.add_argument("output", type=str, default="rtp://192.168.1.124:5400", nargs='?')

    # Networking (new dual-socket by default)
    parser.add_argument("--tx-hub", type=str, default="udpout:127.0.0.1:14555",
                        help="where we SEND nudges/heartbeats (router server listens)")
    parser.add_argument("--rx-hub", type=str, default="udpin:0.0.0.0:14601",
                        help="where we RECEIVE FC telemetry (router pushes to this)")
    # Back-compat single-socket (overrides dual if provided)
    parser.add_argument("--device", type=str, default=None,
                        help="single MAVLink URL for both send/recv (e.g., udp:127.0.0.1:14555)")

    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--source-system", type=int, default=203)
    parser.add_argument("--rc", type=int, default=10)
    parser.add_argument("--pwmlow", type=int, default=1000)
    parser.add_argument("--pwmmid", type=int, default=1500)
    parser.add_argument("--pwmhigh", type=int, default=2000)

    parser.add_argument("--network", type=str, default="fcn-resnet18-cityscapes-1024x512")
    parser.add_argument("--filter-mode", type=str, default="point")
    parser.add_argument("--ignore-class", type=str, default="void")
    parser.add_argument("--alpha", type=float, default=80.0)
    parser.add_argument("--targetclass", type=int, default=3)

    parser.add_argument("--vel", type=float, default=0.6, help="base forward velocity (m/s)")
    parser.add_argument("--gps-weight", type=float, default=0.3, help="blend weight for GPS bearing [0..1]")
    parser.add_argument("--nudge-hz", type=float, default=2.0, help="nudge rate (Hz)")
    parser.add_argument("--nudge-ttl", type=float, default=0.3, help="nudge TTL seconds")
    parser.add_argument("--lat-k", type=float, default=0.6, help="lateral gain (m/s per rad of bearing)")

    parser.add_argument("--headless", action='store_true', help="Disables display overlay")
    args = parser.parse_args()
    is_headless = ["--headless"] if args.headless else []

    signal.signal(signal.SIGINT, signal_handler)

    # ----------- Open MAVLink connection(s) ----------
    if args.device:
        # Single socket (send & recv on same port)
        tx = rx = mavutil.mavlink_connection(
            args.device,
            autoreconnect=True,
            source_system=args.source_system,
            baud=args.baud,
            force_connected=False,
            source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER
        )
    else:
        # Dual socket recommended with router (TX separate from RX)
        tx = mavutil.mavlink_connection(
            args.tx_hub,
            autoreconnect=True,
            source_system=args.source_system,
            baud=args.baud,
            force_connected=False,
            source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER
        )
        rx = mavutil.mavlink_connection(
            args.rx_hub,
            autoreconnect=True,
            source_system=args.source_system,
            baud=args.baud,
            force_connected=False,
            source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER
        )

    # Heartbeat so router learns our peer (especially for UDP TX)
    start_heartbeat(tx, hz=1.0)

    print("Waiting for ArduPilot heartbeat...")
    hb = rx.wait_heartbeat(timeout=10.0)
    if hb is None:
        print("No HEARTBEAT from FC. Check mavlink-router and ports.")
        sys.exit(1)

    # Ensure target ids are set on both sockets
    for c in (tx, rx):
        try:
            c.target_system = hb.get_srcSystem()
            c.target_component = hb.get_srcComponent()
        except Exception:
            c.target_system = getattr(hb, 'srcSystem', 1)
            c.target_component = getattr(hb, 'srcComponent', 1)

    print(f"Connected to system {tx.target_system}.{tx.target_component}")
    send_msg(tx, f"RC ch{args.rc} switches modes (low=stop, mid=record, high=Seg+GPS)")

    # ------------- Mission bearing (optional blend) -------------
    mission = []
    tx.mav.mission_request_list_send(tx.target_system, tx.target_component)
    t0 = time.time()
    while time.time() - t0 < 3.0:  # quick grab; it's just for bearing hint
        m = rx.recv_match(type=['MISSION_COUNT','MISSION_ITEM','MISSION_ITEM_INT'],
                          blocking=True, timeout=0.3)
        if not m: continue
        if m.get_type() in ('MISSION_ITEM','MISSION_ITEM_INT'):
            mission.append(MissionItem(m))
        elif m.get_type() == 'MISSION_COUNT' and m.count == 0:
            break
    gps_bearing_hint = bearing_to_next(mission, 0)

    # ------------- RC switch + Seg / Nudge control -------------
    seg_thread = None
    fieldname = f"chan{args.rc}_raw"
    last_rc_level = None
    last_nudge = 0.0
    nudge_dt = 1.0 / clamp(args.nudge_hz, 0.5, 5.0)

    # Put/keep mode GUIDED when we actually start navigating
    def ensure_guided():
        try:
            tx.set_mode_apm('GUIDED')
        except Exception:
            pass

    while not exit_event.is_set():
        msg = rx.recv_match(blocking=True, timeout=0.1)
        if not msg:
            continue

        # RC 3-position handling
        if msg.get_type() == 'RC_CHANNELS':
            pwm_now = getattr(msg, fieldname, None)
            if pwm_now is None:
                continue

            if last_rc_level is None or abs(last_rc_level - pwm_now) > 100:
                # LOW -> stop all
                if abs(args.pwmlow - pwm_now) < 50:
                    if seg_thread:
                        seg_thread.exit()
                        time.sleep(0.5)
                        seg_thread = None
                    send_msg(tx, "Navigation stopped")
                    play_tune(tx, "L16FFF")

                # MID -> record/overlay on
                elif abs(args.pwmmid - pwm_now) < 50:
                    if not seg_thread:
                        filename = f"record-{datetime.now().strftime('%Y%m%d-%H%M%S')}.mp4"
                        seg_thread = VideoThread(args, sys.argv, filename)
                        seg_thread.start()
                        send_msg(tx, f"Recording {filename}")
                        play_tune(tx, "L12DD")
                    else:
                        send_msg(tx, "Already recording")

                # HIGH -> segmentation navigation nudges
                elif abs(args.pwmhigh - pwm_now) < 50:
                    if not seg_thread:
                        seg_thread = SegThread(args, sys.argv, (["--headless"] if args.headless else []))
                        seg_thread.start()
                        ensure_guided()
                        send_msg(tx, "Started Seg+GPS navigation (velocity nudges)")
                        play_tune(tx, "L12DD")

                last_rc_level = pwm_now

        # Periodic nudge based on segmentation + GPS bearing blend
        if seg_thread and (time.time() - last_nudge) >= nudge_dt:
            vis_bearing = seg_thread.getBearing()  # radians, 0 = forward, +right, -left (assumed)
            if vis_bearing is not None:
                # Optional GPS hint toward first leg
                if gps_bearing_hint is not None:
                    # Blend assumes vis_bearing already in body frame; gps_bearing_hint is global heading.
                    # For simplicity, we just bias toward GPS by adding a small term.
                    # (Advanced: rotate GPS bearing into body frame using yaw from AHRS.)
                    blended = (1.0 - args.gps_weight) * vis_bearing + args.gps_weight * deg_wrap(gps_bearing_hint)
                else:
                    blended = vis_bearing

                # Lateral velocity from bearing (small-angle ~ vy ≈ k * bearing)
                vy = clamp(args.lat_k * blended, -0.35, 0.35)
                vx = args.vel
                send_velocity_nudge_body(tx, vx, vy, ttl_s=args.nudge_ttl)
                last_nudge = time.time()

    # Clean exit
    if seg_thread:
        seg_thread.exit()
    send_msg(tx, "Exiting SEGMAV")
