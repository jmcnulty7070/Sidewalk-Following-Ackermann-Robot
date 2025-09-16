#!/usr/bin/env python3
# ~/segmav/guided_waypoint_runner_pymav.py
# GUIDED mission runner for ArduPilot Rover via pymavlink (no MAVROS).
# - Pulls mission from FC through mavlink-router hub
# - Commands Rover using SET_POSITION_TARGET_GLOBAL_INT (position-only)
# - Uses carrot look-ahead for tighter sidewalk tracking
# - Arrival via acceptance, overshoot, timeout
#
# Example:
#   python3 ~/segmav/guided_waypoint_runner_pymav.py \
#     --hub tcp:127.0.0.1:5760 --speed 0.8 --lookahead 2.0 --accept 0.8

import time, math, argparse, threading
from math import radians, sin, cos, asin, sqrt
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2
mavutil.mavlink = mavlink2

EARTH_R = 6371000.0

def haversine_m(lat1, lon1, lat2, lon2):
    dlat = radians(lat2 - lat1); dlon = radians(lon2 - lon1)
    a = sin(dlat/2.0)**2 + cos(radians(lat1))*cos(radians(lat2))*sin(dlon/2.0)**2
    return 2*EARTH_R*asin(sqrt(a))

def ll_to_xy_m(lat_ref, lon_ref, lat, lon):
    x = (lon - lon_ref) * cos(radians(lat_ref)) * EARTH_R
    y = (lat - lat_ref) * EARTH_R
    return x, y

def xy_to_ll_m(lat_ref, lon_ref, x, y):
    lat = lat_ref + (y / EARTH_R) * (180.0 / math.pi)
    lon = lon_ref + (x / (EARTH_R * cos(radians(lat_ref)))) * (180.0 / math.pi)
    return lat, lon

def start_heartbeat(conn, hz=1.0):
    def loop():
        while True:
            try:
                conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, mavutil.mavlink.MAV_STATE_ACTIVE
                )
                time.sleep(1.0/hz)
            except Exception:
                break
    threading.Thread(target=loop, daemon=True).start()

def set_guided(m):
    m.set_mode_apm('GUIDED')
    time.sleep(0.25)

def maybe_arm(m, do_arm):
    if not do_arm: return
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0
    )
    time.sleep(0.5)

def maybe_set_groundspeed(m, speed_mps):
    if not speed_mps or speed_mps <= 0: return
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0,
        1, float(speed_mps), -1, 0, 0, 0, 0
    )
    time.sleep(0.2)

def current_global(m, tmo=0.5):
    msg = m.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=tmo)
    if not msg: return None
    return (msg.lat/1e7, msg.lon/1e7, msg.relative_alt/1000.0)

def goto_position_global_sender(m, lat, lon, alt=0.0):
    TM = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
          mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
          mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
          mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
          mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
          mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
          mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
          mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
    def send_once():
        m.mav.set_position_target_global_int_send(
            0, m.target_system, m.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT, TM,
            int(lat*1e7), int(lon*1e7), float(alt),
            0,0,0, 0,0,0, 0,0
        )
    return send_once

def compute_carrot(lat_now, lon_now, lat_wp, lon_wp, lookahead_m):
    x0, y0 = 0.0, 0.0
    xb, yb = ll_to_xy_m(lat_now, lon_now, lat_wp, lon_wp)
    dx, dy = xb - x0, yb - y0
    dist = math.hypot(dx, dy)
    if dist <= lookahead_m or dist <= 1e-3:
        return lat_wp, lon_wp
    ux, uy = dx/dist, dy/dist
    cx, cy = x0 + ux*lookahead_m, y0 + uy*lookahead_m
    return xy_to_ll_m(lat_now, lon_now, cx, cy)

def fetch_mission(m, timeout=10.0):
    def request_and_wait(ts, tc):
        m.mav.mission_request_list_send(ts, tc)
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = m.recv_match(type='MISSION_COUNT', blocking=True, timeout=1.0)
            if msg: return int(msg.count)
        return None

    order = [(m.target_system, m.target_component),
             (m.target_system, 0), (1,1), (1,0), (0,0)]
    count = target = None
    for ts, tc in order:
        c = request_and_wait(ts, tc)
        if c not in (None, 0):
            count, target = c, (ts, tc)
            break
    if not count:
        raise RuntimeError("Mission COUNT not received or zero (Write a mission from Mission Planner)")

    items = []
    for seq in range(count):
        m.mav.mission_request_int_send(target[0], target[1], seq)
        t1, got = time.time(), None
        while time.time() - t1 < 2.5:
            msg = m.recv_match(type=['MISSION_ITEM_INT','MISSION_ITEM'], blocking=True, timeout=0.5)
            if not msg: continue
            if int(getattr(msg,'seq',-1)) != seq: continue
            got = msg; break
        if got is None: raise RuntimeError(f"Timeout waiting for mission item {seq}")
        cmd = int(got.command)
        if got.get_type() == 'MISSION_ITEM_INT':
            lat, lon, alt = got.x/1e7, got.y/1e7, float(got.z)
        else:
            lat, lon, alt = float(got.x), float(got.y), float(got.z)
        p1, p2, p4 = float(got.param1), float(got.param2), float(got.param4)
        items.append((cmd, lat, lon, alt, p1, p2, p4))
    return items

def main():
    ap = argparse.ArgumentParser(description="GUIDED mission runner with carrot look-ahead (Rover)")
    ap.add_argument("--hub", default="tcp:127.0.0.1:5760",
                    help="mavlink-router hub (e.g., tcp:127.0.0.1:5760 or udpout:127.0.0.1:14553)")
    ap.add_argument("--speed", type=float, default=0.8, help="Ground speed m/s (0=unchanged)")
    ap.add_argument("--lookahead", type=float, default=2.0, help="Carrot look-ahead meters (1.5–3.0 good)")
    ap.add_argument("--accept", type=float, default=0.8, help="Acceptance radius (m); clamped to ≥0.6")
    ap.add_argument("--overshoot", type=float, default=1.0, help="Advance if d grows by this after 10s")
    ap.add_argument("--timeout", type=float, default=90.0, help="Time limit per waypoint (s)")
    ap.add_argument("--arm", action="store_true", help="Arm (if policy allows)")

    args = ap.parse_args()
    m = mavutil.mavlink_connection(args.hub, source_system=201, source_component=190)

    # announce + keepalive
    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0,0, mavutil.mavlink.MAV_STATE_ACTIVE)
    m.mav.ping_send(int(time.time()*1e6), 0, 0, 0)
    start_heartbeat(m, 1.0)

    hb = m.wait_heartbeat(timeout=10)
    if hb is None:
        raise RuntimeError("No HEARTBEAT from FCU (check router & /dev/telem1)")
    try:
        m.target_system = hb.get_srcSystem(); m.target_component = hb.get_srcComponent()
    except Exception:
        m.target_system = getattr(hb, 'srcSystem', 1); m.target_component = getattr(hb, 'srcComponent', 1)
    print(f"Connected: target sys={m.target_system} comp={m.target_component}")

    set_guided(m)
    maybe_arm(m, args.arm)
    maybe_set_groundspeed(m, args.speed)

    items = fetch_mission(m)
    wps = [(lat,lon,alt,p1,p2,p4) for (cmd,lat,lon,alt,p1,p2,p4) in items if cmd == 16]
    if not wps: raise SystemExit("Mission has no NAV_WAYPOINT (16) items")

    for i,(lat_wp,lon_wp,alt_wp,hold_s,accept_m,yaw_deg) in enumerate(wps):
        accept = max(args.accept, accept_m) if accept_m > 0 else max(args.accept, 0.6)
        hold = hold_s if hold_s > 0 else 0.0
        print(f"[WP {i}] lat={lat_wp:.7f} lon={lon_wp:.7f} alt={alt_wp:.1f} accept={accept:.2f} lookahead={args.lookahead:.2f}")

        best_d = 1e9; t0 = time.time(); last_send = 0.0
        resend_dt = 0.5  # 2 Hz keep-alive

        while True:
            pos = current_global(m, 0.5)
            if not pos: continue
            lat_now, lon_now, _ = pos

            # carrot target along NOW->WP
            lat_c, lon_c = compute_carrot(lat_now, lon_now, lat_wp, lon_wp, args.lookahead)
            if (time.time() - last_send) >= resend_dt:
                goto_position_global_sender(m, lat_c, lon_c, alt_wp)()
                last_send = time.time()

            d = haversine_m(lat_now, lon_now, lat_wp, lon_wp)
            best_d = min(best_d, d)

            if d <= accept: print(f"  arrived: d={d:.2f}"); break
            if (time.time() - t0) > args.timeout: print("  timeout; advancing"); break
            if (d - best_d) > args.overshoot and (time.time() - t0) > 10.0:
                print("  overshoot; advancing"); break

        if hold > 0:
            print(f"  hold {hold:.1f}s"); time.sleep(hold)

    print("Mission complete (GUIDED).")

if __name__ == "__main__":
    main()
