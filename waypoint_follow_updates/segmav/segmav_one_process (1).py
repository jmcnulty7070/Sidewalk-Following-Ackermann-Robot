#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SegMAV One-Process Controller (Ackermann-friendly, segmentation overlay)
- Mission pull from FCU (RX hub)
- Course-to-line guidance between sequential waypoints (L1-style)
- Blend: waypoint vector + sidewalk segmentation vector
- BODY_NED forward velocity + yaw-rate (no lateral vy) for Ackermann
- RC5 gating, OA slowdown, HUD, live preview with segmentation overlay
- Hotkeys: [r]=refetch mission/params  [p]=pause TX  [q]=quit
"""

import argparse
import math
import os
import re
import select
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional, Tuple

import numpy as np

# Prefer new NVIDIA module names; fallback to older ones if needed
try:
    import jetson_inference as ji
    import jetson_utils as ju
except Exception:
    try:
        import jetson.inference as ji  # type: ignore
        import jetson.utils as ju      # type: ignore
    except Exception as e:
        print("[warn] jetson-inference not available, running in 'no-vision' fallback:", e, file=sys.stderr)
        ji = None
        ju = None

from pymavlink import mavutil

EARTH_R = 6371000.0  # meters

def clamp(v: float, vmin: float, vmax: float) -> float:
    return vmin if v < vmin else vmax if v > vmax else v

def ang_wrap_pi(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

def equirect_xy(lat0: float, lon0: float, lat: float, lon: float) -> Tuple[float, float]:
    """Local N/E meters using equirectangular around (lat0, lon0)."""
    lat0r = math.radians(lat0)
    lon0r = math.radians(lon0)
    latr = math.radians(lat)
    lonr = math.radians(lon)
    x_e = (lonr - lon0r) * math.cos(0.5 * (latr + lat0r)) * EARTH_R  # East
    y_n = (latr - lat0r) * EARTH_R                                   # North
    return y_n, x_e  # N, E

def equirect_dist_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> Tuple[float, float]:
    lat1r = math.radians(lat1)
    lon1r = math.radians(lon1)
    lat2r = math.radians(lat2)
    lon2r = math.radians(lon2)
    x = (lon2r - lon1r) * math.cos(0.5 * (lat1r + lat2r))
    y = (lat2r - lat1r)
    dist = math.hypot(x, y) * EARTH_R
    bearing = math.atan2(x, y)  # N=0, E=+90deg
    return dist, bearing

def unit_vec_from_heading(hdg_rad: float) -> np.ndarray:
    return np.array([math.cos(hdg_rad), math.sin(hdg_rad)], dtype=float)  # N,E

def normalize(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    return (v / n) if n > 1e-9 else np.array([1.0, 0.0], dtype=float)

def ned_to_body(vx_n: float, vy_e: float, yaw_rad: float) -> Tuple[float, float]:
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
    vx_b = cy * vx_n + sy * vy_e
    vy_b = -sy * vx_n + cy * vy_e
    return vx_b, vy_b

@dataclass
class WP:
    seq: int
    lat: float
    lon: float
    radius: float  # acceptance radius (m)

@dataclass
class VehicleState:
    lat: Optional[float] = None
    lon: Optional[float] = None
    yaw: Optional[float] = None
    rc5: Optional[int] = None
    min_obstacle_m: Optional[float] = None

class MavClient:
    def __init__(self, rx_url: str, tx_url: str, sysid: int = 245, compid: int = 190):
        self.rx = mavutil.mavlink_connection(rx_url, autoreconnect=True)
        self.tx = mavutil.mavlink_connection(
            tx_url, autoreconnect=True, source_system=sysid, source_component=compid
        )
        self.target_system = 1
        self.target_component = 1
        self.state = VehicleState()
        self._lock = threading.Lock()
        self._running = True

    def wait_heartbeat(self, timeout: float = 8.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = self.rx.recv_match(type="HEARTBEAT", blocking=True, timeout=timeout)
            if msg:
                self.target_system = getattr(msg, "sysid", 1)
                self.target_component = getattr(msg, "compid", 1)
                return True
        return False

    def start_reader(self) -> None:
        th = threading.Thread(target=self._reader_loop, daemon=True)
        th.start()

    def _reader_loop(self) -> None:
        while self._running:
            msg = self.rx.recv_match(blocking=False)
            if not msg:
                time.sleep(0.002)
                continue
            mtype = msg.get_type()
            if mtype == "GLOBAL_POSITION_INT":
                with self._lock:
                    self.state.lat = msg.lat * 1e-7
                    self.state.lon = msg.lon * 1e-7
            elif mtype == "ATTITUDE":
                with self._lock:
                    self.state.yaw = float(msg.yaw)  # radians
            elif mtype == "RC_CHANNELS":
                with self._lock:
                    self.state.rc5 = getattr(msg, "chan5_raw", None)
            elif mtype == "OBSTACLE_DISTANCE":
                dists = getattr(msg, "distances", [])
                vals = [d for d in dists if 1 <= d < 60000]  # cm
                if vals:
                    mind = (min(vals) / 100.0)
                    with self._lock:
                        self.state.min_obstacle_m = mind

    def stop(self) -> None:
        self._running = False

    def get_param(self, name: str, default: Optional[float] = None, timeout: float = 2.0) -> Optional[float]:
        self.tx.mav.param_request_read_send(
            self.target_system, self.target_component, name.encode("utf-8"), -1
        )
        t0 = time.time()
        while time.time() - t0 < timeout:
            m = self.rx.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
            if not m:
                break
            pid = m.param_id
            if isinstance(pid, (bytes, bytearray)):
                pid = pid.decode("utf-8", errors="ignore")
            pid = str(pid).rstrip("\x00")
            if pid == name:
                return float(m.param_value)
        return default

    def fetch_mission(self, default_radius: float = 1.5, timeout: float = 8.0) -> List[WP]:
        # Mission protocol on RX (matches your test)
        self.rx.mav.mission_request_list_send(self.target_system, self.target_component)
        count = None
        t0 = time.time()
        while time.time() - t0 < timeout:
            m = self.rx.recv_match(type="MISSION_COUNT", blocking=True, timeout=1.0)
            if m:
                count = int(m.count)
                break
        if not count:
            print("[warn] No mission on FCU or timeout.")
            return []
        wps: List[WP] = []
        for i in range(count):
            self.rx.mav.mission_request_int_send(self.target_system, self.target_component, i)
            mi = None
            t1 = time.time()
            while time.time() - t1 < 2.0:
                r = self.rx.recv_match(blocking=True, timeout=1.0)
                if not r:
                    continue
                if r.get_type() in ("MISSION_ITEM_INT", "MISSION_ITEM") and getattr(r, "seq", -1) == i:
                    mi = r
                    break
            if mi is None:
                self.rx.mav.mission_request_send(self.target_system, self.target_component, i)
                t2 = time.time()
                while time.time() - t2 < 2.0:
                    r = self.rx.recv_match(blocking=True, timeout=1.0)
                    if not r:
                        continue
                    if r.get_type() == "MISSION_ITEM" and getattr(r, "seq", -1) == i:
                        mi = r
                        break
            if mi is None:
                print(f"[error] failed to get mission item {i}, abort.")
                break

            if mi.get_type() == "MISSION_ITEM_INT":
                lat = mi.x / 1e7
                lon = mi.y / 1e7
            else:
                lat = float(mi.x)
                lon = float(mi.y)

            radius = float(getattr(mi, "param2", 0.0)) or default_radius
            wps.append(WP(seq=i, lat=lat, lon=lon, radius=radius))
        print(f"[info] mission loaded: {len(wps)} items")
        return wps

    def send_velocity_body_with_yawrate(self, vx: float, vz: float, yaw_rate: float) -> None:
        """Send BODY_NED forward velocity (vx), zero lateral vy, and yaw_rate.
        Mask ignores pos/accel/absolute yaw, but uses velocity+yaw_rate.
        type_mask = 2311 (binary 0b100100000111)
        """
        type_mask = 2311
        self.tx.mav.set_position_target_local_ned_send(
            int(round(time.time() * 1000)) & 0xFFFFFFFF,
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            type_mask,
            0, 0, 0,
            float(vx), 0.0, float(vz),
            0, 0, 0,
            0.0, float(yaw_rate),
        )

class KeyThread:
    def __init__(self) -> None:
        self._keys: Deque[str] = deque()
        self._running = False
        self._th = None
        try:
            self._have_tty = os.isatty(sys.stdin.fileno())
        except Exception:
            self._have_tty = False
        self._saved = None

    def start(self) -> None:
        if not self._have_tty:
            return
        import termios
        import tty

        self._saved = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self._running = True
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()

    def _loop(self) -> None:
        while self._running:
            r, _, _ = select.select([sys.stdin], [], [], 0.05)
            if r:
                ch = sys.stdin.read(1)
                if ch:
                    self._keys.append(ch)
            time.sleep(0.005)

    def consume(self):
        while self._keys:
            yield self._keys.popleft()

    def stop(self) -> None:
        self._running = False
        if self._have_tty and self._saved is not None:
            import termios
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._saved)

class SegEstimator:
    def __init__(self, src: str, disp: Optional[str], max_angle_deg: float = 30.0, lower_crop: float = 0.45):
        self.enabled = (ji is not None and ju is not None)
        self.src_uri = src
        self.disp_uri = disp
        self.max_ang = math.radians(max_angle_deg)
        self.lower_crop = lower_crop
        self.net = None
        self.cam = None
        self.disp = None
        self.sidewalk_id = None
        self.road_id = None
        if self.enabled:
            try:
                self.net = ji.segNet("fcn-resnet18-cityscapes")
                # Let the URI include codec/size/framerate query (no extra kwargs)
                self.cam = ju.videoSource(self.src_uri)
                self.disp = ju.videoOutput(self.disp_uri) if self.disp_uri else None
                self.sidewalk_id = self._class_id_by_name("sidewalk")
                self.road_id = self._class_id_by_name("road")
                print(f"[vision] labels: sidewalk={self.sidewalk_id}, road={self.road_id}")
            except Exception as e:
                print("[warn] vision init failed:", e, file=sys.stderr)
                self.enabled = False

    def _class_id_by_name(self, name_substr: str) -> Optional[int]:
        if not self.net:
            return None
        for cid in range(self.net.GetNumClasses()):
            if name_substr in self.net.GetClassDesc(cid).lower():
                return cid
        return None

    def estimate_vector(self, yaw_rad: float) -> Tuple[np.ndarray, float, float]:
        """Returns (vec_ned_unit, width_scale[0..1], dx_norm[-1..1])."""
        if not self.enabled:
            return unit_vec_from_heading(yaw_rad), 1.0, 0.0

        img = self.cam.Capture()
        if img is None:
            return unit_vec_from_heading(yaw_rad), 0.8, 0.0

        # Run network + overlay the segmentation (keeps the image sharp)
        self.net.Process(img)
        if self.disp:
            try:
                self.net.Overlay(img)
                self.disp.Render(img)
            except Exception:
                pass

        # Get class mask and compute centroid offset
        try:
            mask = self.net.Mask(img)
        except TypeError:
            mask = self.net.Mask(img)

        width_scale = 0.8
        dx_norm = 0.0
        try:
            np_mask = ju.cudaToNumpy(mask)  # HxW class IDs
            h, w = np_mask.shape[:2]
            y0 = int(h * (1.0 - self.lower_crop))
            roi = np_mask[y0:, :]

            target_id = self.sidewalk_id if self.sidewalk_id is not None else self.road_id
            if target_id is not None:
                ys, xs = np.where(roi == target_id)
                num = max(1, roi.size)
                if xs.size > 0:
                    cx = float(np.mean(xs))
                    dx_norm = (cx - (w / 2.0)) / (w / 2.0)
                    frac = clamp(float(xs.size) / float(num), 0.0, 1.0)
                    width_scale = max(0.4, math.sqrt(frac))
                else:
                    ids = [i for i in [self.sidewalk_id, self.road_id] if i is not None]
                    if ids:
                        m2 = np.isin(roi, ids)
                        ys, xs = np.where(m2)
                        if xs.size > 0:
                            cx = float(np.mean(xs))
                            dx_norm = (cx - (w / 2.0)) / (w / 2.0)
                            frac = clamp(float(xs.size) / float(num), 0.0, 1.0)
                            width_scale = max(0.4, math.sqrt(frac))
        except Exception:
            pass

        dpsi = clamp(dx_norm, -1.0, 1.0) * self.max_ang
        seg_hdg = yaw_rad + dpsi
        seg_vec = unit_vec_from_heading(seg_hdg)
        return seg_vec, width_scale, dx_norm

def segment_guidance(
    lat: float,
    lon: float,
    A: WP,
    B: WP,
    l1_dist: float,
    xtrack_cap_m: float,
    switch_ahead_m: float,
) -> Tuple[np.ndarray, float, float, bool]:
    Ay, Ax = equirect_xy(A.lat, A.lon, A.lat, A.lon)  # (0,0)
    By, Bx = equirect_xy(A.lat, A.lon, B.lat, B.lon)
    Py, Px = equirect_xy(A.lat, A.lon, lat, lon)

    AB = np.array([By - Ay, Bx - Ax], dtype=float)  # N,E
    AP = np.array([Py - Ay, Px - Ax], dtype=float)
    L2 = float(np.dot(AB, AB)) + 1e-9
    t = float(np.dot(AP, AB)) / L2  # projection param
    proj = AB * t
    along_m = float(np.linalg.norm(proj))
    path_vec = normalize(AB)
    cross_z = AB[0] * AP[1] - AB[1] * AP[0]
    xtrack_m = cross_z / max(1e-6, math.sqrt(L2))

    seg_len = math.sqrt(L2)
    advance = (t > 1.0) and (along_m - seg_len > switch_ahead_m)

    x_err = clamp(xtrack_m, -xtrack_cap_m, xtrack_cap_m)
    lookahead = proj + path_vec * l1_dist
    left_norm = normalize(np.array([-path_vec[1], path_vec[0]], dtype=float))
    lookahead -= left_norm * x_err
    to_la = lookahead - AP
    desired = normalize(to_la)
    return desired, xtrack_m, along_m, advance

def resolve_v4l2_uri(uri: str) -> str:
    """Resolve named v4l2 device like /dev/astra_rgb -> /dev/videoN."""
    if uri.startswith("v4l2:///dev/") and not re.search(r"/dev/video\d+$", uri):
        dev = uri[len("v4l2://") :]  # -> "/dev/astra_rgb"
        real = os.path.realpath(dev)
        if os.path.exists(real):
            return "v4l2://" + real
    return uri

def main() -> None:
    ap = argparse.ArgumentParser(
        description="SegMAV One-Process: mission + segmentation + BODY_NED vx + yaw-rate (Ackermann)"
    )
    ap.add_argument("camera", help="camera URI (e.g., v4l2:///dev/astra_rgb?input-codec=mjpeg&width=1280&height=720&framerate=30)")
    ap.add_argument("display", nargs="?", default="display://0", help="display output (preview) or 'none'")
    ap.add_argument("--tx-hub", default="udpout:127.0.0.1:14555", help="MAVLink TX hub")
    ap.add_argument("--rx-hub", default="udpin:0.0.0.0:14601", help="MAVLink RX hub")
    ap.add_argument("--nudge-hz", type=float, default=5.0, help="Command rate (Hz)")
    ap.add_argument("--gps-weight", type=float, default=0.5, help="Blend weight toward sidewalk (0..1; higher=hug sidewalk more)")
    ap.add_argument("--vel", type=float, default=1.0, help="Base m/s if WP_SPEED unavailable")
    ap.add_argument("--max-angle", type=float, default=30.0, help="Max seg correction (deg)")
    ap.add_argument("--rc5-high", type=int, default=1700, help="RC5 threshold for TX enable")
    ap.add_argument("--lower-crop", type=float, default=0.45, help="Lower image fraction for seg centroid")
    ap.add_argument("--no-vision", action="store_true", help="Disable segmentation")
    ap.add_argument("--l1", type=float, default=3.0, help="Lookahead distance along segment (m)")
    ap.add_argument("--xtrack-cap-m", type=float, default=1.0, help="Max lateral correction applied (m)")
    ap.add_argument("--switch-ahead-m", type=float, default=0.5, help="Advance when past B by this margin (m)")
    ap.add_argument("--wp-cap-radius", type=float, default=6.0, help="Max point-capture radius considered (m)")
    ap.add_argument("--yaw-kp", type=float, default=1.0, help="Yaw-rate proportional gain (rad/s per rad)")
    ap.add_argument("--max-yaw-rate", type=float, default=0.6, help="Max yaw rate (rad/s)")
    ap.add_argument("--min-turn-speed", type=float, default=0.3, help="When misaligned, slow to this while turning (m/s)")
    ap.add_argument("--log-tx", action="store_true", help="Print TX lines for debugging")
    args = ap.parse_args()

    display_uri = None if (args.display.lower() in ("none", "null", "off")) else args.display

    mav = MavClient(rx_url=args.rx_hub, tx_url=args.tx_hub)
    print("[info] waiting for HEARTBEAT on RX…")
    if not mav.wait_heartbeat(10.0):
        print("[error] no HEARTBEAT on RX hub; check router endpoints.")
        return
    print("[info] heartbeat ok")
    mav.start_reader()

    def read_speed_and_radius() -> Tuple[float, float]:
        wp_speed = mav.get_param("WP_SPEED", default=None)
        base_speed_v = args.vel if (wp_speed is None or wp_speed <= 0.0) else wp_speed
        if wp_speed and wp_speed > 0:
            print(f"[param] WP_SPEED={wp_speed:.2f} m/s")
        else:
            print(f"[param] WP_SPEED unavailable; using --vel={base_speed_v:.2f} m/s")
        wp_radius_v = mav.get_param("WP_RADIUS", default=1.5)
        print(f"[param] WP_RADIUS={wp_radius_v:.2f} m")
        return base_speed_v, wp_radius_v

    base_speed, wp_radius = read_speed_and_radius()
    mission = mav.fetch_mission(default_radius=wp_radius)
    if not mission:
        print("[warn] empty mission; seg-only")

    camera_uri = resolve_v4l2_uri(args.camera)
    if args.no_vision:
        seg = SegEstimator(src="v4l2:///dev/null", disp=None)
        seg.enabled = False
    else:
        seg = SegEstimator(src=camera_uri, disp=display_uri, max_angle_deg=args.max_angle, lower_crop=args.lower_crop)

    keys = KeyThread()
    try:
        keys.start()
    except Exception as e:
        print(f"[warn] key input disabled: {e}")

    dt_loop = 1.0 / max(1.0, args.nudge_hz)
    idx = 0
    tx_paused = False
    dx_norm_last = 0.0

    print("[info] control loop: RC5 High enables TX | keys [r]=refetch [p]=pause [q]=quit")

    while True:
        t0 = time.time()
        # Hotkeys
        for ch in keys.consume():
            if ch in ("q", "Q"):
                print("[info] quit")
                return
            if ch in ("p", "P"):
                tx_paused = not tx_paused
                print(f"[info] TX {'paused' if tx_paused else 'resumed'}")
            if ch in ("r", "R"):
                print("[info] refetching mission + params…")
                base_speed, wp_radius = read_speed_and_radius()
                mission = mav.fetch_mission(default_radius=wp_radius)
                idx = 0
                print("[info] mission refetch complete")

        # Snapshot vehicle state
        with mav._lock:
            lat = mav.state.lat
            lon = mav.state.lon
            yaw = mav.state.yaw
            rc5 = mav.state.rc5
            min_obs = mav.state.min_obstacle_m

        # --- Path guidance (course-to-line) ---
        path_vec = None
        wp_dist = None
        if mission and lat is not None and lon is not None:
            idx = int(clamp(idx, 0, max(0, len(mission) - 1)))
            A = mission[idx]
            if idx < len(mission) - 1:
                B = mission[idx + 1]
                desired, xtrack_m, along_m, advance = segment_guidance(
                    lat, lon, A, B,
                    l1_dist=args.l1,
                    xtrack_cap_m=args.xtrack_cap_m,
                    switch_ahead_m=args.switch_ahead_m,
                )
                path_vec = desired
                d_to_B, _ = equirect_dist_bearing(lat, lon, B.lat, B.lon)
                wp_dist = d_to_B
                cap_r = min(args.wp_cap_radius, max(0.5, B.radius))
                if advance or (d_to_B <= cap_r):
                    idx = min(idx + 1, len(mission) - 1)
            else:
                d_to_A, bearing = equirect_dist_bearing(lat, lon, A.lat, A.lon)
                path_vec = unit_vec_from_heading(bearing)
                wp_dist = d_to_A

        # --- Segmentation vector ---
        width_scale = 1.0
        dx_norm = 0.0
        if yaw is not None:
            seg_vec, width_scale, dx_norm = seg.estimate_vector(yaw)
            dx_norm_last = dx_norm
        else:
            seg_vec = path_vec if path_vec is not None else np.array([1.0, 0.0])

        # --- Blend path vs sidewalk ---
        beta = clamp(args.gps_weight, 0.0, 1.0)  # weight toward sidewalk
        if path_vec is None:
            blend = normalize(seg_vec)
        else:
            blend = normalize((1.0 - beta) * path_vec + beta * seg_vec)

        # Desired heading from blended vector
        desired_hdg = math.atan2(blend[1], blend[0])  # atan2(E, N)
        yaw_err = 0.0 if yaw is None else ang_wrap_pi(desired_hdg - yaw)

        # --- Speed composition ---
        speed = base_speed * width_scale
        # Slow near waypoint
        if wp_dist is not None and wp_dist < 4.0:
            speed *= clamp(wp_dist / 4.0, 0.3, 1.0)
        # Slow for obstacle proximity (if available)
        if min_obs is not None:
            prox_scale = clamp((min_obs - 0.4) / (1.2 - 0.4), 0.0, 1.0)
            speed *= prox_scale
        # If misaligned, reduce speed for better turning
        if abs(yaw_err) > math.radians(10.0):
            speed = max(speed, args.min_turn_speed)

        # --- TX gating ---
        rc5_high = (rc5 is not None and rc5 >= args.rc5_high)
        tx_enabled = (not tx_paused) and rc5_high

        # Compute yaw-rate command (Ackermann-friendly)
        yaw_rate_cmd = clamp(args.yaw_kp * yaw_err, -args.max_yaw_rate, args.max_yaw_rate)

        if yaw is not None and tx_enabled:
            # Forward in BODY_NED with commanded yaw rate; no lateral vy
            mav.send_velocity_body_with_yawrate(vx=speed, vz=0.0, yaw_rate=yaw_rate_cmd)
            if args.log_tx:
                print(f"[tx] vx_b={speed:+.2f} m/s  yaw_err={yaw_err:+.2f} rad  r={yaw_rate_cmd:+.2f} rad/s  v={base_speed:.2f}  beta={beta:.2f}  dx={dx_norm_last:+.2f}")

        # --- HUD (preview status bar) ---
        if seg.disp:
            s_rc = f"{rc5 or 0}"
            s_wp = f"{idx+1}/{len(mission)}" if mission else "0/0"
            s_dst = f"{(wp_dist or 0.0):.1f}m" if wp_dist is not None else "--"
            s_tx = "TX:ON" if tx_enabled else ("TX:PAUSE" if tx_paused else "TX:OFF")
            s_ws = f"W:{width_scale:.2f}"
            s_sp = f"v:{speed:.2f}m/s"
            s_dx = f"dx:{dx_norm_last:+.2f}"
            seg.disp.SetStatus(
                f"SegMAV Ackermann | {s_tx} | RC5:{s_rc} | WP {s_wp} d={s_dst} | {s_sp} {s_ws} {s_dx} | keys:[r][p][q]"
            )

        # Loop rate
        dt_used = time.time() - t0
        sleep_left = dt_loop - dt_used
        if sleep_left > 0:
            time.sleep(sleep_left)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
