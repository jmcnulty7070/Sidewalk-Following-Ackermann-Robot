#!/usr/bin/env python3
# GUIDED multi-WP controller (velocity + yaw in BODY_NED),
# blends waypoint bearing with segmentation yaw. Fixed /dev/video4, preview ON.
# CH5: LOW=OFF, MID=PREVIEW, HIGH=BLEND.

import time, math, argparse, collections
import numpy as np
import cv2
from pymavlink import mavutil
from yolop_backend import YOLOPBackend

def wrap_pi(a): return (a + math.pi) % (2*math.pi) - math.pi
def atan2_avg(a, b, w):
    return math.atan2((1-w)*math.sin(a)+w*math.sin(b),
                      (1-w)*math.cos(a)+w*math.cos(b))

def equirect_NE_from_ll(lat0, lon0, lat, lon):
    R = 6378137.0
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    n = dlat * R
    e = dlon * R * math.cos(math.radians(lat0))
    return n, e

def dist_NE(n1,e1,n2,e2): return math.hypot(n2-n1, e2-e1)
def bearing_NE(n_now,e_now,n_tgt,e_tgt): return math.atan2(e_tgt-e_now, n_tgt-n_now)

def largest_contour(mask):
    cnts,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return max(cnts, key=cv2.contourArea) if cnts else None
def centroid(sub):
    M = cv2.moments(sub, True)
    if M["m00"] <= 0: return None
    return int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])
def steven_angle_from_mask(mask):
    h,w = mask.shape[:2]
    cnt = largest_contour(mask)
    if cnt is None or cv2.contourArea(cnt) < 400: return 0.0
    midy = h//2
    top, bot = mask[:midy,:], mask[midy:,:]
    c1, c2 = centroid(top), centroid(bot)
    if c1 is None or c2 is None: return 0.0
    p1 = (c1[0], c1[1]); p2 = (c2[0], c2[1]+midy)
    dx = p2[0]-p1[0]; dy = p1[1]-p2[1]
    return math.atan2(dx, dy) * 0.8

class AngleSmoother3:
    def __init__(self): self.buf=collections.deque(maxlen=3)
    def add(self,x): self.buf.append(float(x)); return sum(self.buf)/len(self.buf)

def open_mav(dev="/dev/telem1,115200"):
    if dev.startswith("/dev/") and "," in dev:
        port, baud = dev.split(",",1)
        m = mavutil.mavlink_connection(port, baud=int(baud))
    else:
        m = mavutil.mavlink_connection(dev)
    m.wait_heartbeat()
    return m

def set_guided_and_arm(m):
    m.set_mode_apm("GUIDED"); time.sleep(0.3)
    m.mav.command_long_send(m.target_system,m.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                            0, 1,0,0,0,0,0,0)
    t0=time.time()
    while time.time()-t0 < 6:
        hb=m.recv_match(type="HEARTBEAT", blocking=False)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED): return True
        time.sleep(0.1)
    return False

def pull_mission(m, timeout=10.0):
    m.mav.mission_request_list_send(m.target_system, m.target_component)
    count=None; t_end=time.time()+timeout
    while time.time()<t_end:
        msg=m.recv_match(type="MISSION_COUNT", blocking=False)
        if msg: count=msg.count; break
        time.sleep(0.05)
    if count is None: raise RuntimeError("No MISSION_COUNT")
    items=[]
    for i in range(count):
        m.mav.mission_request_int_send(m.target_system, m.target_component, i)
        t0=time.time()
        while time.time()-t0 < 2.0:
            msg=m.recv_match(type=["MISSION_ITEM_INT","MISSION_ITEM"], blocking=False)
            if not msg: time.sleep(0.02); continue
            if msg.seq==i:
                if msg.get_type()=="MISSION_ITEM":
                    msg.x=int(msg.x*1e7); msg.y=int(msg.y*1e7)
                items.append(msg); break
        else: raise RuntimeError(f"Timeout item {i}")
    m.mav.mission_ack_send(m.target_system, m.target_component,
                           mavutil.mavlink.MAV_MISSION_ACCEPTED,0)
    return items

def is_nav_wp(cmd): return int(cmd) in (16,19,21)
def extract_nav_wps(items):
    wps=[]
    for it in items:
        if is_nav_wp(it.command):
            wps.append((it.x/1e7, it.y/1e7, it.z))
    if not wps: raise RuntimeError("No NAV waypoints in mission")
    return wps

def read_rc_ch5(m):
    msg=m.recv_match(type="RC_CHANNELS", blocking=False)
    return msg.chan5_raw if msg else None

def get_att_yaw(m):
    msg=m.recv_match(type="ATTITUDE", blocking=False)
    return msg.yaw if msg else None

def get_local_NE(m, lat0=None, lon0=None):
    msg=m.recv_match(type=["LOCAL_POSITION_NED","GLOBAL_POSITION_INT"], blocking=False)
    if not msg: return (None,(lat0,lon0))
    if msg.get_type()=="LOCAL_POSITION_NED": return ((msg.x,msg.y),(lat0,lon0))
    lat=msg.lat/1e7; lon=msg.lon/1e7
    if lat0 is None: return (None,(lat,lon))
    return (equirect_NE_from_ll(lat0,lon0,lat,lon),(lat0,lon0))

def type_mask_vel_yaw():
    IGN_PX=1<<0; IGN_PY=1<<1; IGN_PZ=1<<2
    IGN_VZ=1<<5; IGN_AX=1<<6; IGN_AY=1<<7; IGN_AZ=1<<8
    IGN_YAWR=1<<11
    return (IGN_PX|IGN_PY|IGN_PZ|IGN_VZ|IGN_AX|IGN_AY|IGN_AZ|IGN_YAWR)

def main():
    ap=argparse.ArgumentParser("GUIDED multi-WP + seg blend (/dev/video4)")
    ap.add_argument("--device", default="/dev/telem1,115200")
    ap.add_argument("--vx", type=float, default=1.2)
    ap.add_argument("--acc-radius", type=float, default=2.0)
    ap.add_argument("--hz", type=float, default=15.0)
    ap.add_argument("--yaw-blend", type=float, default=0.35)  # 0..1 seg weight
    ap.add_argument("--flip", default="none")
    args=ap.parse_args()

    m=open_mav(args.device); print(f"[MAV] connected: sys {m.target_system} comp {m.target_component}")

    items=pull_mission(m); wps_ll=extract_nav_wps(items)
    print(f"[MAV] got {len(wps_ll)} NAV WPs")

    cam=cv2.VideoCapture("/dev/video4")
    if not cam.isOpened(): raise RuntimeError("Cannot open /dev/video4")
    yolop=YOLOPBackend(); smoother=AngleSmoother3()

    set_guided_and_arm(m)

    lat0=lon0=None; wp_NE=None
    PERIOD=1.0/args.hz
    typemask=type_mask_vel_yaw()
    frame=mavutil.mavlink.MAV_FRAME_BODY_NED
    wp_idx=0; last_send=0.0

    try:
        while wp_idx < len(wps_ll):
            ch5 = read_rc_ch5(m)
            state = "OFF" if (ch5 is None or ch5 < 1300) else ("PREVIEW" if ch5 < 1700 else "BLEND")

            (NE,(lat0,lon0)) = get_local_NE(m,lat0,lon0)
            if lat0 is not None and wp_NE is None:
                wp_NE=[ equirect_NE_from_ll(lat0,lon0,lat,lon) for (lat,lon,_alt) in wps_ll ]
                print("[MAV] origin locked; WPs projected")

            yaw_seg_rel=0.0
            ok, img = cam.read()
            if ok:
                if args.flip=="rotate-180": img=cv2.rotate(img, cv2.ROTATE_180)
                elif args.flip=="flip-horizontal": img=cv2.flip(img,1)
                elif args.flip=="flip-vertical": img=cv2.flip(img,0)

                mask = yolop.infer_mask(img)
                yaw_seg_rel = smoother.add( steven_angle_from_mask(mask) )

                # preview overlay
                h,w=img.shape[:2]
                small=cv2.resize(mask,(w//4,h//4))
                img[0:h//4, w-w//4:w] = cv2.cvtColor(small, cv2.COLOR_GRAY2BGR)
                cv2.putText(img, f"CH5={state} seg_yaw={math.degrees(yaw_seg_rel):+.1f} deg",
                            (10,24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                cv2.imshow("GUIDED + Seg Blend (/dev/video4)", img)
                cv2.waitKey(1)

            yaw_att = get_att_yaw(m)
            if NE is None or wp_NE is None or yaw_att is None:
                time.sleep(0.01); continue

            n_now,e_now = NE
            n_tgt,e_tgt = wp_NE[wp_idx]
            d = dist_NE(n_now,e_now,n_tgt,e_tgt)
            yaw_wp = bearing_NE(n_now,e_now,n_tgt,e_tgt)
            yaw_cmd = yaw_wp

            if state == "BLEND":
                yaw_seg_abs = (yaw_att + yaw_seg_rel + math.pi)%(2*math.pi)-math.pi
                yaw_cmd = atan2_avg(yaw_wp, yaw_seg_abs, args.yaw_blend)

            if d <= args.acc_radius:
                print(f"[WP] reached {wp_idx}/{len(wp_NE)-1} at {d:.2f} m")
                wp_idx += 1
                time.sleep(0.2); continue

            now=time.time()
            if now-last_send>=PERIOD:
                m.mav.set_position_target_local_ned_send(
                    int(now*1000), m.target_system, m.target_component,
                    frame, typemask,
                    0,0,0, args.vx,0,0, 0,0,0, yaw_cmd, 0
                ); last_send=now
            time.sleep(0.005)

    finally:
        # stop
        for _ in range(5):
            m.mav.set_position_target_local_ned_send(
                int(time.time()*1000), m.target_system, m.target_component,
                frame, typemask, 0,0,0, 0,0,0, 0,0,0, 0,0
            ); time.sleep(0.05)
        cam.release(); cv2.destroyAllWindows()
        print("[DONE] mission complete or aborted.")

if __name__=="__main__":
    main()
