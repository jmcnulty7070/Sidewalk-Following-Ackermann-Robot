#!/usr/bin/env python3
# guided_wp_segnet_preview.py
# Defaults baked to your setup:
# - MAVLink: /dev/telem1,115200
# - Camera:  v4l2:///dev/video4
# - SegNet:  Cityscapes, sidewalk (class_id=8)
# - CH5:     3-pos switch (LOW=off, MID=preview, HIGH=BLEND)
# - Sends SET_POSITION_TARGET_LOCAL_NED (BODY_NED) velocity X + yaw
import time, argparse, math
import cv2
from pymavlink import mavutil
from backend_segnet import SegNetBackend
from common_overlay import draw_steven_overlay, AngleSmoother3, flip_img, YELLOW

def atan2_blend(a, b, w):
    return math.atan2((1-w)*math.sin(a)+w*math.sin(b),
                      (1-w)*math.cos(a)+w*math.cos(b))

def equirect_NE(lat0, lon0, lat, lon):
    R = 6378137.0
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    n = dlat * R
    e = dlon * R * math.cos(math.radians(lat0))
    return n, e

def dist_NE(n1, e1, n2, e2): return math.hypot(n2-n1, e2-e1)
def bearing_NE(n1, e1, n2, e2): return math.atan2(e2-e1, n2-n1)

def mav_open(dev):
    if dev.startswith("/dev/") and "," in dev:
        port, baud = dev.split(",", 1)
        m = mavutil.mavlink_connection(port, baud=int(baud))
    else:
        m = mavutil.mavlink_connection(dev)
    m.wait_heartbeat(); 
    return m

def set_guided_arm(m):
    m.set_mode_apm("GUIDED"); time.sleep(0.3)
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1,0,0,0,0,0,0)
    t0 = time.time()
    while time.time() - t0 < 6:
        hb = m.recv_match(type="HEARTBEAT", blocking=False)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            return True
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
        while time.time()-t0<2.0:
            msg=m.recv_match(type=["MISSION_ITEM_INT","MISSION_ITEM"], blocking=False)
            if msg and msg.seq==i:
                if msg.get_type()=="MISSION_ITEM":
                    msg.x=int(msg.x*1e7); msg.y=int(msg.y*1e7)
                items.append(msg); break
            time.sleep(0.02)
        else: raise RuntimeError(f"Timeout item {i}")
    m.mav.mission_ack_send(m.target_system, m.target_component,
                           mavutil.mavlink.MAV_MISSION_ACCEPTED, 0)
    return items

def is_nav(cmd): return int(cmd) in (16,19,21)
def extract_nav(items):
    wps=[]
    for it in items:
        if is_nav(it.command): wps.append((it.x/1e7, it.y/1e7, it.z))
    if not wps: raise RuntimeError("No NAV waypoints")
    return wps

def read_ch5(m):
    msg=m.recv_match(type="RC_CHANNELS", blocking=False)
    return getattr(msg,'chan5_raw',None) if msg else None

def get_att_yaw(m):
    msg=m.recv_match(type="ATTITUDE", blocking=False)
    return getattr(msg,'yaw',None) if msg else None

def get_local_NE(m, lat0=None, lon0=None):
    msg=m.recv_match(type=["LOCAL_POSITION_NED","GLOBAL_POSITION_INT"], blocking=False)
    if not msg: return (None,(lat0,lon0))
    if msg.get_type()=="LOCAL_POSITION_NED": return ((msg.x,msg.y),(lat0,lon0))
    lat=msg.lat/1e7; lon=msg.lon/1e7
    if lat0 is None: return (None,(lat,lon))
    return (equirect_NE(lat0,lon0,lat,lon),(lat0,lon0))

def type_mask_vel_yaw():
    IGN_PX=1<<0; IGN_PY=1<<1; IGN_PZ=1<<2
    IGN_VZ=1<<5; IGN_AX=1<<6; IGN_AY=1<<7; IGN_AZ=1<<8
    IGN_YAWR=1<<11
    return (IGN_PX|IGN_PY|IGN_PZ|IGN_VZ|IGN_AX|IGN_AY|IGN_AZ|IGN_YAWR)

def main():
    ap = argparse.ArgumentParser("GUIDED + SegNet sidewalk blend (BODY_NED velocity+yaw) with preview")
    ap.add_argument("--device", default="/dev/telem1,115200")
    ap.add_argument("--camera", default="v4l2:///dev/video4")
    ap.add_argument("--flip", default="none")
    ap.add_argument("--network", default="fcn-resnet18-cityscapes")
    ap.add_argument("--class-id", type=int, default=8)
    ap.add_argument("--vx", type=float, default=1.2)
    ap.add_argument("--acc-radius", type=float, default=2.0)
    ap.add_argument("--yaw-blend", type=float, default=0.35)
    ap.add_argument("--hz", type=float, default=15.0)
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.camera.replace("v4l2://", ""))
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera {args.camera}")

    seg = SegNetBackend(network=args.network, class_id=args.class_id)
    m = mav_open(args.device); print("[MAV] connected")

    items = pull_mission(m); wps = extract_nav(items)
    print(f"[MAV] NAV WPs: {len(wps)}")
    set_guided_arm(m)

    lat0=lon0=None; wp_NE=None
    PERIOD=1.0/args.hz
    typemask=type_mask_vel_yaw()
    frame=mavutil.mavlink.MAV_FRAME_BODY_NED
    idx=0; last=0.0
    smooth=AngleSmoother3()

    try:
        while idx < len(wps):
            ch5 = read_ch5(m)
            mode = "OFF" if (ch5 is None or ch5 < 1300) else ("PREVIEW" if ch5 < 1700 else "BLEND")

            (NE,(lat0,lon0)) = get_local_NE(m, lat0, lon0)
            if lat0 is not None and wp_NE is None:
                wp_NE = [ equirect_NE(lat0,lon0,lat,lon) for (lat,lon,_alt) in wps ]
                print("[MAV] origin locked; WPs projected")

            ok, frame = cap.read()
            if not ok:
                time.sleep(0.01)
                continue
            frame = flip_img(frame, args.flip)
            mask = seg.infer_mask(frame)
            angle_deg_raw, overlay = draw_steven_overlay(frame, mask)
            yaw_seg_rel = math.radians( smooth.add(angle_deg_raw) )

            h, w = overlay.shape[:2]
            small = cv2.resize(mask, (w//4, h//4))
            overlay[0:h//4, w-w//4:w] = cv2.cvtColor(small, cv2.COLOR_GRAY2BGR)
            cv2.putText(overlay, f"CH5={mode}", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, YELLOW, 2)
            cv2.imshow("GUIDED + SegNet (sidewalk) — blend", overlay); cv2.waitKey(1)

            yaw_att = get_att_yaw(m)
            if NE is None or wp_NE is None or yaw_att is None:
                time.sleep(0.01)
                continue

            n_now,e_now = NE
            n_tgt,e_tgt = wp_NE[idx]
            d = dist_NE(n_now,e_now,n_tgt,e_tgt)
            yaw_wp = bearing_NE(n_now,e_now,n_tgt,e_tgt)  # absolute

            yaw_cmd = yaw_wp
            if mode == "BLEND":
                yaw_seg_abs = ((yaw_att + yaw_seg_rel + math.pi) % (2*math.pi)) - math.pi
                yaw_cmd = atan2_blend(yaw_wp, yaw_seg_abs, args.yaw_blend)

            if d <= args.acc_radius:
                print(f"[WP] reached {idx}/{len(wp_NE)-1} @ {d:.2f} m")
                idx += 1
                time.sleep(0.2)
                continue

            now=time.time()
            if now - last >= PERIOD:
                m.mav.set_position_target_local_ned_send(
                    int(now*1000), m.target_system, m.target_component,
                    frame, typemask,
                    0,0,0,  args.vx,0,0,  0,0,0,  yaw_cmd, 0
                )
                last = now

            time.sleep(0.005)

    finally:
        for _ in range(5):
            m.mav.set_position_target_local_ned_send(
                int(time.time()*1000), m.target_system, m.target_component,
                frame, typemask, 0,0,0, 0,0,0, 0,0,0, 0,0
            )
        cap.release(); cv2.destroyAllWindows()
        print("[DONE]")

if __name__ == "__main__":
    main()
