#!/usr/bin/env python3
# AUTO mission + small steering nudges via RC override on CH1.
# Fixed camera v4l2:///dev/video4, preview window ON.
# CH5: LOW=OFF, MID=PREVIEW, HIGH=NUDGE.

import time, math, argparse, collections
import cv2
from pymavlink import mavutil
from yolop_backend import YOLOPBackend

def flip_img(img, mode):
    if mode=="rotate-180": return cv2.rotate(img, cv2.ROTATE_180)
    if mode=="flip-horizontal": return cv2.flip(img,1)
    if mode=="flip-vertical": return cv2.flip(img,0)
    return img

def largest_contour(mask):
    cnts,_=cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return max(cnts, key=cv2.contourArea) if cnts else None

def centroid(sub):
    M=cv2.moments(sub,True)
    if M["m00"]<=0: return None
    return int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])

def seg_angle(mask):
    h,w=mask.shape[:2]
    cnt=largest_contour(mask)
    if not cnt or cv2.contourArea(cnt)<400: return 0.0
    midy=h//2
    c1=centroid(mask[:midy,:]); c2=centroid(mask[midy:,:])
    if not c1 or not c2: return 0.0
    p1=(c1[0],c1[1]); p2=(c2[0],c2[1]+midy)
    dx=p2[0]-p1[0]; dy=p1[1]-p2[1]
    return math.atan2(dx,dy)*0.8  # rad, right +

class Smoother3:
    def __init__(self): self.b=collections.deque(maxlen=3)
    def add(self,x): self.b.append(float(x)); return sum(self.b)/len(self.b)

def rc_ch5(m):
    msg=m.recv_match(type="RC_CHANNELS", blocking=False)
    return msg.chan5_raw if msg else None

def rc_override_steer(m, steer_pwm):
    m.mav.rc_channels_override_send(
        m.target_system, m.target_component,
        int(steer_pwm), 0,0,0,0,0,0,0
    )

def main():
    ap=argparse.ArgumentParser("AUTO + nudges (fixed /dev/video4)")
    ap.add_argument("--device", default="/dev/telem1,115200")
    ap.add_argument("--flip", default="none")
    ap.add_argument("--hz", type=float, default=15.0)
    ap.add_argument("--neutral", type=int, default=1500)
    ap.add_argument("--kp_us_per_deg", type=float, default=4.0)
    ap.add_argument("--max_delta_us", type=int, default=120)
    ap.add_argument("--deadband_deg", type=float, default=2.0)
    args=ap.parse_args()

    cap=cv2.VideoCapture("/dev/video4")
    if not cap.isOpened(): raise RuntimeError("Cannot open /dev/video4")

    yolop=YOLOPBackend()
    # parse device string like /dev/telem1,115200
    dev=args.device
    if dev.startswith("/dev/") and "," in dev:
        port, baud = dev.split(",",1)
        m=mavutil.mavlink_connection(port, baud=int(baud))
    else:
        m=mavutil.mavlink_connection(dev)
    m.wait_heartbeat()
    print(f"[MAV] connected: sys {m.target_system} comp {m.target_component}")

    sm=Smoother3()
    period=1.0/args.hz
    last=0.0

    try:
        while True:
            ok, img = cap.read()
            if not ok: time.sleep(0.01); continue
            img=flip_img(img, args.flip)

            ch5 = rc_ch5(m)
            state="OFF" if (ch5 is None or ch5<1300) else ("PREVIEW" if ch5<1700 else "NUDGE")

            mask = yolop.infer_mask(img)
            yaw_rel = sm.add( seg_angle(mask) )
            # preview
            h,w=img.shape[:2]
            small=cv2.resize(mask,(w//4,h//4))
            img[0:h//4, w-w//4:w] = cv2.cvtColor(small, cv2.COLOR_GRAY2BGR)
            cv2.putText(img, f"CH5={state} yaw={math.degrees(yaw_rel):+.1f} deg",
                        (10,24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            cv2.imshow("AUTO + Nudges (/dev/video4)", img); cv2.waitKey(1)

            now=time.time()
            if state=="NUDGE" and (now-last)>=period:
                err_deg = math.degrees(yaw_rel)
                if abs(err_deg) < args.deadband_deg:
                    steer=args.neutral
                else:
                    delta = args.kp_us_per_deg * err_deg
                    delta = max(-args.max_delta_us, min(args.max_delta_us, delta))
                    steer = int(args.neutral + delta)
                rc_override_steer(m, steer)
                last=now
            elif state!="NUDGE":
                # release
                m.mav.rc_channels_override_send(m.target_system, m.target_component,
                                                0,0,0,0,0,0,0,0)
            time.sleep(0.005)

    finally:
        m.mav.rc_channels_override_send(m.target_system, m.target_component, 0,0,0,0,0,0,0,0)
        cap.release(); cv2.destroyAllWindows()

if __name__=="__main__":
    main()
