#!/usr/bin/env python3
# auto_nudge_segnet_preview.py
# Defaults baked to your setup:
# - MAVLink: /dev/telem1,115200  (Jetson FTDI -> Cube TELEM1)
# - Camera:  v4l2:///dev/video4  (Intel RealSense RGB UVC)
# - SegNet:  Cityscapes, class_id=8 (sidewalk)
# - CH5:     3-pos switch (LOW=off, MID=preview, HIGH=NUDGE)
import time, argparse
import cv2
from pymavlink import mavutil
from backend_segnet import SegNetBackend
from common_overlay import draw_steven_overlay, AngleSmoother3, flip_img, YELLOW

def get_rc_ch5(m):
    msg = m.recv_match(type="RC_CHANNELS", blocking=False)
    return getattr(msg, 'chan5_raw', None) if msg else None

def rc_override_steer(m, steer_pwm=None):
    ch = [0]*8
    if steer_pwm is not None:
        ch[0] = int(steer_pwm)  # CH1
    m.mav.rc_channels_override_send(m.target_system, m.target_component, *ch)

def main():
    ap = argparse.ArgumentParser("AUTO + SegNet sidewalk nudges with Steven-style preview")
    ap.add_argument("--device", default="/dev/telem1,115200")
    ap.add_argument("--camera", default="v4l2:///dev/video4")
    ap.add_argument("--flip", default="none")
    ap.add_argument("--network", default="fcn-resnet18-cityscapes")
    ap.add_argument("--class-id", type=int, default=8)
    ap.add_argument("--hz", type=float, default=15.0)
    ap.add_argument("--neutral", type=int, default=1500)
    ap.add_argument("--kp_us_per_deg", type=float, default=4.0)
    ap.add_argument("--max_delta_us", type=int, default=120)
    ap.add_argument("--deadband_deg", type=float, default=2.0)
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.camera.replace("v4l2://", ""))
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera {args.camera}")

    seg = SegNetBackend(network=args.network, class_id=args.class_id)

    if args.device.startswith("/dev/") and "," in args.device:
        port, baud = args.device.split(",", 1)
        m = mavutil.mavlink_connection(port, baud=int(baud))
    else:
        m = mavutil.mavlink_connection(args.device)
    m.wait_heartbeat()
    print("[MAV] connected")

    smoother = AngleSmoother3()
    period = 1.0 / args.hz
    last = 0.0

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.01)
                continue
            frame = flip_img(frame, args.flip)

            ch5 = get_rc_ch5(m)
            state = "OFF" if (ch5 is None or ch5 < 1300) else ("PREVIEW" if ch5 < 1700 else "NUDGE")

            mask = seg.infer_mask(frame)
            angle_deg_raw, overlay = draw_steven_overlay(frame, mask)
            angle_deg = smoother.add(angle_deg_raw)

            cv2.putText(overlay, f"CH5={state}  angle={angle_deg:+.1f} deg",
                        (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, YELLOW, 2)
            h, w = overlay.shape[:2]
            small = cv2.resize(mask, (w//4, h//4))
            overlay[0:h//4, w-w//4:w] = cv2.cvtColor(small, cv2.COLOR_GRAY2BGR)
            cv2.imshow("AUTO + SegNet (sidewalk) — nudges", overlay)
            cv2.waitKey(1)

            now = time.time()
            if state == "NUDGE" and (now - last) >= period:
                err = angle_deg
                if abs(err) < args.deadband_deg:
                    steer_pwm = args.neutral
                else:
                    delta = args.kp_us_per_deg * err
                    delta = max(-args.max_delta_us, min(args.max_delta_us, delta))
                    steer_pwm = int(args.neutral + delta)
                rc_override_steer(m, steer_pwm)
                last = now
            elif state != "NUDGE":
                rc_override_steer(m, None)

            time.sleep(0.005)

    finally:
        rc_override_steer(m, None)
        cap.release(); cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
