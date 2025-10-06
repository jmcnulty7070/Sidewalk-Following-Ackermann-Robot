#!/usr/bin/env python3

"""
steer_sweep.py — Jetson → Pixhawk/Cube (ArduPilot Rover)
Send RC override on CH1 to sweep steering: center → right → left → center.

Safety:
- Put the rover on a stand (front wheels off the ground) for the first test.
- Ensure the safety switch is disengaged (if present) and that SERVO1 is your steering.
- This uses RC_CHANNELS_OVERRIDE; it releases override on exit.

Usage examples:
  python3 steer_sweep.py --conn udp:127.0.0.1:14551
  python3 steer_sweep.py --conn serial:/dev/telem1:115200 --min 1100 --trim 1500 --max 1900

Args:
  --conn   MAVLink connection string (udp:..., tcp:..., or serial:/dev/telem1:115200)
  --ch     RC channel number for steering (default: 1)
  --min    PWM at full-left  (default: 1100)
  --trim   PWM at center     (default: 1500)
  --max    PWM at full-right (default: 1900)
  --hold   Seconds to hold at each end/center (default: 1.0)
  --repeat Number of times to repeat the sweep (default: 1)
"""
import argparse
import sys
import time
from pymavlink import mavutil

def build_override(frame, ch_idx, pwm):
    """
    Build an RC override frame for up to 8 channels.
    ch_idx is 0-based index in [0..7]; 0 means CH1.
    pwm=0 means 'no change' for that channel.
    """
    values = [0]*8
    if 0 <= ch_idx < 8:
        values[ch_idx] = int(pwm)
    return values

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--conn", default="udp:127.0.0.1:14551", help="MAVLink connection string")
    ap.add_argument("--ch", type=int, default=1, help="RC channel for steering (1-8)")
    ap.add_argument("--min", type=int, default=1100, help="PWM for full-left")
    ap.add_argument("--trim", type=int, default=1500, help="PWM for center")
    ap.add_argument("--max", type=int, default=1900, help="PWM for full-right")
    ap.add_argument("--hold", type=float, default=1.0, help="Hold time at each position (seconds)")
    ap.add_argument("--repeat", type=int, default=1, help="Number of sweep repetitions")
    args = ap.parse_args()

    ch_idx = args.ch - 1
    if ch_idx < 0 or ch_idx > 7:
        print("Channel must be 1..8")
        sys.exit(2)

    print(f"Connecting: {args.conn}")
    m = mavutil.mavlink_connection(args.conn)
    m.wait_heartbeat()
    print(f"Heartbeat OK (sys={m.target_system}, comp={m.target_component})")

    def send_pwm(pwm):
        vals = build_override(m, ch_idx, pwm)
        m.mav.rc_channels_override_send(m.target_system, m.target_component, *vals)

    def center(): 
        print(f"→ center {args.trim}µs"); send_pwm(args.trim)

    def right():  
        print(f"→ right  {args.max}µs");  send_pwm(args.max)

    def left():   
        print(f"→ left   {args.min}µs");  send_pwm(args.min)

    try:
        # Initial center
        center()
        time.sleep(args.hold)

        for i in range(args.repeat):
            print(f"Sweep {i+1}/{args.repeat}")
            right();  time.sleep(args.hold)
            left();   time.sleep(args.hold)
            center(); time.sleep(args.hold)

        print("Done. Releasing override…")

    finally:
        # Release override (all zeros = no override)
        m.mav.rc_channels_override_send(m.target_system, m.target_component, 0,0,0,0,0,0,0,0)
        print("Override released. Bye.")

if __name__ == "__main__":
    main()
