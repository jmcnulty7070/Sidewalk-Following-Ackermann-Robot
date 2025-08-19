#!/usr/bin/env python3
import math, time, yaml, signal, sys
import rospy
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import SetMode
from geographiclib.geodesic import Geodesic

def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    dlat = math.radians(lat2-lat1)
    dlon = math.radians(lon2-lon1)
    a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1))*math.cos(math.radians(lat2))*math.sin(dlon/2)**2
    return 2*R*math.asin(math.sqrt(a))

def make_vel_yawrate_msg(vx_mps, yaw_rate_rps):
    m = PositionTarget()
    m.coordinate_frame = PositionTarget.FRAME_BODY_NED  # forward in body-X
    IGN_XYZ     = (1<<0) | (1<<1) | (1<<2)
    IGN_VYVZ    = (1<<4) | (1<<5)
    IGN_ACCEL   = (1<<6) | (1<<7) | (1<<8) | (1<<9)
    IGN_YAW     = (1<<10)                # ignore absolute yaw
    # we WILL use yaw_rate, so do NOT set bit 11
    m.type_mask = IGN_XYZ | IGN_VYVZ | IGN_ACCEL | IGN_YAW
    m.velocity.x = float(vx_mps)
    m.velocity.y = 0.0
    m.velocity.z = 0.0
    m.yaw_rate   = float(yaw_rate_rps)
    return m

class TurnManager:
    def __init__(self, yaml_path):
        rospy.init_node("guided_turn_manager", anonymous=True)
        self.fix = None
        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.cb_fix, queue_size=1)
        self.pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=50)
        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        with open(yaml_path, "r") as f:
            cfg = yaml.safe_load(f)
        self.turns = cfg.get("turns", [])
        self.done = set()
        self.stop = False

    def cb_fix(self, msg): self.fix = msg

    def ensure_guided(self):
        try:
            self.set_mode(base_mode=0, custom_mode="GUIDED")
        except Exception:
            pass

    def stream_for(self, vx, yaw_rate_rps, duration_s, rate_hz=30.0):
        r = rospy.Rate(rate_hz)
        m = make_vel_yawrate_msg(vx, yaw_rate_rps)
        t0 = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            if self.stop: break
            self.pub.publish(m)
            if rospy.Time.now().to_sec() - t0 >= duration_s:
                break
            r.sleep()

    def soft_stop(self, rate_hz=20.0, n=10):
        m = make_vel_yawrate_msg(0.0, 0.0)
        r = rospy.Rate(rate_hz)
        for _ in range(n):
            self.pub.publish(m); r.sleep()

    def run(self):
        rospy.loginfo("[turns] Ready. Waiting for GPS fix…")
        rate = rospy.Rate(5)
        while not rospy.is_shutdown() and self.fix is None:
            rate.sleep()
        rospy.loginfo("[turns] Got GPS. Managing turn zones.")

        while not rospy.is_shutdown() and not self.stop:
            if self.fix is None:
                rate.sleep(); continue

            lat = self.fix.latitude
            lon = self.fix.longitude

            for i, t in enumerate(self.turns):
                if i in self.done: continue
                d = haversine_m(lat, lon, t["center_lat"], t["center_lon"])
                if d <= float(t["enter_radius_m"]):
                    rospy.loginfo(f"[turns] Entered zone '{t['name']}' (d={d:.1f} m) → performing arc")
                    self.ensure_guided()

                    # 1) Instruct SegMAV to pause steering (optional: via a topic/param if you added one).
                    # If you don't have a pause hook, pick stronger yaw_rate so it dominates for the short turn.

                    # 2) Perform the turn arc
                    vx = float(t["speed_mps"])
                    yaw_rps = math.radians(float(t["yaw_rate_dps"]))
                    dur = float(t["duration_s"])
                    self.stream_for(vx, yaw_rps, dur, rate_hz=30.0)

                    # 3) Hand steering back to SegMAV by resuming straight speed (no yaw_rate)
                    resume = float(t.get("resume_speed_mps", vx))
                    self.stream_for(resume, 0.0, 1.0, rate_hz=20.0)
                    self.soft_stop(n=2)  # tiny smoothing
                    self.done.add(i)
                    rospy.loginfo(f"[turns] Zone '{t['name']}' complete. Resumed straight run.")
            rate.sleep()

    def request_stop(self, *_):
        self.stop = True
        self.soft_stop()

def main():
    if len(sys.argv) != 2:
        print("Usage: guided_turn_manager.py /path/to/turns.yaml")
        sys.exit(1)
    mgr = TurnManager(sys.argv[1])
    signal.signal(signal.SIGINT, mgr.request_stop)
    signal.signal(signal.SIGTERM, mgr.request_stop)
    mgr.run()

if __name__ == "__main__":
    main()
