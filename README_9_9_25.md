# SegMAV: Mission-Driven GUIDED Runner + Full Bring-Up

This guide shows how to:
- Use the mission you built in **Mission Planner**.
- Run that mission in **GUIDED** mode (so SegMAV can still add gentle velocity “nudges”).
- Launch everything in the correct order (simple, step-by-step).

---

## Option A (Recommended): Use the Mission You Loaded in Mission Planner

1. In **Mission Planner → Plan**, create your mission and click **Write** to upload it to the rover.
2. Run the updated `guided_waypoint_runner.py` with `--from-mission`.  
   This pulls the current mission from MAVROS, finds `MAV_CMD_NAV_WAYPOINT (16)` items, and walks them in **GUIDED**:
   - **lat/lon/alt** from the mission item
   - **acceptance radius** from `param2` (default **1.5 m** if ≤ 0)
   - **hold** from `param1` (default **0 s** if ≤ 0)
   - **yaw** from `param4` (unchanged if not set)

---

## Drop-In Script: `~/segmav/guided_waypoint_runner.py`

> Save this file, then `chmod +x ~/segmav/guided_waypoint_runner.py`

```python
#!/usr/bin/env python3
import csv, math, time, argparse, math as m
import rospy
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, WaypointList
from mavros_msgs.srv import CommandBool, SetMode, CommandLong, WaypointPull

EARTH_R = 6371000.0
def haversine_m(lat1, lon1, lat2, lon2):
    dlat=m.radians(lat2-lat1); dlon=m.radians(lon2-lon1)
    a=m.sin(dlat/2)**2 + m.cos(m.radians(lat1))*m.cos(m.radians(lat2))*m.sin(dlon/2)**2
    return 2*EARTH_R*m.asin(m.sqrt(a))

def read_csv(path):
    pts=[]
    with open(path,'r') as f:
        for row in csv.reader(f):
            if not row or row[0].strip().startswith('#'): continue
            lat=float(row[0]); lon=float(row[1])
            alt=float(row[2]) if len(row)>2 and row[2] else 0.0
            hold=float(row[3]) if len(row)>3 and row[3] else 0.0
            accept=float(row[4]) if len(row)>4 and row[4] else 1.5
            yaw=float(row[5]) if len(row)>5 and row[5] else float('nan')
            pts.append((lat,lon,alt,hold,accept,yaw))
    if not pts: raise RuntimeError("No waypoints in CSV")
    return pts

def read_mission_via_mavros(default_accept=1.5):
    rospy.wait_for_service("/mavros/mission/pull")
    pull = rospy.ServiceProxy("/mavros/mission/pull", WaypointPull)
    ok = pull().success
    if not ok:
        raise RuntimeError("mavros/mission/pull failed")
    msg = rospy.wait_for_message("/mavros/mission/waypoints", WaypointList, timeout=5.0)
    pts=[]
    for i,wp in enumerate(msg.waypoints):
        # MAV_CMD_NAV_WAYPOINT = 16
        if int(wp.command) != 16:
            continue
        lat, lon, alt = wp.x_lat, wp.y_long, wp.z_alt
        hold = wp.param1 if wp.param1 > 0 else 0.0          # seconds
        accept = wp.param2 if wp.param2 > 0 else default_accept
        yaw = wp.param4 if math.isfinite(wp.param4) else float('nan')
        pts.append((lat,lon,alt,hold,accept,yaw))
    if not pts:
        raise RuntimeError("Mission has no NAV_WAYPOINT (16) items")
    return pts

class GuidedRunner:
    def __init__(self, points, arm_after, ground_speed):
        self.points = points
        self.arm_after = arm_after
        self.gs = ground_speed
        self.state = State(); self.fix = NavSatFix()
        rospy.init_node("guided_waypoint_runner", anonymous=True)
        rospy.Subscriber("/mavros/state", State, self._state_cb, queue_size=1)
        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self._fix_cb, queue_size=1)
        rospy.wait_for_service("/mavros/cmd/arming")
        rospy.wait_for_service("/mavros/set_mode")
        rospy.wait_for_service("/mavros/cmd/command")
        self.srv_arm   = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.srv_mode  = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.srv_clong = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)

    def _state_cb(self, msg): self.state = msg
    def _fix_cb(self,   msg): self.fix   = msg

    def wait_connected(self, tmo=20):
        t0=time.time(); rate=rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.state.connected: return
            if time.time()-t0>tmo: raise RuntimeError("No FCU connection via MAVROS")
            rate.sleep()

    def set_guided(self):
        if not self.srv_mode(base_mode=0, custom_mode="GUIDED").mode_sent:
            raise RuntimeError("Set mode GUIDED failed")
        rospy.loginfo("GUIDED set.")

    def maybe_arm(self):
        if self.arm_after and not self.srv_arm(True).success:
            rospy.logwarn("Arming failed (continuing)")

    def do_reposition(self, lat, lon, alt=0.0, yaw_deg=float('nan'), accept_m=1.5, speed_mps=None):
        gs = 0.0 if (speed_mps is None or speed_mps <= 0) else speed_mps
        yaw = 0.0 if math.isnan(yaw_deg) else float(yaw_deg)
        # MAV_CMD_DO_REPOSITION (192)
        ok = self.srv_clong(False, 192, 0, gs, 0.0, yaw, 0.0, lat, lon, alt).success
        if not ok: raise RuntimeError("DO_REPOSITION failed")
        rospy.loginfo("Target: lat=%.7f lon=%.7f alt=%.1f accept=%.2fm", lat, lon, alt, accept_m)

    def run(self):
        self.wait_connected(); self.set_guided(); self.maybe_arm()
        rate=rospy.Rate(5)
        for i,(lat,lon,alt,hold,accept,yaw) in enumerate(self.points):
            self.do_reposition(lat,lon,alt,yaw,accept,self.gs)
            while not rospy.is_shutdown():
                if self.fix.status.status != 0:
                    d = haversine_m(self.fix.latitude, self.fix.longitude, lat, lon)
                    rospy.loginfo_throttle(2.0, "WP %d: dist=%.2f m (accept=%.2f)", i, d, accept)
                    if d <= accept: break
                rate.sleep()
            rospy.loginfo("WP %d reached. Holding %.1fs", i, hold)
            t_end=time.time()+hold
            while not rospy.is_shutdown() and time.time()<t_end: rate.sleep()
        rospy.loginfo("Mission complete (GUIDED).")

if __name__=="__main__":
    import math
    ap = argparse.ArgumentParser()
    ap.add_argument("--file", help="CSV with lat,lon,alt,hold_s,accept_m,yaw_deg")
    ap.add_argument("--from-mission", action="store_true", help="Use current FCU mission (pulled via MAVROS)")
    ap.add_argument("--arm", action="store_true", help="Arm the rover")
    ap.add_argument("--speed", type=float, default=0.0, help="Ground speed m/s (0=unchanged)")
    args = ap.parse_args()
    if args.from_mission and args.file:
        raise SystemExit("--file and --from-mission are mutually exclusive")
    if args.from_mission:
        pts = read_mission_via_mavros()
    elif args.file:
        pts = read_csv(args.file)
    else:
        raise SystemExit("Specify either --from-mission or --file <csv>")
    GuidedRunner(pts, args.arm, args.speed).run()
```

Make it executable:
```bash
chmod +x ~/segmav/guided_waypoint_runner.py
```

Run it against the mission you wrote from Mission Planner:
```bash
env -i bash --noprofile --norc -c '
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
python3 ~/segmav/guided_waypoint_runner.py --from-mission --arm --speed 1.2
'
```

> `--arm` arms for you (remove it to arm manually).  
> `--speed 1.2` ~ 1.2 m/s; adjust as you like.

---

## “8th-Grade-Level” Bring-Up Order (Everything in the Right Order)

Open a new terminal for each “Terminal X” step.

### 0) Clean slate (stop old stuff)
```bash
sudo pkill mavlink-routerd; pkill -f mavros_node; pkill -f mavsegmav_merged.py; pkill -f scan_to_obstacle_; pkill -f roscore || true
```

### Terminal A — Start ROS (the traffic cop)
```bash
source /opt/ros/noetic/setup.bash
roscore
```

### Terminal B — Start MAVLink Router (wires everything)
```bash
source /opt/ros/noetic/setup.bash
sudo mavlink-routerd -v -c /etc/mavlink-router/main.conf
```
You should see: UART `/dev/telem1`, UDP `14550 / 14553 / 14555 / 14601`.

### Terminal C — Start the LiDAR (robot “eyes”)
```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch ldlidar_stl_ros ld19.launch
# Check:
rostopic hz /scan   # ~9–10 Hz
```

### Terminal D — Start Proximity bridge (LiDAR → OBSTACLE_DISTANCE)
```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export MAVLINK20=1
rosrun scan_to_obstacle scan_to_obstacle_mavlink.py \
  _scan:=/scan _sectors:=72 _min_range_m:=0.20 _max_range_m:=12.0 \
  _yaw_offset_deg:=0.0 _mavlink_out:=udpout:127.0.0.1:14553
```
If the **Proximity** ring looks rotated in Mission Planner, restart with `_yaw_offset_deg:=90` (or `-90`, `180`).

### Terminal E — Start MAVROS (so scripts can talk to the FCU)
```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch segmav_bringup mavros_router.launch \
  fcu_url:=udp://0.0.0.0:14600@127.0.0.1:14553
# Check:
rostopic echo -n1 /mavros/state   # should be connected: True
```

### Terminal F — Start SegMAV (gentle nudges)
```bash
python3 ~/segmav/mavsegmav_merged.py --headless
# Defaults inside script: TX→14555, RX←14601
```

### Mission Planner (on your PC)
- In **Plan** tab, load or make your mission.
- Click **Write** to upload.
- **Ctrl-F → Mavlink Inspector**: `OBSTACLE_DISTANCE` should tick > 0 Hz.
- **Ctrl-F → Proximity**: the 360° ring should render.

### Terminal G — Run the GUIDED runner (walks your uploaded mission)
```bash
env -i bash --noprofile --norc -c '
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
python3 ~/segmav/guided_waypoint_runner.py --from-mission --arm --speed 1.2
'
```

---

## Quick Health Checks

```bash
# Router & sockets
sudo ss -lunp | egrep '14550|14553|14555|14600|14601'

# LiDAR topic rate
rostopic hz /scan

# MAVROS connected?
rostopic echo -n1 /mavros/state
```

**Rover params (once):**  
`PRX_TYPE=5`, `AVOID_ENABLE=1`, `OA_TYPE=1` (BendyRuler). Optional: `AVOID_MARGIN=1.0`.

**Tip:** SegMAV velocity nudges run *at the same time* as GUIDED targets.

**Stop everything:** press **Ctrl-C** in each terminal.  
To stop router if daemonized: `sudo systemctl stop mavlink-router`.

---

## Option B: “Go Here” Clicks From Mission Planner (FYI)

When you right‑click → **Guided → Go Here**, Mission Planner sends `DO_REPOSITION (192)` or `SET_POSITION_TARGET_GLOBAL_INT`.  
You *could* mirror those via a small ROS node (e.g., subscribe to `/mavros/setpoint_raw/global`), but **Option A** (pull the mission) is simpler and sturdier for multi‑waypoint runs.

---
