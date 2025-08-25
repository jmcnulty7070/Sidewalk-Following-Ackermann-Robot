# SegMAV Car Mission (GUIDED) — Read Me

This guide shows how to run a simple “drive to points” mission with a rover.
We run the mission in **GUIDED** mode so **SegMAV** can add **velocity nudges**
(small steering/speed corrections) while the rover heads to each target.

It’s step-by-step, with copy-paste commands.

---

## 🛡️ Safety first

- **E‑stop:** Keep your emergency stop within reach. Know how to stop the car instantly.
- **Area:** Use a clear, open space. No people or pets on the path.
- **First power‑on test:** Put the rover on stands so wheels can spin safely.
- **RC transmitter:**
  - Power **ON** and linked.
  - Throttle/steering centered; trims centered.
  - **Mode switch:** set to a position that **allows GUIDED from the ground station.** Avoid positions that force MANUAL/AUTO if you want the computer to take GUIDED.
- **Arming / safety switch (if present on the vehicle):**
  - Disengage only when you’re ready to move.
  - Know how to re‑engage to stop the vehicle.
- **Power & wiring:** LiDAR, computer, and FCU are powered and cabled firmly.

---

## 🔧 One-time FCU parameters (ArduPilot Rover)

Set these once (Mission Planner → Config → Full Parameter List), then **reboot** the flight controller:

- `PRX_TYPE = 2` (MAVLink)  
- `OA_TYPE = 1` (BendyRuler)  
- `AVOID_ENABLE = 7`

These enable obstacle messages from the LiDAR and onboard avoidance.

---

## ✅ What you need already set up

- Ubuntu 20.04 with **ROS Noetic**.
- `mavlink-router` configured to use the serial port **/dev/telem1** at **115200**.
- A persistent LiDAR symlink: **/dev/ldlidar** (points at the correct USB device).
- Your catkin workspace builds the **`ldlidar_stl_ros`** package.
- SegMAV repo at `~/segmav`.
- Two helper scripts present:
  - `~/segmav/guided_waypoint_runner.py` (below)
  - `~/segmav/scan_to_distance_sensor_mavlink.py` (PRX bridge; created earlier)

---

## 🗺️ Waypoints (what the car will drive to)

Create or edit this file to list your points:

```bash
tee ~/segmav/waypoints.csv >/dev/null <<'CSV'
# lat,lon,alt_m,hold_s,accept_m,yaw_deg
47.3977419, 8.5455938, 0, 0, 1.5, 0
47.3979000, 8.5458000, 0, 0, 1.5, 0
47.3980500, 8.5456000, 0, 0, 1.5, 0
CSV
```

- **lat, lon** are in degrees.  
- **alt_m** can be `0` for Rover (required by MAVLink but ignored).  
- **hold_s** is the wait time after reaching a point.  
- **accept_m** is how close (meters) counts as “reached.”  
- **yaw_deg** is optional (heading at arrival). Use `0` if unsure.

---

## 🧲 Physical switches & mode checklist (before driving)

- **RC Tx ON**, receiver bound, battery OK.  
- **Mode switch** on the TX set to a position that **allows GUIDED** via GCS (doesn’t force MANUAL/AUTO).  
- **E‑stop** released only when safe to move; know where it is.  
- **Safety/arming switch** (if fitted) **disengaged** when ready.  
- **Wheel chocks/stands** removed right before moving.

---

## ▶️ Start everything (each section = its own terminal)

### 1) Router (owns the serial: `/dev/telem1` @ 115200)
```bash
sudo systemctl restart mavlink-router
journalctl -u mavlink-router -b --no-pager | tail -n 20
sudo lsof /dev/telem1      # should show mavlink-routerd holding it
```

### 2) ROS master
```bash
env -i bash --noprofile --norc
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
roscore
```

### 3) MAVROS (listens for FCU data on UDP 14551)
```bash
env -i bash --noprofile --norc
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
roslaunch mavros apm.launch fcu_url:=udp://:14551@

# Check connection:
# rostopic echo -n1 /mavros/state   # should show: connected: True
```

### 4) LiDAR node → publishes `/scan`
```bash
# workspace shell so ROS can find the LiDAR package
bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1

rosrun ldlidar_stl_ros ldlidar_stl_ros_node   _product_name:=LDLiDAR_LD19   _port_name:=/dev/ldlidar   _frame_id:=laser   _port_baudrate:=230400

# In another tab you can check:
# rostopic hz /scan
```

### 5) Proximity bridge (turns `/scan` into DISTANCE_SENSOR for the FCU)
This makes Mission Planner **Proximity** show the arcs.

```bash
# clean apt shell (no workspace overlay)
env -i bash --noprofile --norc
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1

python3 ~/segmav/scan_to_distance_sensor_mavlink.py
```
> If the arcs look rotated (front arc isn’t in front), edit the script’s `ANGLE_OFFSET_DEG` to `-90`, `+90`, or `180`, save, and run again.

### 6) SegMAV (velocity nudges)
```bash
cd ~/segmav
python3 mavsegmav_merged.py --device=udpin:0.0.0.0:14555 --preview
```

SegMAV publishes **velocity-only** nudges that blend with GUIDED targets.

### 7) GUIDED multi‑waypoint runner
This sets **GUIDED**, optionally arms, and walks through your CSV.

```bash
env -i bash --noprofile --norc
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1

python3 ~/segmav/guided_waypoint_runner.py --file ~/segmav/waypoints.csv --arm --speed 1.2
```
- `--arm` arms the rover (remove it to arm manually).
- `--speed 1.2` sets desired ground speed to ~1.2 m/s (change as needed).

---

## 🧠 What the GUIDED runner does

1. Switches the FCU to **GUIDED** (not AUTO).  
2. (Optional) **Arms** the rover.  
3. For each point in the CSV:
   - Sends **MAV_CMD_DO_REPOSITION** to set the target.
   - Waits until the rover is within **`accept_m`** meters.
   - Waits **`hold_s`** seconds (if any).
   - Proceeds to the next point.
4. While it runs, **SegMAV** can gently push with velocity nudges.

---

## 🛠️ Create the GUIDED runner (only if you don’t have it yet)

```bash
tee ~/segmav/guided_waypoint_runner.py >/dev/null <<'PY'
#!/usr/bin/env python3
import csv, math, time, argparse
import rospy
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandLong

EARTH_R = 6371000.0  # meters
def haversine_m(lat1, lon1, lat2, lon2):
    dlat = math.radians(lat2-lat1); dlon = math.radians(lon2-lon1)
    a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1))*math.cos(math.radians(lat2))*math.sin(dlon/2)**2
    return 2*EARTH_R*math.asin(math.sqrt(a))

def read_csv(path):
    pts=[]
    with open(path,'r') as f:
        for row in csv.reader(f):
            if not row or row[0].strip().startswith('#'): continue
            lat=float(row[0]); lon=float(row[1]); alt=float(row[2]) if len(row)>2 and row[2] else 0.0
            hold=float(row[3]) if len(row)>3 and row[3] else 0.0
            accept=float(row[4]) if len(row)>4 and row[4] else 1.5
            yaw=float(row[5]) if len(row)>5 and row[5] else float('nan')
            pts.append((lat,lon,alt,hold,accept,yaw))
    if not pts: raise RuntimeError("No waypoints in CSV")
    return pts

class GuidedRunner:
    def __init__(self, csv_file, arm_after, ground_speed):
        self.points = read_csv(csv_file)
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
        ok = self.srv_clong(False, 192, 0, gs, 0.0, yaw, 0.0, lat, lon, alt).success
        if not ok: raise RuntimeError("DO_REPOSITION failed")
        rospy.loginfo("Target: lat=%.7f lon=%.7f accept=%.2fm", lat, lon, accept_m)
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
    ap = argparse.ArgumentParser()
    ap.add_argument("--file", required=True, help="CSV with lat,lon,alt,hold_s,accept_m,yaw_deg")
    ap.add_argument("--arm", action="store_true", help="Arm the rover")
    ap.add_argument("--speed", type=float, default=0.0, help="Ground speed m/s (0=unchanged)")
    args = ap.parse_args()
    GuidedRunner(args.file, args.arm, args.speed).run()
PY
chmod +x ~/segmav/guided_waypoint_runner.py
```

---

## 👀 What to watch in Mission Planner

- **Messages** tab should show `Mode GUIDED` after you start the runner.  
- **Ctrl‑F → MAVLink Inspector → DISTANCE_SENSOR** should show changing values for several IDs.  
- **Data → Proximity** should show arcs that shrink when an obstacle is nearby.

If Proximity is blank:
- Ensure the LiDAR is publishing: `rostopic hz /scan` shows a steady rate.
- Make sure only **one** PRX bridge is running (use the **DISTANCE_SENSOR** bridge).
- Put a box within ~2–3 m of the sensor and check Inspector values drop.
- Adjust the bridge’s `ANGLE_OFFSET_DEG` if the arcs look rotated.

---

## 🛑 How to stop

In each terminal:
- Press **Ctrl‑C** to stop that program (LiDAR, bridge, SegMAV, runner).
- To stop ROS master: Ctrl‑C in the `roscore` terminal.
- To stop the router: `sudo systemctl stop mavlink-router`.

---

## 🧩 Troubleshooting quick list

- **MAVROS not connected?**  
  `rostopic echo -n1 /mavros/state` should show `connected: True`.  
  If not, check the router logs and confirm UDP 14551 is open.

- **No `/scan`?**  
  The LiDAR node likely isn’t reading the device. Check your `_product_name`, `_port_name`, and permissions.

- **Proximity not showing?**  
  Run only `scan_to_distance_sensor_mavlink.py`, move an object close, and check Inspector.

- **Not moving in GUIDED?**  
  Try `--arm` on the runner, ensure no RC switch is forcing another mode, and make sure GPS has a fix.
