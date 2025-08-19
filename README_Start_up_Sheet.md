# 🚗 Sidewalk GPS Robot – Manual Ops Startup Sheet

This sheet covers **manual startup** for the SegMAV + GUIDED mission streamer system.  
Each step is single-line, easy to run, and tested for the NVIDIA Jetson + CrossFlight/Pixhawk stack.

---

## 0) Environment Setup (per new terminal)

```bash
source /opt/ros/noetic/setup.bash
[ -f ~/catkin_ws/devel/setup.bash ] && source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=127.0.0.1
```

---

## 1) Start MAVLink Router (FC → UDP)

```bash
sudo systemctl restart mavlink-router
sudo systemctl status mavlink-router --no-pager
sudo ss -ulpn | egrep '14550|14551|mavlink-routerd'
```

**Ports**
- `14550` → Mission Planner  
- `14551` → SegMAV + MAVROS  

---

## 2) Start MAVROS (binds to 14551)

```bash
roslaunch mavros apm.launch fcu_url:=udp://0.0.0.0:14551@
```

**Quick checks:**
```bash
rostopic echo -n1 /mavros/state
rostopic echo -n1 /mavros/global_position/raw/fix
```

---

## 3) Start LiDAR (optional: RViz + MP Proximity)

```bash
rosrun ldlidar_stl_ros ldlidar_stl_ros_node \
  _product_name:=LDLiDAR_LD19 \
  _port_name:=/dev/ldlidar \
  _port_baudrate:=230400 \
  _frame_id:=laser \
  _topic_name:=/scan
```

**Verify:**
```bash
rostopic hz /scan
```

**Relay for Mission Planner proximity:**
```bash
rosrun topic_tools relay /scan /mavros/obstacle/send
```

---

## 4) Start SegMAV (sidewalk follower / nudge)

```bash
cd ~/segmav
python3 mavsegmav_merged.py \
  --device=udpin:127.0.0.1:14551 \
  --input=v4l2:///dev/Astra_rgb \
  --preview
```

- Use `--preview` for HDMI overlay.  
- Drop `--preview` for headless mode.  

---

## 5) (Optional) Tiny GUIDED Waypoint Streamer

Streams Mission Planner waypoints one-by-one while SegMAV keeps car centered.

```bash
cd ~/segmav
python3 guided_mission.py --device=udpin:127.0.0.1:14551
```

---

## 6) Mission Flow

1. Power robot, confirm GNSS fix in Mission Planner.  
2. Set **GUIDED** mode + ARM.  
3. Upload waypoints in Mission Planner *(or run `guided_mission.py`)*.  
4. SegMAV window shows segmentation overlay.  
5. Drive:  
   - In GUIDED: click-to-go in MP or let streamer push next point.  
   - SegMAV keeps vehicle re-centered on sidewalk.  
6. To stop: **Disarm** or switch HOLD/LOITER.  

---

## 7) Quick Troubleshooting

**Router won’t start**
```bash
sudo journalctl -u mavlink-router -n 100 --no-pager
```

**Port busy / bind error**
```bash
sudo ss -ulpn | egrep '14550|14551|mavlink-routerd|mavros'
```
Stop duplicates, then restart.

**No LiDAR in RViz / Mission Planner**
```bash
ls -l /dev/ldlidar
rostopic hz /scan
rosnode list | grep topic_tools
```

**SegMAV no video**
```bash
ls -l /dev/Astra_rgb
python3 mavsegmav_merged.py --input=v4l2:///dev/Astra_rgb --preview
```

**No FCU link in MAVROS**
- MAVROS log must show: `Got HEARTBEAT, connected`.  
- If not: check router config + ensure only one process binds `14551`.  

---

## 8) Safety & Mode Tips

- **GUIDED** = SegMAV centering + streamed targets (click-to-go or `guided_mission.py`).  
- **AUTO** = ArduRover runs mission autonomously (no SegMAV corrections).  
- For **tight turns (2m radius Ackermann)**:  
  - Space waypoints wider in Mission Planner.  
  - SegMAV re-centers back to sidewalk after maneuver.  

---

✅ **That’s it — predictable startup, modular pieces, and safe recovery.**  
Tape this sheet to the robot for field ops.
