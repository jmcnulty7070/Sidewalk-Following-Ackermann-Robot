# Robot Control GUI for Jetson Nano

This GUI provides an integrated control panel for your SegMAV + MAVROS + MAVLink Router + LDLiDAR stack.  
It runs on Jetson Nano with ROS Noetic and PyQt5, giving you both system control and live visual feedback.

---

## Features

- **Toggle buttons** to start/stop:
  - MAVLink Router
  - MAVROS
  - LDLiDAR
  - SegMAV
- Buttons change **red → green** when health checks pass.
- **Two live panes**:
  - Segmentation preview captured directly from the Jetson Inference display.
  - LiDAR plot subscribing to `/scan`.
- **Indicators**:
  - GUIDED mode (green when `/mavros/state.mode == "GUIDED"`).
  - ARMED state (green when `/mavros/state.armed == true`).
  - Waypoints received (green after any list received on `/mavros/mission/waypoints`).
- **Waypoint list**: Textbox prints `lat, lon` for each mission item (if in global frame).
- **Reset All button**: Runs your merged script (`~/robot_stack_reset_and_start.sh`) to reset and start the stack.

---

## Installation

### Base dependencies
```bash
sudo apt-get update
sudo apt-get install -y python3-pyqt5 python3-pyqt5.qtquick python3-pyqt5.qtmultimedia                         gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad                         xdotool x11-utils x11-xserver-utils xrandr
pip3 install --user pyqtgraph opencv-python rospkg
```

### ROS dependencies
```bash
sudo apt-get install -y ros-noetic-roslaunch ros-noetic-rospy ros-noetic-sensor-msgs ros-noetic-mavros-msgs
```

---

## Running

```bash
python3 robot_control_gui.py
```

---

## SegMAV Preview Capture

The GUI captures the SegMAV window overlay using X11:

1. **Exact window capture (best):**  
   If a window titled `"SegMAV"` exists, the GUI uses `ximagesrc xid=<winid>` to capture only that window.

2. **Cropped full-screen (fallback):**  
   If the window id isn’t found, it grabs the screen and applies a `videocrop` using the window’s geometry from `xwininfo`/`xdotool`/`xrandr`.  
   This removes desktop clutter while still showing only the SegMAV region.

3. **Full-screen (last resort):**  
   If geometry can’t be determined, the GUI falls back to full-screen capture.

You can change the target window name by editing:
```python
WINDOW_TITLE = "SegMAV"
```

---

## Notes

- Make sure your SegMAV preview actually opens a window on the Jetson (`display://0`).  
  If you run headless, the GUI’s capture won’t see it.
- If `xdotool` can’t find the window, the GUI will capture the full screen or crop automatically.
- The **Reset All** button assumes your merged script is at `~/robot_stack_reset_and_start.sh`.  
  If it’s elsewhere, set `RESET_SCRIPT` at the top of the file.

---

## Example Flow

1. Launch the GUI:
   ```bash
   python3 robot_control_gui.py
   ```
2. Press **Reset All** to clear old processes and start a clean stack.
3. Toggle **MAVLink Router**, **MAVROS**, **LiDAR**, and **SegMAV** as needed.
4. Watch the SegMAV preview and LiDAR data live.
5. Monitor indicators for GUIDED, ARMED, and waypoint status.
6. View the mission waypoint list with lat/lon entries.

