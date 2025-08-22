#!/usr/bin/env python3
# -*- coding: utf-8 -*-
\"\"\"
Robot Control GUI for Jetson Nano
- Toggle buttons for: MAVLink Router, MAVROS, LDLiDAR, SegMAV
- Buttons turn red -> green when health checks pass
- Segmentation preview captured from Jetson display:
    1) Prefer exact window capture via xid (xdotool).
    2) If xid not found, crop the full-screen capture to the window rectangle (xwininfo/xrandr + videocrop).
    3) Final fallback: full-screen capture.
- Live LiDAR /scan plot
- Indicators: GUIDED, ARMED, Waypoints
- Waypoint list (lat, lon)
- Reset All button (runs merged reset/start script)
\"\"\"

import sys
import subprocess
import threading
import time
from pathlib import Path

from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

# Try to import ROS
try:
    import rospy
    from sensor_msgs.msg import LaserScan
    from mavros_msgs.msg import State, WaypointList
    ROS_AVAILABLE = True
except Exception as e:
    ROS_AVAILABLE = False
    print("[WARN] ROS not fully available in GUI:", e)

# Try to import OpenCV for preview
try:
    import cv2
    OPENCV_AVAILABLE = True
except Exception as e:
    OPENCV_AVAILABLE = False
    print("[WARN] OpenCV not available for preview:", e)

###############################################
# Configuration
###############################################
# Commands
CMD_MAVLINK_ROUTER_RESTART = ["sudo", "systemctl", "restart", "mavlink-router"]
CMD_MAVLINK_ROUTER_STOP    = ["sudo", "systemctl", "stop", "mavlink-router"]

CMD_MAVROS_START = ["bash", "-lc", "nohup roslaunch mavros apm.launch fcu_url:=udp://0.0.0.0:14551@ > ~/mavros.log 2>&1 &"]
CMD_MAVROS_STOP  = ["bash", "-lc", "pkill -f 'roslaunch mavros apm.launch' || true"]

CMD_LDLIDAR_START = ["bash", "-lc",
    "nohup rosrun ldlidar_stl_ros ldlidar_stl_ros_node "
    "_product_name:=LDLiDAR_LD19 "
    "_port_name:=/dev/ldlidar "
    "_port_baudrate:=230400 "
    "_frame_id:=laser "
    "_topic_name:=/scan > ~/ldlidar.log 2>&1 &"
]
CMD_LDLIDAR_STOP = ["bash", "-lc", "pkill -f ldlidar_stl_ros_node || true"]

SEGMAV_DIR = str(Path.home() / "segmav")
SEGMAV_SCRIPT = "mavsegmav_merged.py"
CMD_SEGMAV_START = ["bash", "-lc",
    f"cd {SEGMAV_DIR} && nohup python3 {SEGMAV_SCRIPT} "
    "--device=udpin:127.0.0.1:14551 "
    "--input=v4l2:///dev/Astra_rgb "
    "--preview > ~/segmav.log 2>&1 &"
]
CMD_SEGMAV_STOP = ["bash", "-lc", "pkill -f mavsegmav_merged.py || true"]

# Reset/Start merged script path (edit if different)
RESET_SCRIPT = str(Path.home() / "robot_stack_reset_and_start.sh")
CMD_RESET_ALL = ["bash", "-lc", f"chmod +x {RESET_SCRIPT} && {RESET_SCRIPT}"]

# Health checks
CHECK_UDP_PORTS_CMD = ["bash", "-lc", "ss -ulpn | egrep '14550|14551|mavlink-routerd' || true"]
CHECK_MAVROS_TOPIC = "/mavros/state"
CHECK_LIDAR_TOPIC = "/scan"

# Segmentation window title to target for crop
WINDOW_TITLE = "SegMAV"  # change to match your on-screen window title
SCREEN_FPS = 30

###############################################
# X11 capture pipeline builders
###############################################
def run(cmd):
    return subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=False)

def x11_pipeline_for_xid(title: str):
    # Try to get window id via xdotool
    res = run(["bash", "-lc", f"xdotool search --name '{title}' | head -n1"])
    xid = res.stdout.strip()
    if xid:
        # Capture that specific window id (no desktop clutter)
        return f"ximagesrc xid={xid} use-damage=0 show-pointer=false ! video/x-raw,framerate={SCREEN_FPS}/1 ! videoconvert ! appsink"
    return None

def get_window_geometry(title: str):
    # Try xdotool geometry (returns lines like: X=..., Y=..., WIDTH=..., HEIGHT=...)
    res = run(["bash", "-lc", f"xdotool search --name '{title}' | head -n1"])
    xid = res.stdout.strip()
    if xid:
        g = run(["bash", "-lc", f"xdotool getwindowgeometry --shell {xid}"])
        if g.returncode == 0:
            geom = {}
            for line in g.stdout.splitlines():
                if "=" in line:
                    k, v = line.strip().split("=", 1)
                    geom[k] = v
            try:
                return {
                    "x": int(geom.get("X", "0")),
                    "y": int(geom.get("Y", "0")),
                    "w": int(geom.get("WIDTH", "0")),
                    "h": int(geom.get("HEIGHT", "0")),
                }
            except Exception:
                pass
    # Fallback: xwininfo by name (first match)
    res = run(["bash", "-lc", f"xwininfo -name '{title}' 2>/dev/null"])
    if res.returncode == 0 and "Absolute upper-left X" in res.stdout:
        vals = {}
        for line in res.stdout.splitlines():
            if ":" in line:
                k, v = [s.strip() for s in line.split(":", 1)]
                vals[k] = v
        try:
            return {
                "x": int(vals.get("Absolute upper-left X", "0")),
                "y": int(vals.get("Absolute upper-left Y", "0")),
                "w": int(vals.get("Width", "0")),
                "h": int(vals.get("Height", "0")),
            }
        except Exception:
            pass
    return None

def get_screen_size():
    # Try xrandr to find current primary resolution
    res = run(["bash", "-lc", "xrandr | awk '/\\*/{print $1; exit}'"])
    if res.returncode == 0 and "x" in res.stdout:
        try:
            w, h = res.stdout.strip().split("x")
            return int(w), int(h)
        except Exception:
            pass
    # Fallback to xdpyinfo
    res = run(["bash", "-lc", "xdpyinfo | awk -F'[ x]+' '/dimensions:/{print $3 \"x\" $4; exit}'"])
    if res.returncode == 0 and "x" in res.stdout:
        try:
            w, h = res.stdout.strip().split("x")
            return int(w), int(h)
        except Exception:
            pass
    return None, None

def x11_pipeline_cropped(title: str):
    geom = get_window_geometry(title)
    sw, sh = get_screen_size()
    if not geom or not sw or not sh:
        return None
    left = max(0, geom["x"])
    top = max(0, geom["y"])
    right = max(0, sw - (geom["x"] + geom["w"]))
    bottom = max(0, sh - (geom["y"] + geom["h"]))
    # Crop the screen capture to just the window rectangle
    return (
        "ximagesrc use-damage=0 show-pointer=false ! "
        f"video/x-raw,framerate={SCREEN_FPS}/1 ! "
        f"videocrop left={left} right={right} top={top} bottom={bottom} ! "
        "videoconvert ! appsink"
    )

def x11_pipeline_fullscreen():
    return f"ximagesrc use-damage=0 show-pointer=false ! video/x-raw,framerate={SCREEN_FPS}/1 ! videoconvert ! appsink"

###############################################
# Helpers
###############################################
def run_cmd(cmd):
    try:
        return subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=False)
    except Exception:
        return None

def process_running_grep(pattern):
    cmd = ["bash", "-lc", f"ps aux | egrep -E '{pattern}' | egrep -v egrep"]
    res = run_cmd(cmd)
    if not res:
        return False
    return len(res.stdout.strip()) > 0

def udp_ports_ok():
    res = run_cmd(CHECK_UDP_PORTS_CMD)
    return bool(res and res.stdout and "mavlink-routerd" in res.stdout)

def rostopic_echo_once(topic, timeout=2.0):
    cmd = ["bash", "-lc", f"timeout {timeout} rostopic echo -n1 {topic}"]
    res = run_cmd(cmd)
    return bool(res and res.returncode == 0)

###############################################
# ROS LiDAR Subscriber (thread-safe buffer)
###############################################
class LidarBuffer(QtCore.QObject):
    updated = QtCore.pyqtSignal()
    def __init__(self):
        super().__init__()
        self.lock = threading.Lock()
        self.angles = None
        self.ranges = None

    def callback(self, msg):
        angle = msg.angle_min
        angles = []
        for _ in msg.ranges:
            angles.append(angle)
            angle += msg.angle_increment
        with self.lock:
            self.angles = angles
            self.ranges = list(msg.ranges)
        self.updated.emit()

###############################################
# Main GUI
###############################################
class RobotControlGUI(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Sidewalk GPS Robot Control")
        self.setMinimumSize(1250, 950)

        # State
        self.seg_cap = None
        self.seg_timer = QtCore.QTimer()
        self.seg_timer.timeout.connect(self.update_seg_frame)

        self.lidar_buf = LidarBuffer()
        self.lidar_buf.updated.connect(self.update_lidar_plot)

        self.mavlink_on = False
        self.mavros_on = False
        self.lidar_on = False
               self.segmav_on = False

        self.guided_mode = False
        self.armed_state = False
        self.have_waypoints = False

        # ROS state buffers
        self._waypoint_text = ""

        # UI
        self.build_ui()

        # Background health checks
        self.health_timer = QtCore.QTimer()
        self.health_timer.timeout.connect(self.refresh_health)
        self.health_timer.start(2000)

        # Initialize ROS subscribers
        if ROS_AVAILABLE:
            threading.Thread(target=self.init_ros, daemon=True).start()

    def build_ui(self):
        layout = QtWidgets.QVBoxLayout(self)

        # Top row: toggle buttons + Reset
        btn_row = QtWidgets.QHBoxLayout()

        self.btn_mavlink = QtWidgets.QPushButton("MAVLink Router")
        self.btn_mavlink.setCheckable(True)
        self.btn_mavlink.clicked.connect(self.toggle_mavlink)
        btn_row.addWidget(self.btn_mavlink)

        self.btn_mavros = QtWidgets.QPushButton("MAVROS")
        self.btn_mavros.setCheckable(True)
        self.btn_mavros.clicked.connect(self.toggle_mavros)
        btn_row.addWidget(self.btn_mavros)

        self.btn_lidar = QtWidgets.QPushButton("LDLiDAR")
        self.btn_lidar.setCheckable(True)
        self.btn_lidar.clicked.connect(self.toggle_lidar)
        btn_row.addWidget(self.btn_lidar)

        self.btn_segmav = QtWidgets.QPushButton("SegMAV")
        self.btn_segmav.setCheckable(True)
        self.btn_segmav.clicked.connect(self.toggle_segmav)
        btn_row.addWidget(self.btn_segmav)

        self.btn_reset = QtWidgets.QPushButton("Reset All")
        self.btn_reset.setStyleSheet("background:#444; color:#fff; font-weight:bold;")
        self.btn_reset.clicked.connect(self.reset_all)
        btn_row.addWidget(self.btn_reset)

        layout.addLayout(btn_row)

        # Second row: status indicators
        status_row = QtWidgets.QHBoxLayout()

        self.ind_guided = QtWidgets.QLabel("GUIDED")
        self.ind_guided.setAlignment(QtCore.Qt.AlignCenter)
        self.ind_guided.setFixedHeight(28)
        self.ind_guided.setStyleSheet(self._indicator_style(False))
        status_row.addWidget(self.ind_guided)

        self.ind_armed = QtWidgets.QLabel("ARMED")
        self.ind_armed.setAlignment(QtCore.Qt.AlignCenter)
        self.ind_armed.setFixedHeight(28)
        self.ind_armed.setStyleSheet(self._indicator_style(False))
        status_row.addWidget(self.ind_armed)

        self.ind_wp = QtWidgets.QLabel("Waypoints")
        self.ind_wp.setAlignment(QtCore.Qt.AlignCenter)
        self.ind_wp.setFixedHeight(28)
        self.ind_wp.setStyleSheet(self._indicator_style(False))
        status_row.addWidget(self.ind_wp)

        layout.addLayout(status_row)

        # Split: upper two previews, bottom waypoint text box
        split_v = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        layout.addWidget(split_v, stretch=1)

        # Upper (horizontal) split for segmentation preview and LiDAR plot
        split_h = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        split_v.addWidget(split_h)

        # Segmentation preview group
        seg_group = QtWidgets.QGroupBox("Segmentation Preview (cropped from display)")
        seg_v = QtWidgets.QVBoxLayout(seg_group)
        self.seg_label = QtWidgets.QLabel("No video")
        self.seg_label.setAlignment(QtCore.Qt.AlignCenter)
        self.seg_label.setMinimumHeight(360)
        self.seg_label.setStyleSheet("background:#111; color:#aaa;")
        seg_v.addWidget(self.seg_label)
        split_h.addWidget(seg_group)

        # LiDAR plot group
        lidar_group = QtWidgets.QGroupBox("LiDAR /scan")
        lidar_v = QtWidgets.QVBoxLayout(lidar_group)
        self.lidar_plot = pg.PlotWidget()
        self.lidar_plot.setBackground('k')
        self.lidar_plot.showGrid(x=True, y=True, alpha=0.3)
        self.lidar_plot.setAspectLocked(True)
        self.lidar_scatter = pg.ScatterPlotItem(size=3, brush=pg.mkBrush(0, 255, 0, 200))
        self.lidar_plot.addItem(self.lidar_scatter)
        lidar_v.addWidget(self.lidar_plot)
        split_h.addWidget(lidar_group)

        split_h.setSizes([650, 650])

        # Waypoint text box
        wp_group = QtWidgets.QGroupBox("Mission Waypoints (lat, lon)")
        wp_v = QtWidgets.QVBoxLayout(wp_group)
        self.wp_text = QtWidgets.QPlainTextEdit()
        self.wp_text.setReadOnly(True)
        self.wp_text.setMaximumBlockCount(10000)
        wp_v.addWidget(self.wp_text)
        split_v.addWidget(wp_group)

        split_v.setSizes([700, 250])

        self.refresh_button_styles()

    def _indicator_style(self, on: bool):
        return "background-color: #1e7e34; color: white; font-weight: bold; border-radius: 6px;" if on \
            else "background-color: #b02a37; color: white; font-weight: bold; border-radius: 6px;"

    def set_btn_state(self, btn, on):
        btn.setChecked(on)
        if on:
            btn.setStyleSheet("background-color: #1e7e34; color: white; font-weight: bold;")
        else:
            btn.setStyleSheet("background-color: #b02a37; color: white; font-weight: bold;")

    def refresh_button_styles(self):
        self.set_btn_state(self.btn_mavlink, self.mavlink_on)
        self.set_btn_state(self.btn_mavros, self.mavros_on)
        self.set_btn_state(self.btn_lidar, self.lidar_on)
        self.set_btn_state(self.btn_segmav, self.segmav_on)

        self.ind_guided.setStyleSheet(self._indicator_style(self.guided_mode))
        self.ind_armed.setStyleSheet(self._indicator_style(self.armed_state))
        self.ind_wp.setStyleSheet(self._indicator_style(self.have_waypoints))

    #########################################
    # Health checks
    #########################################
    def refresh_health(self):
        # MAVLink Router
        self.mavlink_on = udp_ports_ok()

        # MAVROS
        self.mavros_on = rostopic_echo_once(CHECK_MAVROS_TOPIC, timeout=1.2)

        # LiDAR
        self.lidar_on = rostopic_echo_once(CHECK_LIDAR_TOPIC, timeout=1.0)

        # SegMAV process presence
        self.segmav_on = process_running_grep("mavsegmav_merged.py")

        self.refresh_button_styles()

        # Start/stop segmentation capture based on SegMAV window availability
        if OPENCV_AVAILABLE:
            if self.segmav_on and self.seg_cap is None:
                self.start_seg_capture()
            elif not self.segmav_on and self.seg_cap is not None:
                self.stop_seg_capture()

    #########################################
    # ROS init & subscribers
    #########################################
    def init_ros(self):
        try:
            if not rospy.core.is_initialized():
                rospy.init_node("robot_control_gui", anonymous=True, disable_signals=True)

            # LiDAR
            rospy.Subscriber(CHECK_LIDAR_TOPIC, LaserScan, self.lidar_buf.callback, queue_size=1)

            # MAVROS state for GUIDED + ARMED
            rospy.Subscriber("/mavros/state", State, self._state_cb, queue_size=1)

            # Waypoints
            rospy.Subscriber("/mavros/mission/waypoints", WaypointList, self._wp_cb, queue_size=1)

            rospy.loginfo("RobotControlGUI: ROS subscribers ready")
        except Exception as e:
            print("[ROS] init failed:", e)

    def _state_cb(self, msg):
        self.guided_mode = (msg.mode == "GUIDED")
        self.armed_state = bool(getattr(msg, "armed", False))
        QtCore.QMetaObject.invokeMethod(self, "refresh_button_styles", QtCore.Qt.QueuedConnection)

    def _wp_cb(self, msg):
        self.have_waypoints = len(msg.waypoints) > 0
        lines = []
        for i, wp in enumerate(msg.waypoints):
            try:
                lat = getattr(wp, "x_lat", None)
                lon = getattr(wp, "y_long", None)
                if lat is not None and lon is not None:
                    lines.append(f"{i:02d}: {lat:.7f}, {lon:.7f}")
            except Exception:
                continue
        self._waypoint_text = "\n".join(lines) if lines else "(No global lat/lon waypoints or not in global frame)"
        QtCore.QMetaObject.invokeMethod(self, "_update_wp_text_ui", QtCore.Qt.QueuedConnection)

    @QtCore.pyqtSlot()
    def _update_wp_text_ui(self):
        self.wp_text.setPlainText(self._waypoint_text)
        self.refresh_button_styles()

    #########################################
    # LiDAR plot
    #########################################
    def update_lidar_plot(self):
        import math
        with self.lidar_buf.lock:
            angles = self.lidar_buf.angles
            ranges = self.lidar_buf.ranges
        if angles is None or ranges is None:
            return
        xs, ys = [], []
        for a, r in zip(angles, ranges):
            if r and r > 0.02 and r < 80.0:
                xs.append(r * math.cos(a))
                ys.append(r * math.sin(a))
        spots = [{'pos': (x, y)} for x, y in zip(xs, ys)]
        self.lidar_scatter.setData(spots)

    #########################################
    # Segmentation DISPLAY capture (window or cropped)
    #########################################
    def start_seg_capture(self):
        if not OPENCV_AVAILABLE:
            return
        # 1) Try dedicated window capture via xid
        pipeline = x11_pipeline_for_xid(WINDOW_TITLE)
        # 2) Else, try full-screen capture cropped to the window rectangle
        if not pipeline:
            pipeline = x11_pipeline_cropped(WINDOW_TITLE)
        # 3) Final fallback: full-screen capture
        if not pipeline:
            pipeline = x11_pipeline_fullscreen()

        try:
            self.seg_cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if not self.seg_cap.isOpened():
                print("[Video] Failed to open X11 capture pipeline. Check GStreamer plugins and DISPLAY.")
                self.seg_cap.release()
                self.seg_cap = None
                return
            self.seg_timer.start(int(1000/SCREEN_FPS))
        except Exception as e:
            print("[Video] Exception:", e)
            self.seg_cap = None

    def stop_seg_capture(self):
        self.seg_timer.stop()
        if self.seg_cap:
            try:
                self.seg_cap.release()
            except Exception:
                pass
        self.seg_cap = None
        self.seg_label.setText("No video")

    def update_seg_frame(self):
        if not self.seg_cap:
            return
        ret, frame = self.seg_cap.read()
        if not ret:
            return
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        qimg = QtGui.QImage(rgb.data, w, h, ch * w, QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap.fromImage(qimg).scaled(
            self.seg_label.width(), self.seg_label.height(),
            QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation
        )
        self.seg_label.setPixmap(pix)

    #########################################
    # Button toggles + Reset
    #########################################
    def toggle_mavlink(self):
        if self.mavlink_on:
            run_cmd(CMD_MAVLINK_ROUTER_STOP)
            time.sleep(0.5)
        else:
            run_cmd(CMD_MAVLINK_ROUTER_RESTART)
            time.sleep(1.0)
        self.refresh_health()

    def toggle_mavros(self):
        if self.mavros_on:
            run_cmd(CMD_MAVROS_STOP)
            time.sleep(0.5)
        else:
            run_cmd(CMD_MAVROS_START)
            time.sleep(2.0)
        self.refresh_health()

    def toggle_lidar(self):
        if self.lidar_on:
            run_cmd(CMD_LDLIDAR_STOP)
            time.sleep(0.5)
        else:
            run_cmd(CMD_LDLIDAR_START)
            time.sleep(1.5)
        self.refresh_health()

    def toggle_segmav(self):
        if self.segmav_on:
            run_cmd(CMD_SEGMAV_STOP)
            time.sleep(0.8)
        else:
            run_cmd(CMD_SEGMAV_START)
            time.sleep(2.0)
        self.refresh_health()

    def reset_all(self):
        res = run_cmd(CMD_RESET_ALL)
        msg = res.stdout if res and res.stdout else "(no output)"
        QtWidgets.QMessageBox.information(self, "Reset All", f"Script finished.\n\nOutput:\n{msg[:2000]}")
        QtCore.QTimer.singleShot(2000, self.refresh_health)

###############################################
# Main
###############################################
def main():
    app = QtWidgets.QApplication(sys.argv)
    w = RobotControlGUI()
    w.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
