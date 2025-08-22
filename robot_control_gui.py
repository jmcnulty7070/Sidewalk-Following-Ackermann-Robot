#!/usr/bin/env python3
# -*- coding: utf-8 -*-
\"\"\"
Robot Control GUI for Jetson Nano
Features:
- Toggle buttons to start/stop each program (MAVLink Router, MAVROS, LDLiDAR, SegMAV)
- Buttons turn red (stopped) -> green (running) after health checks pass
- Live Segmentation preview (RTP at udp://127.0.0.1:5400)
- Live LiDAR /scan plot (ROS LaserScan subscriber)
- GUIDED Mode indicator (green when /mavros/state.mode == "GUIDED")
- Waypoints Received indicator (green when any waypoints are received)
- Text box listing latitude/longitude for each mission waypoint
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

# Try to import OpenCV for segmentation preview
try:
    import cv2
    OPENCV_AVAILABLE = True
except Exception as e:
    OPENCV_AVAILABLE = False
    print("[WARN] OpenCV not available for preview:", e)

###############################################
# Configuration
###############################################
# Commands (edit paths/args to match your setup)
CMD_MAVLINK_ROUTER_RESTART = [\"sudo\", \"systemctl\", \"restart\", \"mavlink-router\"]
CMD_MAVLINK_ROUTER_STOP    = [\"sudo\", \"systemctl\", \"stop\", \"mavlink-router\"]

# MAVROS launch (runs in background)
CMD_MAVROS_START = [\"bash\", \"-lc\", \"nohup roslaunch mavros apm.launch fcu_url:=udp://0.0.0.0:14551@ > ~/mavros.log 2>&1 &\"]
CMD_MAVROS_STOP  = [\"bash\", \"-lc\", \"pkill -f 'roslaunch mavros apm.launch' || true\"]

# LDLiDAR node
CMD_LDLIDAR_START = [\"bash\", \"-lc\",
    \"nohup rosrun ldlidar_stl_ros ldlidar_stl_ros_node "
    "_product_name:=LDLiDAR_LD19 "
    "_port_name:=/dev/ldlidar "
    "_port_baudrate:=230400 "
    "_frame_id:=laser "
    "_topic_name:=/scan > ~/ldlidar.log 2>&1 &\"
]
CMD_LDLIDAR_STOP = [\"bash\", \"-lc\", \"pkill -f ldlidar_stl_ros_node || true\"]

# SegMAV script (assumes repo at ~/segmav)
SEGMAV_DIR = str(Path.home() / \"segmav\")
SEGMAV_SCRIPT = \"mavsegmav_merged.py\"
CMD_SEGMAV_START = [\"bash\", \"-lc\",
    f\"cd {SEGMAV_DIR} && nohup python3 {SEGMAV_SCRIPT} "
    "--device=udpin:127.0.0.1:14551 "
    "--input=v4l2:///dev/Astra_rgb "
    "--preview > ~/segmav.log 2>&1 &\"
]
CMD_SEGMAV_STOP = [\"bash\", \"-lc\", \"pkill -f mavsegmav_merged.py || true\"]

# Health checks
CHECK_UDP_PORTS_CMD = [\"bash\", \"-lc\", \"ss -ulpn | egrep '14550|14551|mavlink-routerd' || true\"]
CHECK_MAVROS_TOPIC = \"/mavros/state\"
CHECK_LIDAR_TOPIC = \"/scan\"

# Segmentation preview capture URL (RTP)
SEGMAV_RTP_URL = \"udpsrc port=5400 caps=application/x-rtp ! rtpjitterbuffer latency=100 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink\"
SEGMAV_CAPTURE_BACKEND = cv2.CAP_GSTREAMER if OPENCV_AVAILABLE else 0

###############################################
# Helpers
###############################################
def run_cmd(cmd):
    try:
        return subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=False)
    except Exception:
        return None

def process_running_grep(pattern):
    cmd = [\"bash\", \"-lc\", f\"ps aux | egrep -E '{pattern}' | egrep -v egrep\"]
    res = run_cmd(cmd)
    if not res:
        return False
    return len(res.stdout.strip()) > 0

def udp_ports_ok():
    res = run_cmd(CHECK_UDP_PORTS_CMD)
    return bool(res and res.stdout and \"mavlink-routerd\" in res.stdout)

def rostopic_echo_once(topic, timeout=2.0):
    cmd = [\"bash\", \"-lc\", f\"timeout {timeout} rostopic echo -n1 {topic}\"]
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
        self.setWindowTitle(\"Sidewalk GPS Robot Control\")
        self.setMinimumSize(1200, 900)

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
        self.have_waypoints = False

        # ROS state buffers
        self._waypoint_text = \"\"

        # UI
        self.build_ui()

        # Background health checks
        self.health_timer = QtCore.QTimer()
        self.health_timer.timeout.connect(self.refresh_health)
        self.health_timer.start(2000)

        # Initialize ROS subscribers (non-blocking)
        if ROS_AVAILABLE:
            threading.Thread(target=self.init_ros, daemon=True).start()

    def build_ui(self):
        layout = QtWidgets.QVBoxLayout(self)

        # Top row: toggle buttons
        btn_row = QtWidgets.QHBoxLayout()

        self.btn_mavlink = QtWidgets.QPushButton(\"MAVLink Router\")
        self.btn_mavlink.setCheckable(True)
        self.btn_mavlink.clicked.connect(self.toggle_mavlink)
        btn_row.addWidget(self.btn_mavlink)

        self.btn_mavros = QtWidgets.QPushButton(\"MAVROS\")
        self.btn_mavros.setCheckable(True)
        self.btn_mavros.clicked.connect(self.toggle_mavros)
        btn_row.addWidget(self.btn_mavros)

        self.btn_lidar = QtWidgets.QPushButton(\"LDLiDAR\")
        self.btn_lidar.setCheckable(True)
        self.btn_lidar.clicked.connect(self.toggle_lidar)
        btn_row.addWidget(self.btn_lidar)

        self.btn_segmav = QtWidgets.QPushButton(\"SegMAV\")
        self.btn_segmav.setCheckable(True)
        self.btn_segmav.clicked.connect(self.toggle_segmav)
        btn_row.addWidget(self.btn_segmav)

        layout.addLayout(btn_row)

        # Second row: status indicators
        status_row = QtWidgets.QHBoxLayout()

        self.ind_guided = QtWidgets.QLabel(\"GUIDED Mode\")
        self.ind_guided.setAlignment(QtCore.Qt.AlignCenter)
        self.ind_guided.setFixedHeight(28)
        self.ind_guided.setStyleSheet(self._indicator_style(False))
        status_row.addWidget(self.ind_guided)

        self.ind_wp = QtWidgets.QLabel(\"Waypoints Received\")
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
        seg_group = QtWidgets.QGroupBox(\"Segmentation Preview (RTP @ 5400)\")
        seg_v = QtWidgets.QVBoxLayout(seg_group)
        self.seg_label = QtWidgets.QLabel(\"No video\")
        self.seg_label.setAlignment(QtCore.Qt.AlignCenter)
        self.seg_label.setMinimumHeight(320)
        self.seg_label.setStyleSheet(\"background:#111; color:#aaa;\")
        seg_v.addWidget(self.seg_label)
        split_h.addWidget(seg_group)

        # LiDAR plot group
        lidar_group = QtWidgets.QGroupBox(\"LiDAR /scan\")
        lidar_v = QtWidgets.QVBoxLayout(lidar_group)
        self.lidar_plot = pg.PlotWidget()
        self.lidar_plot.setBackground('k')
        self.lidar_plot.showGrid(x=True, y=True, alpha=0.3)
        self.lidar_plot.setAspectLocked(True)
        self.lidar_scatter = pg.ScatterPlotItem(size=3, brush=pg.mkBrush(0, 255, 0, 200))
        self.lidar_plot.addItem(self.lidar_scatter)
        lidar_v.addWidget(self.lidar_plot)
        split_h.addWidget(lidar_group)

        split_h.setSizes([600, 600])

        # Waypoint text box
        wp_group = QtWidgets.QGroupBox(\"Mission Waypoints (lat, lon)\")
        wp_v = QtWidgets.QVBoxLayout(wp_group)
        self.wp_text = QtWidgets.QPlainTextEdit()
        self.wp_text.setReadOnly(True)
        self.wp_text.setMaximumBlockCount(10000)
        wp_v.addWidget(self.wp_text)
        split_v.addWidget(wp_group)

        split_v.setSizes([600, 300])

        self.refresh_button_styles()

    def _indicator_style(self, on: bool):
        return \"background-color: #1e7e34; color: white; font-weight: bold; border-radius: 6px;\" if on \
            else \"background-color: #b02a37; color: white; font-weight: bold; border-radius: 6px;\"

    def set_btn_state(self, btn, on):
        btn.setChecked(on)
        if on:
            btn.setStyleSheet(\"background-color: #1e7e34; color: white; font-weight: bold;\")
        else:
            btn.setStyleSheet(\"background-color: #b02a37; color: white; font-weight: bold;\")

    def refresh_button_styles(self):
        self.set_btn_state(self.btn_mavlink, self.mavlink_on)
        self.set_btn_state(self.btn_mavros, self.mavros_on)
        self.set_btn_state(self.btn_lidar, self.lidar_on)
        self.set_btn_state(self.btn_segmav, self.segmav_on)

        self.ind_guided.setStyleSheet(self._indicator_style(self.guided_mode))
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
        self.segmav_on = process_running_grep(\"mavsegmav_merged.py\")

        self.refresh_button_styles()

        # Start/stop segmentation capture based on segmav_on
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
                rospy.init_node(\"robot_control_gui\", anonymous=True, disable_signals=True)

            # LiDAR
            rospy.Subscriber(CHECK_LIDAR_TOPIC, LaserScan, self.lidar_buf.callback, queue_size=1)

            # Guided mode (MAVROS state)
            rospy.Subscriber(\"/mavros/state\", State, self._state_cb, queue_size=1)

            # Waypoints
            rospy.Subscriber(\"/mavros/mission/waypoints\", WaypointList, self._wp_cb, queue_size=1)

            rospy.loginfo(\"RobotControlGUI: ROS subscribers ready\")
        except Exception as e:
            print(\"[ROS] init failed:\", e)

    def _state_cb(self, msg):
        self.guided_mode = (msg.mode == \"GUIDED\")
        # Update UI on main thread
        QtCore.QMetaObject.invokeMethod(self, \"refresh_button_styles\", QtCore.Qt.QueuedConnection)

    def _wp_cb(self, msg):
        # Consider \"received\" if any waypoints exist
        self.have_waypoints = len(msg.waypoints) > 0
        # Build lat/lon list if fields exist
        lines = []
        for i, wp in enumerate(msg.waypoints):
            try:
                lat = getattr(wp, \"x_lat\", None)
                lon = getattr(wp, \"y_long\", None)
                if lat is not None and lon is not None:
                    lines.append(f\"{i:02d}: {lat:.7f}, {lon:.7f}\")
            except Exception:
                continue
        self._waypoint_text = \"\\n\".join(lines) if lines else \"(No global lat/lon waypoints or not in global frame)\"
        QtCore.QMetaObject.invokeMethod(self, \"_update_wp_text_ui\", QtCore.Qt.QueuedConnection)

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
    # Segmentation RTP capture
    #########################################
    def start_seg_capture(self):
        if not OPENCV_AVAILABLE:
            return
        try:
            self.seg_cap = cv2.VideoCapture(SEGMAV_RTP_URL, SEGMAV_CAPTURE_BACKEND)
            if not self.seg_cap.isOpened():
                print(\"[Video] Failed to open RTP stream. Check SegMAV output / GStreamer plugins.\")
                self.seg_cap.release()
                self.seg_cap = None
                return
            self.seg_timer.start(33)  # ~30 FPS
        except Exception as e:
            print(\"[Video] Exception:\", e)
            self.seg_cap = None

    def stop_seg_capture(self):
        self.seg_timer.stop()
        if self.seg_cap:
            try:
                self.seg_cap.release()
            except Exception:
                pass
        self.seg_cap = None
        self.seg_label.setText(\"No video\")

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
    # Button toggles
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

###############################################
# Main
###############################################
def main():
    app = QtWidgets.QApplication(sys.argv)
    w = RobotControlGUI()
    w.show()
    sys.exit(app.exec_())

if __name__ == \"__main__\":
    main()
