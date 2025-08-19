#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tiny GUIDED mission streamer for ArduPilot Rover via MAVROS.

What it does:
- Waits for MAVROS connection to FCU.
- Pulls current mission from FCU (uploaded by Mission Planner).
- Filters NAV_WAYPOINT (cmd 16) / NAV_SPLINE_WAYPOINT (cmd 82); skips HOME (seq 0).
- Streams each waypoint as a global GUIDED target until reached, then advances.
- Lets your separate SegMAV process "nudge" to keep centered on the sidewalk.

Run:
    source /opt/ros/noetic/setup.bash
    [ -f ~/catkin_ws/devel/setup.bash ] && source ~/catkin_ws/devel/setup.bash
    export ROS_MASTER_URI=http://localhost:11311
    export ROS_IP=127.0.0.1

    rosrun (or python3) guided_mission.py --auto-guided --auto-arm

Notes:
- Requires MAVROS running and connected on udp://0.0.0.0:14551@
- SegMAV runs separately:
    python3 mavsegmav_merged.py --device=udpin:127.0.0.1:14551 --input=v4l2:///dev/Astra_rgb --preview
"""

import math
import sys
import time
import argparse

import rospy
from std_srvs.srv import Empty
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPoseStamped

from mavros_msgs.srv import WaypointPull, CommandBool, SetMode
from mavros_msgs.msg import WaypointList, State

CMD_NAV_WAYPOINT = 16
CMD_NAV_SPLINE_WAYPOINT = 82


def haversine_m(local_lat, local_lon, tgt_lat, tgt_lon):
    """Distance (meters) between two lat/lon."""
    R = 6371000.0
    phi1 = math.radians(local_lat)
    phi2 = math.radians(tgt_lat)
    dphi = math.radians(tgt_lat - local_lat)
    dl   = math.radians(tgt_lon - local_lon)
    a = math.sin(dphi/2.0)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dl/2.0)**2
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return R * c


class GuidedMissionStreamer:
    def __init__(self, args):
        self.args = args

        rospy.init_node("guided_mission_streamer", anonymous=True)

        # Subscriptions
        self.state = State()
        self.gps = NavSatFix()
        rospy.Subscriber("/mavros/state", State, self._on_state)
        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self._on_gps)

        # Publishers
        self.pub_global = rospy.Publisher(
            "/mavros/setpoint_position/global", GeoPoseStamped, queue_size=10
        )

        # Services
        rospy.loginfo("Waiting for MAVROS services...")
        rospy.wait_for_service("/mavros/mission/pull")
        rospy.wait_for_service("/mavros/cmd/arming")
        rospy.wait_for_service("/mavros/set_mode")
        self.srv_pull = rospy.ServiceProxy("/mavros/mission/pull", WaypointPull)
        self.srv_arm  = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.srv_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        self.wp_list = WaypointList()

        # Waypoint list topic gives the actual list after pull
        rospy.Subscriber("/mavros/mission/waypoints", WaypointList, self._on_wp_list)

    def _on_state(self, msg: State):
        self.state = msg

    def _on_gps(self, msg: NavSatFix):
        self.gps = msg

    def _on_wp_list(self, msg: WaypointList):
        self.wp_list = msg

    def wait_connected(self, timeout=30.0):
        rospy.loginfo("Waiting for MAVROS <-> FCU connection...")
        t0 = rospy.Time.now().to_sec()
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.state.connected:
                rospy.loginfo("Connected to FCU.")
                return True
            if (rospy.Time.now().to_sec() - t0) > timeout:
                rospy.logwarn("Timeout waiting for FCU connection.")
                return False
            rate.sleep()

    def set_mode_guided(self):
        if self.state.mode != "GUIDED":
            rospy.loginfo("Setting mode: GUIDED")
            ok = self.srv_mode(custom_mode="GUIDED").mode_sent
            if not ok:
                rospy.logwarn("Failed to set GUIDED mode (service returned false).")
            else:
                # Wait until mode shows GUIDED
                for _ in range(30):
                    if self.state.mode == "GUIDED":
                        break
                    rospy.sleep(0.1)
                rospy.loginfo(f"Mode now: {self.state.mode}")

    def arm_if_needed(self):
        if not self.state.armed:
            rospy.loginfo("Arming...")
            ok = self.srv_arm(True).success
            if not ok:
                rospy.logwarn("Failed to arm (service returned false).")
            else:
                for _ in range(50):
                    if self.state.armed:
                        break
                    rospy.sleep(0.1)
                rospy.loginfo(f"Armed: {self.state.armed}")

    def pull_mission(self):
        rospy.loginfo("Pulling mission from FCU…")
        try:
            res = self.srv_pull()
            if not res.success:
                rospy.logwarn("WaypointPull: success == False")
            else:
                rospy.loginfo(f"WaypointPull: received={res.wp_received}")
        except rospy.ServiceException as e:
            rospy.logerr(f"WaypointPull failed: {e}")

        # Wait for /mavros/mission/waypoints to deliver the list
        t0 = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            if len(self.wp_list.waypoints) > 0:
                rospy.loginfo(f"Got {len(self.wp_list.waypoints)} waypoints from FCU.")
                return True
            if (rospy.Time.now().to_sec() - t0) > 5.0:
                rospy.logwarn("No waypoints received on /mavros/mission/waypoints.")
                return False
            rospy.sleep(0.1)

    def filtered_waypoints(self):
        """Return mission WPs skipping HOME (seq 0), keeping NAV_WAYPOINT/SPLINE."""
        out = []
        for wp in self.wp_list.waypoints:
            if wp.is_current and wp.seq == 0:
                # Often HOME is seq 0; skip
                continue
            if wp.command in (CMD_NAV_WAYPOINT, CMD_NAV_SPLINE_WAYPOINT):
                out.append(wp)
        return out

    def stream_to_waypoint(self, wp, reach_thresh_m=2.0, hold_sec=0.2, pub_hz=5.0):
        """Continuously publish the given WP as a global setpoint until reached."""
        lat = wp.x_lat
        lon = wp.y_long
        alt = wp.z_alt  # For Rover alt is mostly ignored; still required.

        rospy.loginfo(f"Streaming GUIDED target: seq={wp.seq} "
                      f"lat={lat:.7f} lon={lon:.7f} alt={alt:.2f}")

        rate = rospy.Rate(pub_hz)
        last_log = time.time()

        while not rospy.is_shutdown():
            # Build GeoPoseStamped
            msg = GeoPoseStamped()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            # Frame is geographic (WGS84); header.frame_id can be empty for this plugin
            msg.pose.position.latitude  = lat
            msg.pose.position.longitude = lon
            msg.pose.position.altitude  = alt
            # Orientation unused here
            self.pub_global.publish(msg)

            # Check distance to target; break when within threshold
            if not math.isnan(self.gps.latitude) and not math.isnan(self.gps.longitude):
                d = haversine_m(self.gps.latitude, self.gps.longitude, lat, lon)
                # Log every ~1s
                if (time.time() - last_log) > 1.0:
                    rospy.loginfo(f"  dist={d:.1f} m  mode={self.state.mode}  armed={self.state.armed}")
                    last_log = time.time()
                if d <= reach_thresh_m:
                    rospy.loginfo(f"Reached WP seq={wp.seq} (dist {d:.2f} m) – advancing.")
                    rospy.sleep(hold_sec)
                    return True

            # Abort if mode changed away from GUIDED (user took over)
            if self.state.mode != "GUIDED":
                rospy.logwarn(f"Mode is {self.state.mode}; pausing WP streaming.")
                return False

            rate.sleep()

    def run(self):
        if not self.wait_connected(timeout=60.0):
            rospy.logerr("No FCU connection; exiting.")
            return

        if self.args.auto_guided:
            self.set_mode_guided()
        if self.args.auto_arm:
            self.arm_if_needed()

        if not self.pull_mission():
            rospy.logerr("No mission available from FCU; exiting.")
            return

        wps = self.filtered_waypoints()
        if not wps:
            rospy.logerr("Mission contains no NAV_WAYPOINT/SPLINE items; exiting.")
            return

        rospy.loginfo(f"Executing {len(wps)} waypoints in GUIDED (streamed setpoints).")
        for i, wp in enumerate(wps, 1):
            ok = self.stream_to_waypoint(
                wp,
                reach_thresh_m=self.args.reach_thresh,
                hold_sec=self.args.hold_after_reach,
                pub_hz=self.args.pub_hz
            )
            if not ok:
                rospy.logwarn(f"Stopped at waypoint seq={wp.seq}.")
                break

        rospy.loginfo("GUIDED streaming complete.")


def main():
    ap = argparse.ArgumentParser(description="Tiny GUIDED mission streamer via MAVROS.")
    ap.add_argument("--auto-guided", action="store_true",
                    help="Attempt to set mode GUIDED automatically.")
    ap.add_argument("--auto-arm", action="store_true",
                    help="Attempt to arm automatically (CAUTION).")
    ap.add_argument("--reach-thresh", type=float, default=2.0,
                    help="Distance (m) to consider a waypoint reached. Default 2.0 m")
    ap.add_argument("--hold-after-reach", type=float, default=0.2,
                    help="Seconds to wait after reach before advancing.")
    ap.add_argument("--pub-hz", type=float, default=5.0,
                    help="Publish rate for /mavros/setpoint_position/global.")
    args, _ = ap.parse_known_args()

    try:
        node = GuidedMissionStreamer(args)
        node.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
