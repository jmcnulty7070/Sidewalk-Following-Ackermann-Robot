#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from mavros_msgs.msg import DistanceSensor

def callback(data):
    # Prepare a DistanceSensor message
    ds = DistanceSensor()
    ds.header.stamp = rospy.Time.now()
    ds.header.frame_id = "lidar_front"
    ds.min_distance = 0.1        # Minimum distance LiDAR can measure
    ds.max_distance = 12.0       # Maximum distance LiDAR can measure
    ds.current_distance = min(data.ranges)
    ds.type = DistanceSensor.LASER
    ds.id = 0
    ds.orientation = DistanceSensor.ORIENTATION_FORWARD

    pub.publish(ds)

if __name__ == '__main__':
    rospy.init_node('lidar_to_mavlink_bridge')
    pub = rospy.Publisher('/mavros/distance_sensor/custom', DistanceSensor, queue_size=10)
    rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()
