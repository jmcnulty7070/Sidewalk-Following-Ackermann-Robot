#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from mavros_msgs.msg import DistanceSensor

def callback(data):
    ds = DistanceSensor()
    ds.header.stamp = rospy.Time.now()
    ds.min_distance = 0.1
    ds.max_distance = 12.0
    ds.current_distance = min(data.ranges)
    ds.type = 1
    ds.id = 0
    ds.orientation = 0
    pub.publish(ds)

rospy.init_node('lidar_bridge')
pub = rospy.Publisher('/mavros/distance_sensor/custom', DistanceSensor, queue_size=10)
rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()
