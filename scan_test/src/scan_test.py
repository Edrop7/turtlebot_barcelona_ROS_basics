#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
    # print(len(msg.ranges))
    print(f"right: {msg.ranges[179]}")
    print(f"front: {msg.ranges[359]}")
    print(f"left: {msg.ranges[539]}")
    print(f"back: {msg.ranges[0]}")
rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()