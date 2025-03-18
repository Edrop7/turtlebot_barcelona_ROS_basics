#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

class Lidar:
    def __init__(self):
        self.scan_data = None
        self.min_ray = None
        self.cardinal_rays = None
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback_laser)
    
    def callback_laser(self, msg):
        self.scan_data = msg

    def get_min_ray(self):
        if self.scan_data == None:
            rospy.logwarn("No scan data received yet.")
            return None
        
        ray_tuple = tuple(self.scan_data.ranges)

        min_value = min(ray_tuple)
        min_index = (ray_tuple).index(min_value)
        
        self.min_ray = {
            'value': min_value,
            'index': min_index
        }
        
        return None
    
    def read_min_ray(self):
        if self.min_ray == None:
            rospy.logwarn("min ray not received yet.")
            return None

        rospy.loginfo(f"The min value {self.min_ray['value']} is at index {self.min_ray['index']}")
        
        return None

    def get_cardinal_rays(self):
        if self.scan_data == None:
            rospy.logwarn("No scan data received yet.")
            return None

        self.cardinal_rays = {
            'right': self.scan_data.ranges[179],
            'front': self.scan_data.ranges[359],
            'back': self.scan_data.ranges[0],
            'left': self.scan_data.ranges[539]
        }
        
        return None

    def read_cardinal_rays(self):
        if self.cardinal_rays == None:
            rospy.logwarn("cardinal rays not received yet.")
            return None

        rospy.loginfo(f"{str(self.cardinal_rays)}")
        
        return None