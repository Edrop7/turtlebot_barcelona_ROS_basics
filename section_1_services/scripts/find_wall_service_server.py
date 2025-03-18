#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from section_1_services.srv import FindWall, FindWallResponse

global lidar

class Lidar:
    def __init__(self):
        self.scan_data = None
        self.min_ray = None
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback_laser)
    
    def callback_laser(self, msg):
        self.scan_data = msg

    def get_min_ray(self):
        if self.scan_data == None:
            rospy.logwarn("No scan data received yet.")
            return None
        
        min_value = min(self.scan_data.ranges)
        min_index = (self.scan_data.ranges).index(min_value)
        
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

def callback_service(request):
    lidar.get_min_ray()
    lidar.read_min_ray()

    response = FindWallResponse()
    response.wallfound = True
    return response


lidar = Lidar()
rospy.init_node('service_find_wall_server')
service = rospy.Service('/find_wall', FindWall, callback_service)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
velocity = Twist()
rospy.loginfo("Service /find_wall Ready")
rate = rospy.Rate(10) # 10hz
rate.sleep()
rospy.spin()