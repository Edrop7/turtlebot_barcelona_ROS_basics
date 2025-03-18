#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from section_1_services.srv import FindWall, FindWallResponse
from lidar_module import Lidar

def rotate_robot_to_wall():
    rospy.loginfo("Initiating turn to closest wall")

    lidar.get_min_ray()

    while not (lidar.min_ray['index'] < 363 and lidar.min_ray['index'] > 355):
        lidar.get_min_ray()
        # lidar.read_min_ray()
        msg = Twist()
        msg.angular.z = 0.2
        pub.publish(msg)
    
    msg.angular.z = 0.0
    pub.publish(msg)

    rospy.loginfo("Closest wall now in front of robot")

    return None

def callback_service(request):
    lidar.get_min_ray()
    lidar.read_min_ray()

    rotate_robot_to_wall()

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