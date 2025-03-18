#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from section_1_services.srv import FindWall, FindWallResponse
from lidar_module import Lidar

global rate

def rotate_robot_to_wall(direction):
    rospy.loginfo(f"Initiating turn to {direction} direction")

    lidar.get_min_ray()
    msg = Twist()
    angle = 359
    tol = 10 # angle tolerance

    if direction == 'front':
        angle = 359
    elif direction == 'terminal':
        angle = 179

    msg.angular.z = 0.2 # this needs to be outside loop to prevent continuous running

    #maximal direciton tolerance
    while not (lidar.min_ray['index'] < (angle+(tol)) and lidar.min_ray['index'] > (angle-(tol))):
        lidar.get_min_ray()
        pub.publish(msg)
        rate.sleep() # must be in loop to prevent continuous running on hardware
        # lidar.read_min_ray()
    
    for i in range(5):
        msg.angular.z = 0.0
        pub.publish(msg)
        rate.sleep()

    rospy.loginfo(f"Completed turn to {direction} wall")

    return None

def move_robot_to_wall():
    rospy.loginfo("Initiating approach to front wall")

    lidar.get_cardinal_rays()
    msg = Twist()

    msg.linear.x = 0.02 # this needs to be outside loop to prevent continuous running

    while lidar.cardinal_rays['front'] > 0.3:
        lidar.get_cardinal_rays()
        pub.publish(msg)
        rate.sleep() # must be in loop to prevent continuous running on hardware
        # lidar.read_cardinal_rays()

    for i in range(5):
        msg.linear.x = 0.0
        pub.publish(msg)
        rate.sleep()

    rospy.loginfo("Front wall within 30 cm of robot")
        
    return None

def callback_service(request):
    lidar.get_min_ray()
    lidar.read_min_ray()

    rotate_robot_to_wall('front')
    move_robot_to_wall()
    rotate_robot_to_wall('terminal')

    response = FindWallResponse()
    response.wallfound = True
    return response

lidar = Lidar()

rospy.init_node('service_find_wall_server')

rate = rospy.Rate(10) # 10hz

service = rospy.Service('/find_wall', FindWall, callback_service)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

velocity = Twist()

rospy.loginfo("Service /find_wall Ready")

rate.sleep()
rospy.spin()