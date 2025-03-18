#! /usr/bin/env python

import rospy
from section_1_services.srv import FindWall, FindWallRequest

rospy.init_node('service_find_wall_client')
rospy.wait_for_service('/find_wall')
find_wall_service_client = rospy.ServiceProxy('/find_wall', FindWall)

find_wall_request = FindWallRequest()

result = find_wall_service_client(find_wall_request)
print(result)