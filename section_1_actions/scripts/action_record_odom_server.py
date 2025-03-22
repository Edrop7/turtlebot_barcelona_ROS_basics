#! /usr/bin/env python
import rospy
import actionlib
import math
import time

from section_1_actions.msg import OdomRecordAction, OdomRecordResult, OdomRecordFeedback
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry

class RecordOdomServer:

    # create messages that are used to publish feedback/result
    _feedback = OdomRecordFeedback()
    _result   = OdomRecordResult()

    def __init__(self):
        self._as = actionlib.SimpleActionServer("record_odom", OdomRecordAction, self.goal_callback, False)
        self._as.start()
        self.rate = rospy.Rate(1)
        self.ctrl_c = False
        self.cache_x = float(0)
        self.cache_y = float(0)

    def goal_callback(self, goal):
        rospy.loginfo("Received goal, initializing odom recording")
        is_track_complete = False
        self._feedback.current_total = 0

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.loginfo("Established subscriber to gather Odometry")

        #testing script
        for _ in range(10):
            time.sleep(1)
            self._as.publish_feedback(self._feedback)
        rospy.loginfo("Odometry recording returned to /feedback")

        self._as.set_succeeded(self._result)
        rospy.loginfo("Track completed! Displacement of robot returned to /result")

        

    def odom_callback(self, msg):
        displacement_sum = float()
        displacement_x = float()
        displacement_y = float()

        current_position = msg.pose.pose.position
        current_orientation = msg.pose.pose.orientation
        current_point = Point32()
        current_point.x = current_position.x
        current_point.y = current_position.y
        current_point.z = current_orientation.z

        self._result.list_of_odoms.append(current_point)
        # rospy.loginfo(f"logging {current_point}")

        displacement_x = abs(current_point.x - self.cache_x)
        displacement_y = abs(current_point.y - self.cache_y)
        displacement_sum = math.sqrt(displacement_x*displacement_x + displacement_y*displacement_y)

        self.cache_x = current_point.x
        self.cache_y = current_point.y

        self._feedback.current_total += displacement_sum

        time.sleep(1)

    def shutdownhook(self):
        self.ctrl_c = True

if __name__ == '__main__':
    rospy.init_node('record_odom')
    RecordOdomServer()
    rospy.spin()