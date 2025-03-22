#! /usr/bin/env python
import rospy
import actionlib
from section_1_actions.msg import OdomRecordAction, OdomRecordGoal

def feedback_callback(feedback):
    print("goal sent")

# initializes the action client node
rospy.init_node('record_odom_client')

client = actionlib.SimpleActionClient('/record_odom', OdomRecordAction)
# waits until the action server is up and running
client.wait_for_server()

# creates a goal to send to the action server
goal = OdomRecordGoal()

# sends the goal to the action server, specifying which feedback function
# to call when feedback received
client.send_goal(goal, feedback_cb=feedback_callback)