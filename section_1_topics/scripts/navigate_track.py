#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def clbk_laser(msg):
    regions = {
        'right':  min(msg.ranges[0]),
        'front':  min(msg.ranges[359]),
    }
    take_action(regions)
    print('X: %s' % (str(regions)))

def take_action(regions):
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''

    if regions['front'] >= 1:
        state_description = 'case 1 - forward'
        linear_x = 0.2
        angular_z = 0
    elif regions['front'] < 1:
        state_description = 'case 2 - avoid front object'
        linear_x = 0.0
        angular_z = 1.0
    elif regions['right'] < 1:
        state_description = 'case 3 - avoid right object'
        linear_x = 0.0
        angular_z = 1.0
    elif regions['left'] < 1:
        state_description = 'case 4 - avoid left object'
        linear_x = 0.0
        angular_z = -1.0
    else:
        state_description = 'unknown case'

    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)


def main():
    global pub

    rospy.init_node('topics_quiz_node', anonymous=True)

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub = rospy.Subscriber('/kobuki/laser/scan', LaserScan, clbk_laser)

    rospy.spin()

if __name__ == '__main__':
    main()