#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def callback_laser(msg):
    laser_reading = {
        # ray angles for right and front given min_angle = -pi radian, 720 indexes, right = 0 radian convention
        'right': msg.ranges[179],
        'front': msg.ranges[359]
    }
    take_action(laser_reading)
    print('X: %s' % (str(laser_reading)))

def take_action(laser_reading):
    msg = Twist()
    linear_x = 0
    angular_z = 0
    state_description = ''

    # added a margin of 0.05 m vs spec from notebook to encompass robot width
    if laser_reading['front'] < 0.55:
        state_description = 'case 1 - corner reached, turn with radius 0.25'
        linear_x = 0.1
        angular_z = 0.75
    elif laser_reading['front'] > 0.55:
        if laser_reading['right'] < 0.25:
            state_description = 'case 2 - too close to right wall'
            linear_x = 0.1
            angular_z = 0.2
        elif laser_reading['right'] > 0.25:
            state_description = 'case 3 - too far from right wall'
            linear_x = 0.1
            angular_z = -0.2
        else:
            state_description = 'case 4 - move straight'
            linear_x = 0.1
            angular_z = 0.0
    else:
        state_description = 'unknown case'

    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)


def main():
    global pub

    rospy.init_node('topics_quiz_node', anonymous=True)

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/scan', LaserScan, callback_laser)
    rate = rospy.Rate(10) # 10hz
    rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    main()