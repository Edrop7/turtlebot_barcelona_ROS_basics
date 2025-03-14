#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def callback_odom(msg):
    # used find the current index given an absolute reference
    index_offset = abs(int(msg.pose.pose.orientation.z / 0.00138888888888888888888888888889))
    print(index_offset)
    take_action()

def take_action():
    msg = Twist()
    msg.angular.z = 0.8726646259971647884618453842443
    pub.publish(msg)

def main():
    global pub

    rospy.init_node('find_spin_angle', anonymous=True)

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub2 = rospy.Subscriber('/odom', Odometry, callback_odom)
    rospy.spin()


if __name__ == '__main__':
    main()