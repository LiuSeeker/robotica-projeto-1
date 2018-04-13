#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def frange(start, stop, step=1.0):
    i = start
    while i < stop:
        yield i
        i += step


def girar():
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	twist = Twist()

	while not rospy.is_shutdown():
		twist.angular.z = 0
		for i in frange(0.8,0,-0.1):
			twist.angular.z = i 
			rospy.loginfo('Girando')
		rospy.loginfo('Girou')

		pub.publish(twist)


def main():
    rospy.init_node('girar_angulo')
    try:
        girar()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()