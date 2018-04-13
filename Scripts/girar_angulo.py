#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def girar():
	error = 0.05
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	twist = Twist()

	while not rospy.is_shutdown():
		msg = rospy.wait_for_message("/scan", LaserScan)
		scan_filter = []
		for i in range(360):
			if i <= 15 or i > 335:
				if msg.range[i] >= error:
					self.scan_filter.append(msg.range[i])

		if min(scan_filter) < 0.3:
			rospy.loginfo('Ta mto perto')
			twist.linear.x = 0.0
			twist.angular.z = 0
			for i in range(0.8,0,-0.001):
				twist.angular.z += i 
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