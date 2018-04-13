#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def frange(start, stop, step=1.0):
    i = []
    while start > stop:
    	start += step
    	i.append(start)
    return i

def girar():
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	twist = Twist()
	twist.angular.z = 0
	twist.linear.z = 0
	#Esse for determina o quanto ele vai virar
	for i in frange(0.8,0,-1e-5):
		twist.linear.z = i
		twist.angular.z = i 
		rospy.loginfo('Girando')
	rospy.loginfo('Girou')

	pub.publish(twist)


def main():
    rospy.init_node('girar_angulo')
    try:
        obstacle = girar()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
