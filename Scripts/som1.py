#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy

import numpy as np

import roslib; roslib.load_manifest('sound_play')
import rospy
from turtlebot3_msgs.msg import Sound


	
if __name__=="__main__":
	
	rospy.init_node("som1")
	saida_som = rospy.Publisher("/sound", Sound, queue_size = 2)

	while not rospy.is_shutdown():
		saida_som.publish(1)
		
		#recebe_som = rospy.Subscriber("/sound", Sound, toca_som)

		rospy.sleep(2)


