#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def girar(dado):
	global velocidade_saida
	global girar
	rospy.loginfo('GIRANDO')

	if abs(girar) < 3:
		vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.3))
		velocidade_saida.publish(vel)
	else:
		vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, girar * 0.5))
		velocidade_saida.publish(vel)
	if girar != 0:
		if girar < 0: 
			girar += 1
		else: 
			girar -= 1
		return "Girando"



