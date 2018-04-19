#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy

import numpy as np

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Imu
import transformations
import math

imu = []

def Imu(dado):
	global imu, imu_acele, imu_media, bateu, ang
	imu_acele = np.array(dado.linear_acceleration.x).round(decimals=2)
	imu.append(imu_acele)
	
	#Pegando a media da lista recebida
	if len(imu) >= 12: 
		imu = imu[6:]
	
	imu_media = np.mean(imu)
	ang = math.degrees(math.atan2(dado.linear_acceleration.x, dado.linear_acceleration.y))
	
	#Analisando se bateu
	if abs(imu[-1] - imu_media) >= 3.5:
		imu = []
		bateu = True
		return "Bateu"
	return "Procurar"

def Colidiu(ang, dif):
	global bateu

	if ang <= 80:
		vel = Twist(Vector3(-1.5,0,0), Vector3(0,0,-2))
		velocidade_saida.publish(vel)
		rospy.sleep(2)

	elif ang < 100 and ang > 80:
		vel = Twist(Vector3(-1.5,0,0), Vector3(0,0,2))
		velocidade_saida.publish(vel)
		rospy.sleep(2)

	elif ang >= 100:
		vel = Twist(Vector3(-1.5,0,0), Vector3(0,0,2))
		velocidade_saida.publish(vel)
		rospy.sleep(1.5)




if __name__=="__main__":

	rospy.init_node("le_imu")

	recebe_scan = rospy.Subscriber("/imu", Imu, leu_imu)

	while not rospy.is_shutdown():
		print("Main loop")
		rospy.sleep(2)
