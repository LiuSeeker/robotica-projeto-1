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
	global imu, imu_acele, imu_media, bateu
	imu_acele = np.array(dado.linear_acceleration.x).round(decimals=2)
	imu.append(imu_acele)
	#Pegando a media da lista recebida
	if len(imu) >= 12: 
		imu = imu[6:]
	imu_media = np.mean(imu)

	#Analisando se bateu
	if imu_media > -5 and imuX <= -1: 
		imu = []
		bateu = True
		return "Bateu"
	return "Procurar"

if __name__=="__main__":

	rospy.init_node("le_imu")

	recebe_scan = rospy.Subscriber("/imu", Imu, leu_imu)

	while not rospy.is_shutdown():
		print("Main loop")
		rospy.sleep(2)
