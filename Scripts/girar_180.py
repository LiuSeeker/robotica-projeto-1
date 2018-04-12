#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy

import numpy as np

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Imu
import transformations
import math

global ang_inicial
global ang_final

ang_inicial = 0
ang_final = 0

def leu_imu2(dado):
	global ang_inicial
	global ang_final
	quat = dado.orientation
	lista = [quat.x, quat.y, quat.z, quat.w]
	angulos = np.degrees(transformations.euler_from_quaternion(lista))

	ang_inicial = angulos[0]

	if ang_inicial < 180 and ang_inicial >= 0:
		ang_final = ang_inicial - 180
	elif ang_inicial > -180 and ang_inicial < 0:
		ang_final = ang_inicial + 180

	print("A")
	recebe_scan2.unregister()

recebe_scan2 = rospy.Subscriber("/imu", Imu, leu_imu2)
print(ang_inicial, ang_final)




def leu_imu(dado):
	quat = dado.orientation
	lista = [quat.x, quat.y, quat.z, quat.w]
	angulos = np.degrees(transformations.euler_from_quaternion(lista))

	ang_atual = angulos[0]

	velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.7))

	if ang_atual <= ang_final+3 and ang_atual >= ang_final-3:
		print("b")
		velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

	velocidade_saida.publish(velocidade)

	print(ang_inicial, ang_atual, ang_final)




if __name__=="__main__":
	global velocidade_saida

	rospy.init_node("vira")

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 2)
	recebe_scan = rospy.Subscriber("/imu", Imu, leu_imu)

	while not rospy.is_shutdown():
		print("Main loop")
		rospy.sleep(2)



