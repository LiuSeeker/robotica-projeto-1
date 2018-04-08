#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy

import numpy as np

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan


def scaneou(dado):
	print("- Novo Scan -")
	#print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
	#print("Leituras:")
	distancias = np.array(dado.ranges)
	#print(distancias)
	frente_esquerda = []
	frente_direita = []
	esquerda = []
	tras = []
	direita = []

	for i in range(len(distancias)):
		if i <= 30:
			if distancias[i] < 0.6:
				frente_esquerda.append(distancias[i])
		if i >= 330:
			if distancias[i] < 0.6:
				frente_direita.append(distancias[i])
		if i <= 80 and i > 30:
			if distancias[i] < 0.6:
				esquerda.append(distancias[i])
		if i < 290 and i > 80:
			if distancias[i] < 0.4:
				tras.append(distancias[i])
		if i < 330 and i >= 290:
			if distancias[i] < 0.6:
				direita.append(distancias[i])


	def dist_crit(lado):
		for f in lado:
			if f < 0.2:
				return True

	def dist_alert(lado):
		for f in lado:
			if f < 0.4:
				return True


	if dist_crit(frente_esquerda):
		print("Frente esquerda mt perto")
	elif dist_alert(frente_esquerda):
		print("Frente esquerda perto")
	if dist_crit(frente_direita):
		print("Frente direita mt perto")
	elif dist_alert(frente_direita):
		print("Frente direita perto")
	if dist_crit(esquerda):
		print("Esquerda mt perto")
	elif dist_alert(esquerda):
		print("Esquerda perto")
	if dist_crit(direita):
		print("Direita mt perto")
	elif dist_alert(direita):
		print("Direita perto")
	
	#print(np.array(dado.ranges).round(decimals=2))
	#print("Intensities")
	#print(np.array(dado.intensities).round(decimals=2))

	


if __name__=="__main__":

	rospy.init_node("le_scan")

	#velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)



	while not rospy.is_shutdown():
		#print("Oeee")
		#velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 1))
		#velocidade_saida.publish(velocidade)
		rospy.sleep(1.)


