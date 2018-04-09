#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy

import numpy as np

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan


def converte(valor):
	return valor*44.4/0.501


def scaneou(dado):
	#print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
	#print("Leituras:")
	distancias = np.array(dado.ranges)
	#print(distancias)
	desviando = False
	menor_frente_esquerda = 4
	menor_frente_direita = 4

	velocidade = Twist(Vector3(0.2, 0, 0), Vector3(0, 0, 0)) #Comentar isso dps


	for i in range(len(distancias)):
		if i <= 40:
			
			if converte(distancias[i]) < 50 and converte(distancias[i]) >= 40:
				velocidade = Twist(Vector3(0.2, 0, 0), Vector3(0, 0, -0.4))
				desviando = True
				print("frente e")
			if converte(distancias[i]) < 40:
				velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.5))
				desviando = True
				print("mt frente e")
				if menor_frente_esquerda > converte(distancias[i]):
					menor_frente_esquerda = converte(distancias[i])
			
		if i >= 320:
			if converte(distancias[i]) < 50 and converte(distancias[i]) >= 40:
				velocidade = Twist(Vector3(0.2, 0, 0), Vector3(0, 0, 0.4))
				desviando = True
				print("frente d")
			if converte(distancias[i]) < 40:
				velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.5))
				desviando = True
				print("mt frente d")
				if menor_frente_direita > converte(distancias[i]):
					menor_frente_direita = converte(distancias[i])
			
		if i <= 80 and i > 40:
			if converte(distancias[i]) < 20:
				velocidade = Twist(Vector3(0.2, 0, 0), Vector3(0, 0, -0.2))
				desviando = True
				print("mt esq")
		if i < 320 and i >= 290:
			if converte(distancias[i]) < 20:
				velocidade = Twist(Vector3(0.2, 0, 0), Vector3(0, 0, 0.2))
				desviando = True
				print("mt dir")


	velocidade_saida.publish(velocidade)
	#print(np.array(dado.ranges).round(decimals=2))
	#print("Intensities")
	#print(np.array(dado.intensities).round(decimals=2))
	if desviando:
		return 'desviando'
	else:
		return 'desviado'

	


if __name__=="__main__":
	global velocidade_saida
	global condicao

	rospy.init_node("le_scan")

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 2 )
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)



	while not rospy.is_shutdown():
		print("Oeee")
		rospy.sleep(1.)


