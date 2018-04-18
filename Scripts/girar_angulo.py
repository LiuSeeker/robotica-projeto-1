#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Vector3

girando = False
tempo_giro = 4*3.14*1e9
tempo_inicio = -1
tempo_fim = -1
velocidade_saida = Twist(Vector3(0,0,0), Vector3(0,0,0))

def girar(dado):
	girando = False

	if girando:
		tempo_fim = rospy.get_rostime()
		if tempo_fim - tempo_inicio > tempo_giro:
				vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.0))
				velocidade_saida.publish(vel)
				#return
		else:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.5))
			velocidade_saida.publish(vel)
			#return
	else:
		girando = True
		tempo_inicio = rospy.get_rostime()
		vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.5))
		velocidade_saida.publish(vel)
		#return	


if __name__ == "__main__":
    rospy.init_node("girar_angulo")
    velocidade_saida = rospy.Publisher("cmd_vel", Twist, queue_size=2)
    #func = rospy.Subscriber("/imu",Imu, girar)

    while not rospy.is_shutdown():
		print("Main loop")
		rospy.sleep(0.01)
		girar("dummy")


