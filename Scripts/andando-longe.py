import rospy
import smach
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

def converte(valor):
	return valor*44.4/0.501

def longe(data):
	global velocidade_saida
	distancias = np.array(data.ranges)
	rospy.loginfo('ANDANDO LONGE')
	
	for i in range(len(distancias)):
		if i <= 30 or i>= 330:
			print(converte(distancias[i]))
			
			if converte(distancias[i]) >= 120:
				V = 0.4
				vel = Twist(Vector3(V, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
			elif converte(distancias[i]) >= 80:
				V = 0.05*(converte(distancias[i])-10)
				vel = Twist(Vector3(V, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
			elif converte(distancias[i]) >= 20:
				V = 0.01*(converte(distancias[i])-10)
				vel = Twist(Vector3(V, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)


if __name__=="__main__":
	global velocidade_saida

	rospy.init_node("andando")

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 2 )
	recebe_scan = rospy.Subscriber("/scan", LaserScan, longe)



	while not rospy.is_shutdown():
		print("Oeee")
		rospy.sleep(0.5)
