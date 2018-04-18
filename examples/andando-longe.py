import rospy
import smach


def longe(data):
	global velocidade_saida
	global andando, velLimite, objDist
	rospy.loginfo('ANDANDO')
	
	V = velLimite * objDist
	V = V * (andando/abs(andando))
	vel = Twist(Vector3(V, 0, 0), Vector3(0, 0, 0))
	velocidade_saida.publish(vel)
	if andar != 0:
		if andar < 0: 
			andar+=1
		else: 
			andar-=1
		return "Andando"


if __name__=="__main__":
	global velocidade_saida

	rospy.init_node("andando")

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 2 )
	recebe_scan = rospy.Subscriber("/scan", LaserScan, longe)



	while not rospy.is_shutdown():
		print("Oeee")
		rospy.sleep(0.5)
