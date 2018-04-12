import rospy


inicio = rospy.get_time()
tempo = 0.5e9
agora = rospy.get_time()
passou = inicio - agora


def girar(dados,angle):
	quat = dado.orientation
	lista = [quat.x, quat.y, quat.z, quat.w]
	angulos = np.degrees(transformations.euler_from_quaternion(lista))
	ang_atual = angulos[0]
	print(ang_atual)




if __name__=="__main__":

rospy.init_node("gira")


while not rospy.is_shutdown():
	print("Main loop")
	rospy.sleep(2)


