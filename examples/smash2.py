import roslib
import rospy
import smach
import smach_ros
import rospy
import numpy as np
from numpy import linalg
from tf import TransformerROS
import tf2_ros
import math
import time
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage, LaserScan, Imu
from std_msgs.msg import Header
import cv2
from cv_bridge import CvBridge, CvBridgeError

################################################################################

bridge = CvBridge()

cv_image = None
media = []
centro = []
maior_contorno = []
atraso = 1.5
andar = 0
girar = 0
velLimit = 3

mFrente = []
mDireita = []
mEsquerda = []
mTras = []
MDirenta = 3
MFrente = 3
MEsquerda = 3
MTras = 3
MListTras = []
MListFrente = []
MListDireita = []
MListEsquerda = []

check_delay = False

mimuX = []
flag_bati = 0

################################################################################

def identifica_cor(frame):
	global media, centro, maior_contorno
	frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	cor_menor = np.array([50, 50, 50])
	cor_maior = np.array([70, 255, 255])
	segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)


	segmentado_cor = cv2.morphologyEx(segmentado_cor,cv2.MORPH_CLOSE,np.ones((7, 7)))

	img_out, contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	maior_contorno = None
	maior_contorno_area = 0


	for cnt in contornos:
		area = cv2.contourArea(cnt)
		if area > maior_contorno_area:
			maior_contorno = cnt
			maior_contorno_area = area

	if not maior_contorno is None :
		cv2.drawContours(frame, [maior_contorno], -1, [0, 0, 255], 5)
		maior_contorno = np.reshape(maior_contorno, (maior_contorno.shape[0], 2))
		media = maior_contorno.mean(axis=0)
		media = media.astype(np.int32)
		cv2.circle(frame, tuple(media), 5, [0, 255, 0])
	else:
		media = (0, 0)
	print("o")
	cv2.imshow('video', frame)
	cv2.imshow('seg', segmentado_cor)
	cv2.waitKey(1)

	centro = (frame.shape[0]//2, frame.shape[1]//2)

	return media, centro

def roda_todo_frame_cor(imagem):
	print("frame")
	global cv_image
	global media
	global centro

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.secs
	print("roda")
	if delay > atraso and check_delay==True:
		return
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		media, centro = identifica_cor(cv_image)
		depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)

################################################################################

def scaneou(dado):
	global mFrente, mDireita, mEsquerda, mTras  # media
	global MFrente, MDireita, MEsquerda, MTras  #
	global MListTras, MListFrente, MListDireita, MListEsquerda  # Historico
	# :14*3
	Frente = np.array(dado.ranges)[:14*3]

	# 14*5:14:9
	Esquerda = np.array(dado.ranges)[14*5:14*9]

	# 14*11:14*15
	Tras = np.array(dado.ranges)[14*11:14*15]

	# 14*18:14*21
	Direita = np.array(dado.ranges)[14*18:14*21]

	#print("Intensities")
	#print(np.array(dado.intensities).round(decimals=2))
	tmp = np.mean(Frente)
	if tmp > 3: tmp = 3
	mFrente.append(tmp)

	tmp = np.mean(Direita)
	if tmp > 3: tmp = 3
	mDireita.append(tmp)

	tmp = np.mean(Esquerda)
	if tmp > 3: tmp = 3
	mEsquerda.append(tmp)

	tmp = np.mean(Tras)
	if tmp > 3: tmp = 3
	mTras.append(tmp)

	if len(mFrente) > 5:
		mFrente = mFrente[3:]
		mDireita = mDireita[3:]
		mEsquerda = mEsquerda[3:]
		mTras = mTras[3:]

	MFrente = np.mean(mFrente)
	MTras = np.mean(mTras)
	MDireita = np.mean(mDireita)
	MEsquerda = np.mean(mEsquerda)

	MListFrente.append(MFrente)
	MListDireita.append(MDireita)
	MListEsquerda.append(MEsquerda)
	MListTras.append(MTras)
	if len(MListFrente) > 200:
		MListFrente = MListFrente[50:]
		MListDireita = MListDireita[50:]
		MListEsquerda = MListEsquerda[50:]
		MListTras = MListTras[50:]

def ImuOut(dado):
	global imuX, mimuX, flag_bati
	IimuX = np.array(dado.linear_acceleration.x).round(decimals=4)
	mimuX.append(IimuX)
	if len(mimuX)>10:mimuX = mimuX[5:]
	imuX = np.mean(mimuX)
	if IimuX <= 0.7:  # VAL DETECTA BATI
		mimuX = []
		flag_bati = 1

################################################################################

class Aprender(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['Survive'])

	def execute(self, userdata):
		rospy.loginfo('Executing state APRENDENDO')
		#comando para aprender a ler a cor
		return 'Survive'

class Andando(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['Andando','Analisando','Bati'])

	def execute(self, userdata):
		global velocidade_saida
		global andar, velLimit, ObjDis
		rospy.loginfo('Executing state ANDANDO')
		#comando para andar
		X = velLimit * ObjDis
		X = X * (andar/abs(andar))
		vel = Twist(Vector3(X, 0, 0), Vector3(0, 0, 0))
		velocidade_saida.publish(vel)
		if andar != 0:
			if andar < 0: andar+=1
			else: andar-=1

			return "Andando"
		else:
			if flag_bati == 0:
				return 'Analisando'
			else:
				return 'Bati'

class Girando(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['Girando','Analisando','Bati'])

	def execute(self, userdata):
		global velocidade_saida
		global girar
		rospy.loginfo('Executing state GIRANDO')
		#comando para girar
		if abs(girar) < 3:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.3))
			velocidade_saida.publish(vel)
		else:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, girar * 0.5))
			velocidade_saida.publish(vel)
		if girar != 0:
			if girar < 0: girar += 1
			else: girar -= 1
			return "Girando"
		else:
			if flag_bati == 0:
				return 'Analisando'
			else:
				return 'Bati'

class Analisando(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['Andando','Girando','Survive'])
		self.counter = 0
		self.ObjGiro = "E"

	def execute(self, userdata):
		global andar, girar, FollowCount, ObjDis
		global media, centro, maior_contorno
		rospy.loginfo('Executing state ANALISANDO')
		if self.counter < 3:
			self.counter += 1
			if len(media) != 0 and len(centro) != 0: # achei objeto
				dif_x = media[0]-centro[0]
				dif_y = media[1]-centro[1]
				if math.fabs(dif_x)<30: # Estou Alinhado ao Objeto
					andar += 3
					ObjDis = 100 - len(maior_contorno)  # Parametro Proporcional
					return 'Andando'

				else: #Objeto Esquerda
					if dif_x > 0:
						girar -= 1
						self.ObjGiro = "D"
						print("DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD")

					elif dif_x < 0 and dif_x != -240: #Objeto direita
						self.ObjGiro = "E"
						print("EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE")
						girar += 1
					return 'Girando'
			else:
				if self.ObjGiro == "D":
					girar += 5
				elif self.ObjGiro == "E":
					girar -= 5
				return 'Girando'

		else:
			self.counter = 0
			return "Survive"

################################################################################

class Bati(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['Survive','Bati','Andando','Girando'])
		self.IDO = 0

	def execute(self, userdata):
		global flag_bati
		rospy.loginfo('Executing state BATIIIIIIIIIIIIIIIIIIIIIIIIIIIIII')
		if flag_bati == 1:
			andar = -3
			flag_bati = 0
			return "Andando"
		return "Survive"

class Evitar(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['Survive'])

	def execute(self, userdata):
		global velLimit
		global mFrente
		rospy.loginfo('Executing state EVITAR')

		velLimit = MFrente * 1.1
		print(MFrente)
		return "Survive"

class Survive(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['Bati', 'Evitar', 'Analisando'])
		self.counter = 0

	def execute(self, userdata):
		global flag_bati
		rospy.loginfo('Executing state SURVIVE')
		if self.counter < 1:
			self.counter += 1
			if flag_bati != 0: # Bati
				return "Bati"
			else:  # Evitar
				return 'Evitar'
		else:
			self.counter = 0
			return "Analisando"



def maquina():
	global velocidade_saida
	rospy.init_node('smach_example_state_machine')
	recebedor_cor = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, roda_todo_frame_cor, queue_size = 10, buff_size = 2**24)
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
	recebe_imu = rospy.Subscriber("/imu", Imu, ImuOut)
	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['terminei'])

	# Open the container
	with sm:

		# Add states to the container
		smach.StateMachine.add('APRENDENDO', Aprender(),
								transitions = {'Survive':'SURVIVE'})


		smach.StateMachine.add('SURVIVE', Survive(),
								transitions = {'Analisando':'ANALISANDO',
												'Bati':'BATI',
												'Evitar':"EVITAR"})
		smach.StateMachine.add('EVITAR', Evitar(),
								transitions = {'Survive':'SURVIVE'})
		smach.StateMachine.add('BATI', Bati(),
								transitions = {'Survive':'SURVIVE',
												'Bati': 'BATI',
												'Andando': "ANDANDO",
												'Girando': "GIRANDO"})


		smach.StateMachine.add('ANALISANDO', Analisando(),
								transitions = {'Survive':'SURVIVE',
											'Andando':'ANDANDO',
											'Girando':'GIRANDO'})
		smach.StateMachine.add('ANDANDO', Andando(),
								transitions = {'Analisando':'ANALISANDO',
												'Bati':'BATI',
											 	'Andando':'ANDANDO'})
		smach.StateMachine.add('GIRANDO', Girando(),
								transitions = {'Analisando':'ANALISANDO',
											 'Girando':'GIRANDO',
											 'Bati':"BATI"})



	# Execute SMACH plan
	outcome = sm.execute()

if __name__=="__main__":
	maquina()
