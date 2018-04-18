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
from matplotlib import pyplot as plt

################################################################################

bridge = CvBridge()

# Initiate SIFT detector
sift = cv2.xfeatures2d.SIFT_create()
img1 = cv2.imread('powerpuff-girls.png',0)	  # Imagem a procurar
img1 = cv2.resize(img1,(0,0),fx = 0.18, fy = 0.18)
kp1, des1 = sift.detectAndCompute(img1,None)

FLANN_INDEX_KDTREE = 0
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks = 5)

# Configura o algoritmo de casamento de features
flann = cv2.FlannBasedMatcher(index_params, search_params)


MIN_MATCH_COUNT_FEATURES = 15

POWERPUFF_flag = 0

cv_image = None
frame_hsv = None
media = []
centro = []
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

cor_maior = np.array([70, 255, 255])
cor_menor = np.array([50, 50, 50])

check_delay = False

mimuX = []
imuX = 0

flag_bati = 0
flag_reconfig = 0

lower = 0
upper = 1

################################################################################
def auto_canny(image, sigma=0.33):
	global lower,upper
	# compute the median of the single channel pixel intensities
	v = np.median(image)

	# apply automatic Canny edge detection using the computed median
	lower = int(max(0, (1.0 - sigma) * v))
	upper = int(min(255, (1.0 + sigma) * v))
	edged = cv2.Canny(image, lower, upper)

	# return the edged image
	return edged

def find_features(cv_image):
	global flann, des1, sift
	global MIN_MATCH_COUNT_FEATURES, POWERPUFF_flag
	# Capture frame-by-frame
	frame = cv_image

	# Convert the frame to grayscaleg
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	# A gaussian blur to get rid of the noise in the image
	blur = cv2.GaussianBlur(gray,(5,5),0)
	# Detect the edges present in the image
	bordas = auto_canny(blur)

	img2 = frame # Imagem do cenario

	# find the keypoints and descriptors with SIFT in each image
	img2 = cv2.resize(img2,(0,0),fx = 0.18, fy = 0.18)
	kp2, des2 = sift.detectAndCompute(img2,None)

	# Tenta fazer a melhor comparacao usando o algoritmo
	matches = flann.knnMatch(des1, des2, k=2)

	good = []
	for m,n in matches:
		if m.distance < 0.8 * n.distance:
			good.append(m)

	if len(good) > MIN_MATCH_COUNT_FEATURES: # achei
		POWERPUFF_flag = 1

	else:  # Nao achei
		POWERPUFF_flag = 0

################################################################################

def identifica_cor(frame):
	global cor_maior, cor_menor, frame_hsv
	frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

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
		find_features(cv_image)
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
	global imuX, mimuX, flag_bati, flag_reconfig
	IimuX = np.array(dado.linear_acceleration.x).round(decimals=2)
	mimuX.append(IimuX)
	if len(mimuX)>10: mimuX = mimuX[5:]
	imuX = np.mean(mimuX)
	if imuX <= -3 and imuX > -8: # no outro codigo ta igual a 0
		mimuX = []
		flag_bati = 1
	elif imuX < -8:
		flag_reconfig = 1

################################################################################

class Aprender(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['Survive'])

	def execute(self, userdata):
		global velocidade_saida, flag_bati, flag_reconfig
		global frame_hsv
		global cor_maior,cor_menor
		rospy.loginfo('Executing state APRENDENDO')
		#comando para aprender a ler a cor
		vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
		velocidade_saida.publish(vel)
		time.sleep(5)
		valor_medio_cor_central = np.mean(np.array(frame_hsv)[280:360,200:280,0])
		range_limit = 10
		cor_maior = np.array([valor_medio_cor_central + range_limit, 255, 255])
		cor_menor = np.array([valor_medio_cor_central - range_limit, 50, 50])
		flag_bati = 0
		flag_reconfig = 0
		return 'Survive'

class Andando(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['Andando','Analisando','Bati'])

	def execute(self, userdata):
		global velocidade_saida
		global andar, velLimit, flag_bati
		rospy.loginfo('Executing state ANDANDO')
		#comando para andar

		X = velLimit * 2
		if andar > 0:
			vel = Twist(Vector3(X, 0, 0), Vector3(0, 0, 0))
		else:
			vel = Twist(Vector3(-X, 0, 0), Vector3(0, 0, 0))
		velocidade_saida.publish(vel)
		if andar != 0:
			if andar < 0:
				andar+=1
			else:
				andar-=1
			return "Andando"
		else:
			if flag_bati == 0:
				return 'Analisando'
			else:
				return 'Bati'

class Girando(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['Girando','Analisando'])

	def execute(self, userdata):
		global velocidade_saida
		global girar
		rospy.loginfo('Executing state GIRANDO')
		#comando para girar
		X = girar * 0.5
		vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, X))
		velocidade_saida.publish(vel)
		if girar != 0:
			if girar < 0:
				girar += 1
			else:
				girar -= 1
			return "Girando"
		else:
			return 'Analisando'

class Analisando(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['Andando','Girando','Survive'])
		self.counter = 0
		self.ObjGiro = "E"

	def execute(self, userdata):
		global andar, girar, FollowCount
		global media, centro
		rospy.loginfo('Executing state ANALISANDO')
		if self.counter < 3:
			self.counter += 1
			if POWERPUFF_flag:
				girar = 1000
				print("powerpuff girls!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
				return 'Girando'
			elif len(media) != 0 and len(centro) != 0: # lista nao ta vazia

				dif_x = media[0]-centro[0]
				dif_y = media[1]-centro[1]
				if math.fabs(dif_x) < 60: # Estou Alinhado ao Objeto
					andar += 10
					return 'Andando'

				else: #Objeto Esquerda
					if dif_x > 0:
						girar = -1
						self.ObjGiro = "D"
						print("DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD")

					elif dif_x < 0 and dif_x != -240: #Objeto direita
						self.ObjGiro = "E"
						print("EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE")
						girar = 1
					else:
						girar = 2
					return 'Girando'
			else:
				if self.ObjGiro == "D":
					girar = -2
				elif self.ObjGiro == "E":
					girar = 2
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
		global flag_bati, andar
		rospy.loginfo('Executing state BATIIIIIIIIIIIIIIIIIIIIIIIIIIIIII')
		if flag_bati == 1:
			andar -= 200
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
		smach.State.__init__(self, outcomes=['Bati', 'Evitar', 'Analisando', 'Aprender'])
		self.counter = 0

	def execute(self, userdata):
		global flag_bati
		rospy.loginfo('Executing state SURVIVE')
		if self.counter < 1:
			self.counter += 1
			if flag_reconfig:
				return 'Aprender'
			else:
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

		smach.StateMachine.add('SURVIVE', Survive(),
								transitions = {'Analisando':'ANALISANDO',
												'Bati':'BATI',
												'Evitar':"EVITAR",
												'Aprender':'APRENDENDO'})

		smach.StateMachine.add('APRENDENDO', Aprender(),
								transitions = {'Survive':'SURVIVE'})

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
											 'Girando':'GIRANDO'})



	# Execute SMACH plan
	outcome = sm.execute()

if __name__=="__main__":
	maquina()
