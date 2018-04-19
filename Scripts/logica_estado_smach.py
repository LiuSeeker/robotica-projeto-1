import smach
import smach_ros
import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage, LaserScan, Imu
from cv_bridge import CvBridge, CvBridgeError
import smach
import smach_ros
import cormodule
import tranformations

#xml do haarcascade com o treinamento dos rostos de gatos
face_cascade = cv2.CascadeClassifier('haarcascade_frontalcatface.xml')
bridge = CvBridge()
global cv_image
global dif_x
global media
global centro
global imu
cv_image = None
dif_x = None
media = 0
centro = 0
media0 = None
imu = []
atraso = 1.5E9
delay_frame = 0.05
bateu = None

######################################################################################################################################

#Função que analisa o IMU para detectar colisão
def Imu(dado):
	global imu, imu_acele, imu_media, bateu, ang
	imu_acele = np.array(dado.linear_acceleration.x).round(decimals=2)
	imu.append(imu_acele)
	
	#Pegando a media da lista recebida
	if len(imu) >= 12: 
		imu = imu[6:]
	
	imu_media = np.mean(imu)
	ang = math.degrees(math.atan2(dado.linear_acceleration.x, dado.linear_acceleration.y))
	
	#Analisando se bateu
	if abs(imu[-1] - imu_media) >= 3.5:
		imu = []
		bateu = True
		return "Bateu"
	return "Procurar"

#Caso ele bata com imu essa função ajuda ele a sair
def Colidiu(ang, dif):
	global bateu

	if ang <= 80:
		vel = Twist(Vector3(-1.5,0,0), Vector3(0,0,-2))
		velocidade_saida.publish(vel)
		rospy.sleep(2)

	elif ang < 100 and ang > 80:
		vel = Twist(Vector3(-1.5,0,0), Vector3(0,0,2))
		velocidade_saida.publish(vel)
		rospy.sleep(2)

	elif ang >= 100:
		vel = Twist(Vector3(-1.5,0,0), Vector3(0,0,2))
		velocidade_saida.publish(vel)
		rospy.sleep(1.5)


#Converte o valor da medida do LaserScan para centímetros
def converte(valor):
	return valor*44.4/0.501

#Função que recebe a imagem da câmera
def roda_todo_frame(imagem):
	global cv_image
	global media
	global centro
	global dif_x
	global p
	global area1, area2

	#Calcula o lag de frames
	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.nsecs
	if delay > atraso and check_delay==True:
		print("delay: {}".format(delay/1.0E9))
		return

	try:
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")

		#Deixa a imagem da câmera preto e branco para ser utilizada na detecção de faces
		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		faces = face_cascade.detectMultiScale(gray, 1.3, 5)

		#Detecta os pontos extremos da face
		for(x,y,z,w) in faces:
			cv2.rectangle(cv_image, (x,y), (x+z, y+w), (255,0,0), 2) #Desenha um retângulo em volta da face
			roi_gray = gray[y:y+w, x:x+z]
			roi_color = cv_image[y:y+w, x:x+z]

			media = x+z/2 #Calcula a posição em x do centro da face
			centro = cv_image.shape[0]//1.5 #Calcula a posição em x do centro da imagem

			#Identificando a primeira interacao do 'for' (ao reconhecer gato) para calcular a area da figura
			if p == 0:
				area1 = z*w
				area2 = 0
				p = 1

			#Calculando a area em todas as outras interacoes para definir a velocidade proporcional do robo
			elif p == 1:
				area2 = z*w

			#Caso tenha achado uma face, calcula a diferença entre o centro da face e o centro da imagem
			if media != 0 :
				dif_x = media-centro
			else:
				dif_x = None

		#Define o centro do objeto detectada por cor e o centro da imagem
		media0, centro0, area0 = cormodule.identifica_cor(cv_image)

		#Printa a imagem da câmera
		cv2.imshow("Camera", cv_image)
		cv2.waitKey(1)

	except CvBridgeError as e:
		print("except", e)

#Define a lista de distâncias do scan do LaserScan
def scaneou(dado):
	global distancias
	distancias = np.array(dado.ranges)

#Define o posicionamento pelo IMU
def leu_imu(dado):
	global angulos
	quat = dado.orientation
	lista = [quat.x, quat.y, quat.z, quat.w]
	angulos = np.degrees(transformations.euler_from_quaternion(lista))


#####################################################################################################################################

## Classes para o state machine

#Classe para procurar objetos de interesse
class Procurar(smash.State):

	def __init__(self):
    	smach.State.__init__(self, outcomes=['objeto_1', 'objeto_2', 'nada'])

	def execute(self, userdata):
		global velocidade
		velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.3))

		#Se acha a face:
		if dif_x != None:
    		return 'objeto_1'
    	#Se acha o objeto pela cor de interesse
    	if media0 != (0,0):
    		return 'objeto_2'
    	#Se não acha nada
    	else:
    		return 'nada'

#Apos encontrar o objeto, essa classe serve para seguir o objeto
class Seguir(smash.State):
	def __init__(self):
    	smach.State.__init__(self, outcomes=['longe', 'mt_longe', 'desviar', 'perto', 'perdido'])

	def execute(self, userdata):
  		global velocidade_saida, bateu
		tolerancia = 20
		desviar = False
		longe = False
		mt_longe = False
		perdido = False

		if bateu:
			Colidiu(#angulo
				)
			bateu = False
		#Utilizando as distancias recebidas na funcao scaneou()
		#A primeira distância na lista é a distância em frente ao robô,
		#e o scan segue sentido anti-horário a partir da primeira distância (a frente do robô)
		for i in range(len(distancias)):
			#Faixa de scan na parte frontal esquerda do robô
			if i <= 40:
				#Se houver algo nessa faixa a uma distância menor que 50cm, deixa a flag 'Desviar' como True
				if converte(distancias[i]) < 50 and converte(distancias[i]) != 0:
					desviar = True
			#Faixa de scan na parte frontal direita do robô
			if i >= 320:
				#Se houver algo nessa faixa a uma distância menor que 50cm, deixa a flag 'Desviar' como True
				if converte(distancias[i]) < 50 and converte(distancias[i]) != 0:
					desviar = True
			#Faixa de scan na parte esquerda do robô
			if i <= 70 and i > 40:
				#Se houver algo nessa faixa a uma distância menor que 25cm, deixa a flag 'Desviar' como True
				if converte(distancias[i]) < 25 and converte(distancias[i]) != 0:
					desviar = True
			#Faixa de scan na parte direita do robô
			if i < 290 and i > 70:
				#Se houver algo nessa faixa a uma distância menor que 25cm, deixa a flag 'Desviar' como True
				if converte(distancias[i]) < 25 and converte(distancias[i]) != 0:
					desviar = True

		#Centralizando o rosto (no caso, do gato) de acordo com a tolerancia
		if  dif_x > tolerancia:
			velocidade = Twist(Vector3(0,0,0), Vector3(0,0,-0.2))
			rospy.sleep(delay_frame)
			longe = True

		if dif_x < -tolerancia and dif_x != None:
			velocidade = Twist(Vector3(0,0,0), Vector3(0,0,0.2))
			rospy.sleep(delay_frame)
			longe = True

		#Uma vez centralizado, comparamos a area que o objeto ocupa do frame em relacao a area que ocupava inicialmente
		#para decidir a velocidade que o robo pode andar
		if dif_x > -tolerancia and dif_x < tolerancia:
			#Caso a area esteja muito grande (ou seja, o objeto esta perto do robo), ele para, de maneira que nao colida
			if area2 = area1*1.1:
				velocidade = Twist(Vector3(0,0,0), Vector3(0,0,0))
				rospy.sleep(delay_frame)
				#perto = True
				velocidade_saida.publish(velocidade)
    			return 'perto'

    		#Se o objeto estiver longe, podemos acelerar o robo
			elif area2 <= area1 and area2 > area1*0.6:
				velocidade = Twist(Vector3(0.3,0,0), Vector3(0,0,0))
				rospy.sleep(delay_frame)
				#longe = True
				velocidade_saida.publish(velocidade)
    			return 'longe'

    		#Caso esteja muito longe, sua velocidade é maior ainda
			elif area2 <= area1*0.6:
				velocidade = Twist(Vector3(0.6,0,0), Vector3(0,0,0))
				rospy.sleep(delay_frame)
				#mt_longe = True
				velocidade_saida.publish(velocidade)
    			return 'mt_longe'

    	#Se a 
		if dif_x == None:
			perdido = True

		if desviar:
			return 'desviar'

    	if perdido:
    		return 'perdido'

		'''Comentei essas linhas para mostrar que elas estavam aqui, caso algo de errado, mas é possivel fazer as funcoes
		   delas no codigo
    	if longe:
    		velocidade_saida.publish(velocidade)
    		return 'longe'

    	if mt_longe:
    		velocidade_saida.publish(velocidade)
    		return 'mt_longe'

    	if perto:
    		velocidade_saida.publish(velocidade)
    		return 'perto'''



#Classe utilizada para desviar de objetos perto do robo
class Desviar(smash.State):
	def __init__(self):
    	smach.State.__init__(self, outcomes=['desviado', 'desviando'])

  	def execute(self, userdata):
  		global desviando, bateu

  		desviando = False

  		if bateu:
  			Colidiu(#angulo
  				)
  			bateu = False

  		rospy.sleep(0.05)

		for i in range(len(distancias)):
			#Faixa de scan na parte frontal esquerda do robô
			if i <= 40:
				#Se houver algo nessa faixa a uma distância menor que 50cm, desvia
				if converte(distancias[i]) < 50 and converte(distancias[i]) >= 30:
					velocidade = Twist(Vector3(0.2, 0, 0), Vector3(0, 0, -0.7))
					desviando = True
				#Se houver algo nessa faixa a uma distância menor que 30cm, para e desvia
				elif converte(distancias[i]) < 30 and  converte(distancias[i]) != 0:
					velocidade = Twist(Vector3(-0.1, 0, 0), Vector3(0, 0, -0.9))
					desviando = True
			#Faixa de scan na parte frontal direita do robô
			if i >= 320:
				#Se houver algo nessa faixa a uma distância menor que 50cm, desvia
				if converte(distancias[i]) < 50 and converte(distancias[i]) >= 30:
					velocidade = Twist(Vector3(0.2, 0, 0), Vector3(0, 0, 0.7))
					desviando = True
				#Se houver algo nessa faixa a uma distância menor que 30cm, para e desvia
				elif converte(distancias[i]) < 30 and  converte(distancias[i]) != 0:
					velocidade = Twist(Vector3(-0.1, 0, 0), Vector3(0, 0, 0.9))
					desviando = True
			#Faixa de scan na parte esquerda do robô
			if i <= 70 and i > 40:
				#Se houver algo nessa faixa a uma distância menor que 25cm, desvia
				if converte(distancias[i]) < 25 and converte(distancias[i]) != 0:
					velocidade = Twist(Vector3(0.1, 0, 0), Vector3(0, 0, -0.5))
					desviando = True
			#Faixa de scan na parte direita do robô
			if i < 320 and i >= 290:
				#Se houver algo nessa faixa a uma distância menor que 25cm, desvia
				if converte(distancias[i]) < 25 and converte(distancias[i]) != 0:
					velocidade = Twist(Vector3(0.1, 0, 0), Vector3(0, 0, 0.5))
					desviando = True
			#Faixa de scan na parte traseira do robô
			if i < 290 and i > 70:
				#Se houver algo nessa faixa a uma distância menor que 25cm, para e desvia
				if converte(distancias[i]) < 25 and converte(distancias[i]) != 0:
					velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.5))
					desviando = True

		#Se estiver desviando, publica a velocidade e retorna 'desviando'
		if desviando:
			velocidade_saida.publish(velocidade)
			return 'desviando'
		#Se já acabou de desviar, retorna 'desviado'
		else:
			return 'desviado'

#As duas classes abaixo servem para tocar sons do robo
class Som1(smash.State):
	def __init__(self):
    	smach.State.__init__(self, outcomes=['tocado'])

  	def execute(self, userdata):
  		saida_som.publish(0)
    	return 'tocado'

class Som2(smash.State):
	def __init__(self):
    	smach.State.__init__(self, outcomes=['tocado'])

  	def execute(self, userdata):
  		saida_som.publish(2)
    	return 'tocado'

#Classe que, quando rodada, grava o angulo atual como o inicial e define o angulo final
class Pos_ini(smash.State):
	def __init__(self):
    	smach.State.__init__(self, outcomes=['pego'])

    def execute(self, userdata):
    	global ang_inicial
		global ang_final
		global ang_varia
		ang_inicial = angulos[0]
		ang_varia = 180

		if ang_inicial < ang_varia and ang_inicial >= 0:
			ang_final = ang_inicial - ang_varia
		elif ang_inicial > -ang_varia and ang_inicial < 0:
			ang_final = ang_inicial + ang_varia

		return 'pego'

#Essa classe vira 180 graus
class Virar(smash.State):
	def __init__(self):
    	smach.State.__init__(self, outcomes=['virando', 'virado'])

  	def execute(self, userdata):
  		global velocidade_saida
  		ang_atual = angulos[0]

		velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.7))

		#Se já acabou de virar, retorna 'virado'
		if ang_atual <= ang_final+3 and ang_atual >= ang_final-3:
			return 'virado'
		#Se estiver virando, publica a velocidade e retorna 'virando'
		else:
			velocidade_saida.publish(velocidade)
			return 'virando'

def main():
    global velocidade_saida
    global ang_inicial
	global ang_final
    rospy.init_node('smach_example_state_machine')
    #Publica o som
	saida_som = rospy.Publisher("/sound", Sound, queue_size = 2)
	#Dá subscribe na função 'roda_todo_frame'
	recebedor = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)
	#Publica a velocidade
    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 2)
    #Dá subscribe na função 'scaneou'
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
    #Dá subscribe na função 'leu_imu'
    recebe_imu = rospy.Subscriber("/imu", Imu, leu_imu)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['terminei'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('PROCURAR', Procurar(),
                               transitions={'objeto_1':'SEGUIR', #se achar o objeto 1, retorna 'objeto_1' e executa 'SEGUIR'
                                            'objeto_2':'SOM_2', #se achar o objeto 2, retorna 'objeto_2' e executa 'SOM_2'
                                            'nada': 'PROCURAR'}) #se não achar nada, retorna 'nada' e executa "PROCURAR"

        smach.StateMachine.add('SEGUIR', Seguir(),
                               transitions={'longe':'SEGUIR', #se nao scnear nada perto, estiver longe do objeto, retorna 'longe' e executa 'SEGUIR'
                               				'mt_longe': "SEGUIR", #se nao scnear nada perto, estiver muito longe do objeto, retorna 'longe' e executa 'SEGUIR'
                                            'desviar': 'DESVIAR', #se scanear qq coisa perto (direção qualquer), retorna 'desviar' e executa 'DESVIAR'
                                            'perto': 'SOM_1', #se estiver perto do objeto, retorna 'perto' e executa 'SOM_1'
                                            'perdido': 'PROCURAR'}) #perder o objeto, retorna 'perdido' e executa 'PROCURAR'

        smach.StateMachine.add('DESVIAR', Desviar(),
                               transitions={'desviado': 'PROCURAR'}, #se desviou, retorna 'desviado' e executa 'PROCURAR'
                                            'desviando': 'DESVIAR') #scaneia a direção e desvia, e retorna 'desviado' e executa 'PROCURAR'

        smach.StateMachine.add('SOM_1', Som1(),
                               transitions={'tocado': 'PROCURAR'}) #toca o som1, e retorna 'tocado' e executa 'PROCURAR'

        smach.StateMachine.add('SOM_2', Som2(),
                               transitionsd={'tocado': 'POS_INI'}) #toca o som2, e retorna 'tocado' e executa 'PROCURAR'

      	smach.StateMachine.add('POS_INI', Pos_ini(),
                               transitionsd={'pego': 'VIRAR'}) #grava a posição atual, e retorna 'pego' e executa 'VIRAR'

        smach.StateMachine.add('VIRAR', Virar(),
                               transitions={'virado': 'PROCURAR', #se virou 180 graus, retorna 'virado' e executa 'PROCURAR'
                               				'virando': 'VIRAR'}) #se está virado 180 graus, e retorna 'virando' e executa 'VIRAR'

    # Execute SMACH plan
    outcome = sm.execute()

if _name_ == '_main_':
   	main()
