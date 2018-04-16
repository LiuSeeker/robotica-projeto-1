import smach
import smach_ros

## Importa cada uma das funcoes de seus respectivos arquivos

def converte(valor):
		return valor*44.4/0.501

def roda_todo_frame(imagem):
	global cv_image
	global velocidade_saida
	global media
	global centro

	cap = cv2.VideoCapture(0)

	ret, imagem = cap.read()
	cv_image = bridge.compressed_imagemmsg_to_cv2(imagem, "bgr8")
	gray = cv2.cvtColor(imagem, cv2.COLOR_BGR2GRAY)
	faces = face_cascade.detectMultiScale(gray, 1.3, 5)
	cv2.imshow("Camera", cv_image)

	for(x,y,z,w) in faces:
		cv2.rectangle(imagem, (x,y), (x+z, y+w), (255,0,0), 2)
		roi_gray = gray[y:y+w, x:x+z]
		roi_color = imagem[y:y+w, x:x+z]

		media = x+z/2
		centro = imagem.shape[1]//2

# Treinamento de reconhecimento facial  
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

## Classes para o state machine

#Classe para procurar o objeto
class Procurar(smash.State):

	def __init__(self):
    	smach.State.__init__(self, outcomes=['objeto_1', 'objeto_2', 'nada'])

	def execute(self, userdata):
		  
    	return 'objeto_1'
    
    	return 'objeto_2'
    
    	return 'nada'     

#Apos encontrar o objeto, essa classe serve para seguir o objeto
class Seguir(smash.State):
	def __init__(self):
    	smach.State.__init__(self, outcomes=['longe', 'desviar', 'perto'])

	def execute(self, userdata):
  
    	return 'longe'
    
    	return 'desviar'
    
    	return 'perto'     

#Classe utilizada para desviar de objetos perto do robo
class Desviar(smash.State):
	def __init__(self):
    	smach.State.__init__(self, outcomes=['desviado', 'desviando'])

  	def execute(self, userdata):
      distancias = np.array(dado.ranges)
      desviando = False

      for i in range(len(distancias)):

  			if i <= 40:
  				if converte(distancias[i]) < 50 and converte(distancias[i]) >= 30:
  					velocidade = Twist(Vector3(0.2, 0, 0), Vector3(0, 0, -0.7))
  					desviando = True
  				elif converte(distancias[i]) < 30 and  converte(distancias[i]) != 0:
  					velocidade = Twist(Vector3(-0.1, 0, 0), Vector3(0, 0, -0.9))
  					desviando = True
  				
  			if i >= 320:
  				if converte(distancias[i]) < 50 and converte(distancias[i]) >= 30:
  					velocidade = Twist(Vector3(0.2, 0, 0), Vector3(0, 0, 0.7))
  					desviando = True
  				elif converte(distancias[i]) < 30 and  converte(distancias[i]) != 0:
  					velocidade = Twist(Vector3(-0.1, 0, 0), Vector3(0, 0, 0.9))
  					desviando = True
  				
  			if i <= 70 and i > 40:
  				if converte(distancias[i]) < 25 and converte(distancias[i]) != 0:
  					velocidade = Twist(Vector3(0.1, 0, 0), Vector3(0, 0, -0.5))
  					desviando = True

  			if i < 320 and i >= 290:
  				if converte(distancias[i]) < 25 and converte(distancias[i]) != 0:
  					velocidade = Twist(Vector3(0.1, 0, 0), Vector3(0, 0, 0.5))
  					desviando = True

  			if i < 290 and i > 70:
  				if converte(distancias[i]) < 25 and converte(distancias[i]) != 0:
  					velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.5))
  					desviando = True

  		if velocidade != None:
  			velocidade_saida.publish(velocidade)
  			return 'desviando'
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
    
#Essa classe vira 180 graus
class Virar(smash.State):
	def __init__(self):
    	smach.State.__init__(self, outcomes=['virado'])

  	def execute(self, userdata):
  	quat = dado.orientation
  		lista = [quat.x, quat.y, quat.z, quat.w]
  		angulos = np.degrees(transformations.euler_from_quaternion(lista))

  		ang_inicial, ang_final = angulos_inicial_e_final()

  		ang_atual = angulos[0]

  		velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.7))

  		if ang_atual <= ang_final+3 and ang_atual >= ang_final-3:
  			velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

  		velocidade_saida.publish(velocidade)
      	return 'virado'

#Essa classe gira 360 graus
class Girar(smash.State):
	def __init__(self):
    	smach.State.__init__(self, outcomes=['girado'])

  	def execute(self, userdata):
  
    	return 'girado'


def main():
    global velocidade_saida
    global ang_inicial
    global ang_final
    rospy.init_node('smach_example_state_machine')
    #recebedor = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, recebe)
    saida_som = rospy.Publisher("/sound", Sound, queue_size = 2)

    recebedor = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 2)
    # trazer o recebedor de laser para cá
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)


    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['terminei'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('PROCURAR', Procurar(), 
                               transitions={'objeto_1':'SEGUIR', #se achar o objeto 1, retorna 'objeto_1' e roda 'SEGUIR'
                                            'objeto_2':'SOM_2', #se achar o objeto 2, retorna 'objeto_2' e roda 'SOM_2'
                                            'nada': 'GIRAR'}) #se não achar nada, retorna 'nada' e roda "GIRAR"

        smach.StateMachine.add('SEGUIR', Seguir(), 
                               transitions={'longe':'SEGUIR', #se nao scnear nada perto, estiver longe do objeto e estiver com o objeto centralizado, retorna 'longe' e roda 'SEGUIR'
                                            'desviar': 'DESVIAR', #se scanear qq coisa perto (direção qualquer), retorna 'desviar' e roda 'DESVIAR'
                                            'perto': 'SOM_1'}) #se estiver perto do objeto, retorna 'perto' e roda 'SOM_1'
      
        smach.StateMachine.add('DESVIAR', Desviar(), 
                               transitions={'desviado': 'PROCURAR'},
                                            'desviando': 'DESVIAR') #scaneia a direção e desvia, e retorna 'desviado' e roda 'PROCURAR'
      
        smach.StateMachine.add('SOM_1', Som1(), 
                               transitions={'tocado': 'PROCURAR'}) #toca o som1, e retorna 'tocado' e roda 'PROCURAR'
      
        smach.StateMachine.add('SOM_2', Som2(), 
                               transitionsd={'tocado': 'VIRAR'}) #toca o som2, e retorna 'tocado' e roda 'VIRAR'
      
        smach.StateMachine.add('VIRAR', Virar(), 
                               transitions={'virado': 'PROCURAR'}) #vira 180 graus, e retorna 'virado' e roda 'PROCURAR'
      
        smach.StateMachine.add('GIRAR', Girar(), 
                               transitions={'girado': 'PROCURAR'}) #gira 360 graus, e retorna 'girado' e roda 'PROCURAR'

    # Execute SMACH plan
    outcome = sm.execute()


if _name_ == '_main_':
    main()