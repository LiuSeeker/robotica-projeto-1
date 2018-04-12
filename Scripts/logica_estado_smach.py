import smach
import smach_ros

## Importa cada uma das funcoes de seus respectivos arquivos

import desviar

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
    return desviar.scaneou()

#As duas classes abaixo servem para tocar sons do robo
class Som1(smash.State):
def __init__(self):
    smach.State.__init__(self, outcomes=['tocado'])

  def execute(self, userdata):
  
    return 'tocado'


class Som2(smash.State):
def __init__(self):
    smach.State.__init__(self, outcomes=['tocado'])

  def execute(self, userdata):
  
    return 'tocado'    
    
#Essa classe vira 180 graus
class Virar(smash.State):
def __init__(self):
    smach.State.__init__(self, outcomes=['virado'])

  def execute(self, userdata):
  
    return 'virado'

#Essa classe gira 360 graus
class Girar(smash.State):
def __init__(self):
    smach.State.__init__(self, outcomes=['girado'])

  def execute(self, userdata):
  
    return 'girado'


def main():
    global velocidade_saida
    rospy.init_node('smach_example_state_machine')
    #recebedor = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, recebe)
    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    # trazer o recebedor de laser para cá

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