def main():
    global velocidade_saida
    rospy.init_node('smach_example_state_machine')
    #recebedor = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, recebe)
    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

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


if __name__ == '__main__':
    main()
