#! /usr/bin/env python
# -*- coding:utf-8 -*-

__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda"]


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import smach
import smach_ros


face_cascade = cv2.CascadeClassifier('haarcascade_frontalcatface.xml')


bridge = CvBridge()

global cv_image
global dif_x
global media
global centro
cv_image = None
dif_x = None

atraso = 1.5E9
delay_miranda = 0.05

# Variáveis para permitir que o roda_todo_frame troque dados com a máquina de estados
media = 0
centro = 0



def roda_todo_frame(imagem):
	print("frame")
	global cv_image
	global media
	global centro
	global dif_x

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.nsecs

	if delay > atraso and check_delay==True:
		print("delay: {}".format(delay/1.0E9))
		return
	try:
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		cv_top = cv_image.copy()
		cv2.imshow("Camera", cv_top)

		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		faces = face_cascade.detectMultiScale(gray, 1.3, 5)

		for(x,y,z,w) in faces:
			cv2.rectangle(cv_image, (x,y), (x+z, y+w), (255,0,0), 2)
			roi_gray = gray[y:y+w, x:x+z]
			roi_color = cv_image[y:y+w, x:x+z]

			media = x+z/2
			centro = cv_image.shape[0]//2

			if media != 0 :
				dif_x = media-centro
			else:
				dif_x = None

		print("nuaunw")

	except CvBridgeError as e:
		print("except", e)



## Classes - estados


class Girando(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['alinhou', 'girando'])

    def execute(self, userdata):
		global velocidade_saida
		tolerancia = 20

		if  dif_x > tolerancia:
			velocidade = Twist(Vector3(0,0,0), Vector3(0,0,-0.2))
			velocidade_saida.publish(velocidade); rospy.sleep(delay_miranda)
			return 'girando'
		elif dif_x < -tolerancia and dif_x != None:
			velocidade = Twist(Vector3(0,0,0), Vector3(0,0,0.2))
			velocidade_saida.publish(velocidade); rospy.sleep(delay_miranda)
			return 'girando'
		elif dif_x > -tolerancia and dif_x < tolerancia:
			velocidade = Twist(Vector3(0.2,0,0), Vector3(0,0,0))
			velocidade_saida.publish(velocidade); rospy.sleep(delay_miranda)#! /usr/bin/env python

			return 'alinhou'
		elif dif_x == None:
			velocidade = Twist(Vector3(0,0,0), Vector3(0,0,0.5))
			velocidade_saida.publish(velocidade); rospy.sleep(delay_miranda)
			return 'girando'


class Centralizado(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['alinhando', 'alinhado'])

    def execute(self, userdata):
		global velocidade_saida

		tolerancia = 50

		if  dif_x > tolerancia:
			return 'alinhando'
		elif dif_x < -tolerancia:
			return 'alinhando'
		elif dif_x > -tolerancia and dif_x < tolerancia:
			print(dif_x)
			velocidade = Twist(Vector3(0.5,0,0), Vector3(0,0,0))
			velocidade_saida.publish(velocidade); rospy.sleep(delay_miranda)
			return 'alinhado'
		else:
			return 'alinhando'

# main
def main():
	global velocidade_saida
	global buffer
	rospy.init_node('cor_estados')

	# Para usar a webcam 
	#recebedor = rospy.Subscriber("/cv_camera/image_raw/compressed", CompressedImage, roda_todo_frame, queue_size=1, buff_size = 2**24)
	recebedor = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['terminei'])

	# Open the container
	with sm:
	    # Add states to the container
	    #smach.StateMachine.add('LONGE', Longe(), 
	    #                       transitions={'ainda_longe':'ANDANDO', 
	    #                                    'perto':'terminei'})
	    #smach.StateMachine.add('ANDANDO', Andando(), 
	    #                       transitions={'ainda_longe':'LONGE'})
	    smach.StateMachine.add('GIRANDO', Girando(),
	                            transitions={'girando': 'GIRANDO',
	                            'alinhou':'CENTRO'})
	    smach.StateMachine.add('CENTRO', Centralizado(),
	                            transitions={'alinhando': 'GIRANDO',
	                            'alinhado':'CENTRO'})


	# Execute SMACH plan
	outcome = sm.execute()
	#rospy.spin()


if __name__ == '__main__':
    main()
