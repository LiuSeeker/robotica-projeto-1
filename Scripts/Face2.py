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
global area1, area2
global p
cv_image = None
dif_x = None
area1, area2 = 0,0
atraso = 1.5E9
delay_miranda = 0.05

# Variáveis para permitir que o roda_todo_frame troque dados com a máquina de estados
media = 0
centro = 0
p = False


def roda_todo_frame(imagem):
	print("frame")
	global cv_image
	global media
	global centro
	global dif_x
	global p
	global area1, area2

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.nsecs

	if delay > atraso and check_delay==True:
		print("delay: {}".format(delay/1.0E9))
		return
	try:
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")

		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		faces = face_cascade.detectMultiScale(gray, 1.3, 5)

		
		for(x,y,z,w) in faces:

			cv2.rectangle(cv_image, (x,y), (x+z, y+w), (255,0,0), 2)
			roi_gray = gray[y:y+w, x:x+z]
			roi_color = cv_image[y:y+w, x:x+z]

			media = x+z/2
			centro = cv_image.shape[0]//1.5

			if p == False:
				area1 = z*w
				area2 = 0
				p = True

			elif p ==True:
				area2 = z*w

			print(area1,area2)


			if media != 0 :
				dif_x = media-centro
			else:
				dif_x = None

		cv2.imshow("Camera", cv_image)
		cv2.waitKey(1)

	except CvBridgeError as e:
		print("except", e)



## Classes - estados


class Segue(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['segue'])

    def execute(self, userdata):
		global velocidade_saida
		tolerancia = 20

		if  dif_x > tolerancia:
			velocidade = Twist(Vector3(0,0,0), Vector3(0,0,-0.2))
			velocidade_saida.publish(velocidade); rospy.sleep(delay_miranda)
			return 'segue'
		elif dif_x < -tolerancia and dif_x != None:
			velocidade = Twist(Vector3(0,0,0), Vector3(0,0,0.2))
			velocidade_saida.publish(velocidade); rospy.sleep(delay_miranda)
			return 'segue'
		elif dif_x > -tolerancia and dif_x < tolerancia:
			if area2 > area1:
				velocidade = Twist(Vector3(0.1,0,0), Vector3(0,0,0))
			elif area2 <= area1:
				velocidade = Twist(Vector3(0.4,0,0), Vector3(0,0,0))
			velocidade_saida.publish(velocidade); rospy.sleep(delay_miranda)

			return 'segue'
		elif dif_x == None:
			velocidade = Twist(Vector3(0,0,0), Vector3(0,0,0.5))
			velocidade_saida.publish(velocidade); rospy.sleep(delay_miranda)
			return 'segue'
		else:
			return 'segue'

# main
def main():
	global velocidade_saida
	rospy.init_node('cor_estados')

	recebedor = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	sm = smach.StateMachine(outcomes=['terminei'])

	with sm:
	    smach.StateMachine.add('SEGUE', Segue(),
	                            transitions={'segue': 'SEGUE'})

	outcome = sm.execute()

if __name__ == '__main__':
    main()
