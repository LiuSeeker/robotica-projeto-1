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
import cormodule

face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

bridge = CvBridge()

cv_image = None


def roda_todo_frame(imagem):
	global cv_image
	global velocidade_saida

	cap = cv2.VideoCapture(0)

	ret, img = cap.read()
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	faces = face_cascade.detectMultiScale(gray, 1.3, 5)

	for(x,y,z,w) in faces:
		cv2.rectangle(img, (x,y), (x+z, y+w), (255,0,0), 2)
		roi_gray = gray[y:y+w, x:x+z]
		roi_color = img[y:y+w, x:x+z]

		tolerancia = 50
		media = x+z/2
		centro = img.shape[1]//2

		if media != 0 and centro != 0:
			dif_x = media-centro
		if dif_x > tolerancia:
			# Vira para a direita
			velocidade = Twist(Vector3(0.1,0,0), Vector3(0,0,-0.2))
			print("Vira direita")
		elif dif_x < -tolerancia:
			# Vira para a esquerda
			velocidade = Twist(Vector3(0.1,0,0), Vector3(0,0,0.2))
			print("Vira esquerda")
		elif dif_x <= tolerancia and dif_x >= -tolerancia:
			# Anda, se centralizado
			print("centralizado")
			velocidade = Twist(Vector3(0.5,0,0), Vector3(0,0,0))
			dist = 20*3.04/w
			print(dist)
		else:
			print("Nada")
			velocidade = Twist(Vector3(0,0,0), Vector3(0,0,0))

	velocidade_saida.publish(velocidade)


	


if __name__=="__main__":
	global velocidade_saida

	rospy.init_node("cor")

	# Para usar a Raspberry Pi
	topico_raspberry_camera = "/raspicam_node/image/compressed"

	topico_imagem = topico_raspberry_camera

	recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)


	while not rospy.is_shutdown():
		rospy.sleep(0.5)

