#! /usr/bin/env python
# -*- coding:utf-8 -*-

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

bridge = CvBridge()

cv_image = None
media = []
centro = []
atraso = 1.5

check_delay = False # Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados

def identifica_cor(frame):
	'''
	Segmenta o maior objeto cuja cor é parecida com cor_h (HUE da cor, no espaço HSV).
	'''

	# No OpenCV, o canal H vai de 0 até 179, logo cores similares ao
	# vermelho puro (H=0) estão entre H=-8 e H=8.
	# Precisamos dividir o inRange em duas partes para fazer a detecção
	# do vermelho:
	frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	cor_menor = np.array([50, 50, 50])
	cor_maior = np.array([70, 255, 255])
	segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)

	cor_menor = np.array([0, 50, 50])
	cor_maior = np.array([20, 255, 255])
	Vermelho_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)


	# A operação MORPH_CLOSE fecha todos os buracos na máscara menores
	# que um quadrado 7x7. É muito útil para juntar vários
	# pequenos contornos muito próximos em um só.
	segmentado_cor = cv2.morphologyEx(segmentado_cor,cv2.MORPH_CLOSE,np.ones((7, 7)))
	Vermelho_cor = cv2.morphologyEx(Vermelho_cor,cv2.MORPH_CLOSE,np.ones((7, 7)))

	# Encontramos os contornos na máscara e selecionamos o de maior área
	#contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	img_out, contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	Vimg_out, Vcontornos, Varvore = cv2.findContours(Vermelho_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	maior_contorno = None
	maior_contorno_area = 0

	Vmaior_contorno = None
	Vmaior_contorno_area = 0

	for cnt in contornos:
	    area = cv2.contourArea(cnt)
	    if area > maior_contorno_area:
	        maior_contorno = cnt
	        maior_contorno_area = area

	for cnt in Vcontornos:
		Varea = cv2.contourArea(cnt)
		if Varea > Vmaior_contorno_area:
			Vmaior_contorno = cnt
			Vmaior_contorno_area = Varea

	# Encontramos o centro do contorno fazendo a média de todos seus pontos.
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

	return media, centro, Vmaior_contorno


def roda_todo_frame(imagem):
	print("frame")
	global cv_image
	global media
	global centro
	global Vcontorno

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
		media, centro, Vcontorno = identifica_cor(cv_image)
		depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)



if __name__=="__main__":

	rospy.init_node("cor")
	recebedor = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)
	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3)
	#ecebedor = rospy.Subscriber("/camera/rgb/image_raw", Image, roda_todo_frame, queue_size=2)
	print("Start")

	try:

		while not rospy.is_shutdown():

			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
			if len(media) != 0 and len(centro) != 0:
				dif_x = media[0]-centro[0]
				dif_y = media[1]-centro[1]
				print(dif_x)
				if math.fabs(dif_x)<30: # Se a media estiver muito proxima do centro anda para frente
					print("F")
					vel = Twist(Vector3(0.5,0,0), Vector3(0,0,0))
				else:
					if dif_x > 0: # Vira a direita
						print("D")
						vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.2))
					else: # Vira a esquerda
						print("E")
						vel = Twist(Vector3(0,0,0), Vector3(0,0,0.2))
			try:
				print(len(Vcontorno))
				if len(Vcontorno) > 1000:
					vel = Twist(Vector3(0,0,0), Vector3(0,0,5))
					velocidade_saida.publish(vel)
					print("OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO")

			except:
				pass
			velocidade_saida.publish(vel)
			rospy.sleep(0.01)

	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")
