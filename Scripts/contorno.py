import rospy
import smach


#ARRUMAR OS IMPORTS 

def area_cv(frame):
	global centro, media, maior_cont
	frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	cor_menor = np.array([50, 50, 50])
	cor_maior = np.array([70, 255, 255])
	segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)


	segmentado_cor = cv2.morphologyEx(segmentado_cor,cv2.MORPH_CLOSE,np.ones((7, 7)))

	img_out, contornos = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	maior_area = 0
	maior_contorno = None


	for contorno in contornos:
		a = cv2.contourArea(contorno)
		if a > maior_area:
			maior_area = a
			maior_contorno = contorno
			

	if not maior_contorno is None :
		cv2.drawContours(frame, [maior_contorno], -1, [0, 0, 255], 5)
		maior_contorno = np.reshape(maior_contorno, (maior_contorno.shape[0], 2))
		media = maior_contorno.mean(axis=0)
		media = media.astype(np.int32)
		cv2.circle(frame, tuple(media), 5, [0, 255, 0])
	else:
		media = (0, 0)

	centro = (frame.shape[0]//2, frame.shape[1]//2)

	return media, centro