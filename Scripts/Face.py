import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist, Vector3, Pose

face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

def main():
	cap = cv2.VideoCapture(0)

	while not rospy.is_shutdown():
		vel = Twist(Vector3(0,0,0), Vector3(0,0,0))

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
				vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.2))
			elif dif_x < -tolerancia:
				# Vira para a esquerda
				vel = Twist(Vector3(0,0,0), Vector3(0,0,0.2))
			else:
				# Anda, se centralizado
				vel = Twist(Vector3(0.5,0,0), Vector3(0,0,0))


		cv2.imshow('ROSTOOOOOOOOOO',img)

		key  = cv2.waitKey(1) & 0xFF
		if key == ord('q'):
			break

	cap.release()
	cv2.destroyAllWindows()

if __name__ == "__main__":
	main()