#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Imu
import transformations
import math


def leu_imu(dado):
	quat = dado.orientation
	lista = [quat.x, quat.y, quat.z, quat.w]
	angulos = np.degrees(transformations.euler_from_quaternion(lista))
	mensagem = """
	Tempo: {:}
	Orientação: {:.2f}, {:.2f}, {:.2f}
	Vel. angular: x {:.2f}, y {:.2f}, z {:.2f}\

	Aceleração linear:
	x: {:.2f}
	y: {:.2f}
	z: {:.2f}


""".format(dado.header.stamp, angulos[0], angulos[1], angulos[2], dado.angular_velocity.x, dado.angular_velocity.y, dado.angular_velocity.z, dado.linear_acceleration.x, dado.linear_acceleration.y, dado.linear_acceleration.z)
	print(mensagem)

def girar(graus):

	PI = 3.1415926535897
	#Starts a new node
	rospy.init_node('robot_cleaner', anonymous=True)
	velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
	vel_msg = Twist()

    # Receiveing the user's input
	print("Let's rotate your robot")
	speed = 0.6
	angle = graus
	clockwise = False

    #Converting from angles to radians
	angular_speed = speed*2*PI/360
	relative_angle = angle*2*PI/360

    #We wont use linear components
	vel_msg.linear.x=0
	vel_msg.linear.y=0
	vel_msg.linear.z=0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0

    # Checking if our movement is CW or CCW
	if clockwise:
	    vel_msg.angular.z = -abs(angular_speed)
	else:
	    vel_msg.angular.z = abs(angular_speed)
    # Setting the current time for distance calculus
	t0 = rospy.Time.now().to_sec()
	current_angle = 0

	while(current_angle < relative_angle):
	    velocity_publisher.publish(vel_msg)
	    t1 = rospy.Time.now().to_sec()
	    current_angle = angular_speed*(t1-t0)

    #Forcing our robot to stop
	vel_msg.angular.z = 0
	velocity_publisher.publish(vel_msg)
	rospy.spin()



if __name__=="__main__":

	#rospy.init_node("le_imu")

	#recebe_scan = rospy.Subscriber("/imu", Imu, leu_imu)

	try:
		girar(180)
	except rospy.ROSInterruptException:
		pass

	#while not rospy.is_shutdown():
#		print("Main loop")
#		rospy.sleep(2)


