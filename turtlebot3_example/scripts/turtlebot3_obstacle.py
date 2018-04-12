#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Obstacle():
    def __init__(self):
        self.LIDAR_ERR = 0.05
        self._cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.obstacle()

    def get_scan(self, data):
        scan = LaserScan()
        scan = data
        for i in range(360):
            if i <= 15 or i > 335:
                if scan.ranges[i] >= self.LIDAR_ERR:
                    self.scan_filter.append(scan.ranges[i])

    def obstacle(self):
        self.twist = Twist()
        while not rospy.is_shutdown():
            msg = rospy.wait_for_message("/scan", LaserScan)
            self.scan_filter = []
            for i in range(360):
                if i <= 15 or i > 335:
                    if msg.ranges[i] >= self.LIDAR_ERR:
                        self.scan_filter.append(msg.ranges[i])

            if min(self.scan_filter) < 0.4:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self._cmd_pub.publish(self.twist)
                rospy.loginfo('Stop!')

            else:
                self.twist.linear.x = 0.05
                self.twist.angular.z = 0.0
                rospy.loginfo('distance of the obstacle : %f', min(self.scan_filter))

            self._cmd_pub.publish(self.twist)

def main():
    rospy.init_node('turtlebot3_obstacle')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
