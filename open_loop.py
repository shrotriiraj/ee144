#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import pi

#declaring variables for range and velocity 
v = 0.1
dist = 400
turn = 157
cor = 0.0025

class Turtlebot():
    def __init__(self):
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.run()


    def run(self):
	#first cycle
        vel = Twist()
        vel.linear.x = v
        vel.angular.z = cor
        for i in range(dist):
            self.vel_pub.publish(vel)
            self.rate.sleep()
        vel.linear.x = 0
        vel.angular.z = v
        for i in range(turn):
            self.vel_pub.publish(vel)
            self.rate.sleep()

	#second cycle
        vel.linear.x = v
        vel.angular.z = cor
        for i in range(dist):
            self.vel_pub.publish(vel)
            self.rate.sleep()
        vel.linear.x = 0
        vel.angular.z = v
        for i in range(turn):
            self.vel_pub.publish(vel)
            self.rate.sleep()

	#third cycle
        vel.linear.x = v
        vel.angular.z = cor
        for i in range(dist):
            self.vel_pub.publish(vel)
            self.rate.sleep()
        vel.linear.x = 0
        vel.angular.z = v
        for i in range(turn):
            self.vel_pub.publish(vel)
            self.rate.sleep()

	#fourth cycle
        vel.linear.x = v
        vel.angular.z = cor
        for i in range(dist):
            self.vel_pub.publish(vel)
            self.rate.sleep()


	"""
        vel.linear.x = 0
        vel.angular.z = 0.05
        #while not rospy.is_shutdown():  # uncomment to use while loop
        for i in range(50):
            self.vel_pub.publish(vel)
            self.rate.sleep()
        """


if __name__ == '__main__':
    try:
        whatever = Turtlebot()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")
