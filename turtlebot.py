#!/usr/bin/env python

from math import pi, sqrt, atan2, cos, sin
import operator
import numpy as np
import matplotlib.pyplot as plt

import rospy
import tf
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D

def neighbors(current):
    # define the list of 4 neighbors
    neighbors = [(0.5,0),(-0.5,0),(0,0.5),(0,-0.5)]
    return [ (current[0]+nbr[0], current[1]+nbr[1]) for nbr in neighbors ]

def heuristic_distance(candidate, goal):
    #solved through manhattan dist
    dx = abs(candidate[0] - goal[0])
    dy = abs(candidate[1] - goal[1])
    return  dx + dy
	
def path_construction(parent, goal):
    #path constrction from goal throgh parent list, no start
    path = parent(goal)
    while current != start:
        path.append(current)
        current = parent[current]
    path.reverse()
    return path

def get_path_from_A_star(start, goal, obstacles):
    # input  start: integer 2-tuple of the current grid, e.g., (0, 0)
    #        goal: integer 2-tuple  of the goal grid, e.g., (5, 1)
    #        obstacles: a list of grids marked as obstacles, e.g., [(2, -1), (2, 0), ...]
    # output path: a list of grids connecting start to goal, e.g., [(1, 0), (1, 1), ...]
    #   note that the path should contain the goal but not the start
    #   e.g., the path from (0, 0) to (2, 2) should be [(1, 0), (1, 1), (2, 1), (2, 2)] 

    #path ->final list of coords for pathing
    path = []
    #open_list ->set of nodes to be evaluated
    open_list = [(0,start)]		    
    #closed_list ->set of nodes already evalutated
    closed_list = []	   

    #past_cost
    past_cost = {}
    past_cost[start] = 0

    #parent_list
    parent = {}
    parent[start] = None

    #while OPEN is not empty do 
    while open_list:
        open_list.sort()
        current = open_list.pop(0)[1]
        closed_list.append(current)
	    #get neighbor points
        nbr_list = neighbors(current)
	
	    #if point is goal, break
        if current == goal:
            break
        for nbr in nbr_list:
	        #if point is in the obstacles list skip point
            if nbr in obstacles: 
                continue
            #if point is in the closed_list(we've already checked) skip point
            if nbr in closed_list:
                continue
            tentative_past_cost = past_cost[current] + 1
	        #if point is not in past_cost or it's the lowest option 
	        #update past cost save point
            if  nbr not in past_cost or tentative_past_cost < past_cost[nbr]:
                past_cost[nbr] = tentative_past_cost
                parent[nbr] =current
                new_cost = past_cost[nbr] + heuristic_distance(current,goal)
                open_list.append((new_cost, nbr))

    #path construction
    while current != start:
        path.append(current)
        current = parent[current]
	
    path.reverse()
    rospy.loginfo("path = ")
    rospy.loginfo(path)
    return path


class Turtlebot():

    def __init__(self):
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi",
                                       Twist,
                                       queue_size=10)
        self.rate = rospy.Rate(10)
        self.vel = Twist()
        self.previous_point = [0, 0]
        self.previous_velocity = [0, 0]
        self.vel_steering = 0.25

        # reset odometry to zero
        self.reset_pub = rospy.Publisher("mobile_base/commands/reset_odometry",
                                         Empty,
                                         queue_size=10)
        for i in range(10):
            self.reset_pub.publish(Empty())
            self.rate.sleep()

        # subscribe to odometry
        self.pose = Pose2D()
        self.logging_counter = 0
        self.trajectory = list()
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

        try:
            self.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
        finally:
            # save trajectory into csv file
            np.savetxt('trajectory.csv',
                       np.array(self.trajectory),
                       fmt='%f',
                       delimiter=',')

    def run(self):
        waypoints = get_path_from_A_star((0, 0), (4.5, 0), [(1, 0), (1, -0.5), (2, 1), (2,1.5), (3.5, 0), (3.5, 0.5)])
        #waypoints = [[0.5, 0], [0.5, -0.5], [1, -0.5], [1, 0], [1, 0.5],\
                   # [1.5, 0.5], [1.5, 0], [1.5, -0.5], [1, -0.5], [1, 0],\
     #[1, 0.5], [0.5, 0.5], [0.5, 0], [0, 0], [0, 0]]
	last = waypoints[(len(waypoints) -1)]
	waypoints.append(last)
        for i in range(len(waypoints) - 1):
            self.move_to_point(waypoints[i], waypoints[i + 1])
            
    def polynomial_time_scaling_3rd_order(self, p_start, v_start, p_end, v_end,
                                          T):
        # input: p,v: position and velocity of start/end point
        #        T: the desired time to complete this segment of trajectory (in second)
        # output: the coefficients of this polynomial
        A = np.array([p_start, p_end, v_start, v_end])
        X = np.array([[0, 0, 0, 1], [T**3, T**2, T, 1], [0, 0, 1, 0],
                     [3 * (T**2), 2 * T, 1, 0]])
        return np.dot(np.linalg.inv(X), A)  # return np.dot(inv(x),A)

    def move_to_point(self, current_waypoint, next_waypoint):
        # generate polynomial trajectory and move to current_waypoint
        # next_waypoint is to help determine the velocity to pass current_waypoint
        #Set boundary conditions for position
        #for x
        Pxstart = self.previous_point[0]
        Pxend = current_waypoint[0]

        #for y
        Pystart = self.previous_point[1]
        Pyend = current_waypoint[1]

        #Set boundary conditions for velocity
        #for x
        Vxstart = self.previous_velocity[0]

        #for y
        Vystart = self.previous_velocity[1]

        #Vxend, Vyend
        #need deviation atan(delta x, delta y) (arctan)
        #decompose V in Vx,Vy

        deltax = Pxend - Pxstart
        deltay = Pyend - Pystart
        theta = atan2(deltay, deltax)


        Vxend = self.vel_steering * cos(theta)
        Vyend = self.vel_steering * sin(theta)
        
        T = 1
        #compute coefficient x
        c_x = self.polynomial_time_scaling_3rd_order(Pxstart, Vxstart, Pxend, Vxend,T)
        #compute coefficient y
        c_y = self.polynomial_time_scaling_3rd_order(Pystart, Vystart, Pyend, Vyend,T)

        c = 9 #change c and kp to fine tune
        for i in range(c * T):
            t = i * 0.1
            Vxend = np.dot(np.array([3*(t**2), 2 * t, 1, 0]), c_x)
            Vyend = np.dot(np.array([3*(t**2), 2 * t, 1, 0]), c_y)
            self.vel.linear.x = sqrt(Vxend**2 + Vyend**2)

            #compute theta deviation from Vxend, Vyend
            #the error is thetadeviation - thetacurrent
            kp = 2.5
            dtheta = atan2(Vyend, Vxend)

            if dtheta < -pi:
                dtheta = dtheta + 2*pi
            elif dtheta > pi:
                dtheta = dtheta - 2*pi
            else:
                dtheta = dtheta

            error = dtheta - self.pose.theta

            # if error < -pi:
            #     error = error + 2 * pi
            # elif error > pi:
            #    error = error - 2 * pi
            # else:
            #    error = error

            self.vel.angular.z = error * kp  #setkp
            #publish vel
            self.vel_pub.publish(self.vel)
            self.rate.sleep()

#update prev point and velocity
        self.previous_point = [current_waypoint[0], current_waypoint[1]]
        self.previous_velocity = [Vxend, Vyend]  #last part of loop

    

    def odom_callback(self, msg):
        # get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, \
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch,
         yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        # logging once every 100 times (Gazebo runs at 1000Hz; we save it at 10Hz)
        self.logging_counter += 1
        if self.logging_counter == 100:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x,
                                    self.pose.y])  # save trajectory
            rospy.loginfo("odom: x=" + str(self.pose.x) + \
            ";\t\ty=" + str(self.pose.y) + ";\t\ttheta=" + str(yaw))

if __name__ == '__main__':
    whatever = Turtlebot()
   # visualization()

    def visualization():
    # load csv file and plot trajectory 
        _, ax = plt.subplots(1)
        ax.set_aspect('equal')

        trajectory = np.loadtxt("trajectory.csv", delimiter=',')
        plt.plot(trajectory[:, 0], trajectory[:, 1], linewidth=2)

        plt.xlim(-0.2, 10)
        plt.ylim(-2, 4)
        plt.show()
    if __name__ == '__main__':
        visualization()


