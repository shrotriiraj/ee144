#!/usr/bin/env python

from math import pi, sqrt, atan2, cos, sin
#import numpy as np
#import rospy
#import tf
import operator
#from std_msgs.msg import Empty
#from nav_msgs.msg import Odometry
#from geometry_msgs.msg import Twist, Pose2D


def neighbors(current):
    # define the list of 4 neighbors
    neighbors = [(1,0),(-1,0),(0,1),(0,-1)]
    return [ (current[0]+nbr[0], current[1]+nbr[1]) for nbr in neighbors ]

def heuristic_distance(candidate, goal):
    #solved through euclidean dist sqrt
    dx = abs(candidate[0] - goal[0])
    dy = abs(candidate[1] - goal[1])
    return  dx + dy

    #solved through manhattan dist
    #dx = abs(candiate.x - goal.x)
    #dy = abs(candiate.y - goal.y)
    #return (dx + dy)
	
def path_construction(parent, goal):
    #path constrction from goal throgh parent list, no start
    path = parent(goal)
    #path = 
    return path#.reverse()



def get_path_from_A_star(start, goal, obstacles):
    # input  start: integer 2-tuple of the current grid, e.g., (0, 0)
    #        goal: integer 2-tuple  of the goal grid, e.g., (5, 1)
    #        obstacles: a list of grids marked as obstacles, e.g., [(2, -1), (2, 0), ...]
    # output path: a list of grids connecting start to goal, e.g., [(1, 0), (1, 1), ...]
    #   note that the path should contain the goal but not the start
    #   e.g., the path from (0, 0) to (2, 2) should be [(1, 0), (1, 1), (2, 1), (2, 2)] 
    cost_list = []
    path = []

    #open_list ->set of nodes to be evaluated
    open_list = [(0,start)]		    
    #print(open_list)

    #closed_list ->set of nodes already evalutated
    closed_list = []	   

    #cost_list
    past_cost = {}
    past_cost[start] = 0

    #parent_list
    parent = {}
    parent[start] = None

    #check if you have in open list, if yes feed it to closed list?
    #while OPEN is not empty do 
    while open_list:
	open_list.sort()
        current = open_list.pop(0)[1]
	closed_list.append(current)
	#print("current " + str(current))
        nbr_list = neighbors(current)
	#print(str(nbr_list))
	
	if current == goal:
 	    break
	for nbr in nbr_list:
	    if nbr in obstacles: 
                continue
            
	    #if nbr in closed_list:
		#continue

	    tentative_past_cost = past_cost[current] + 1
	   # print(past_cost[nbr])  or
	    #print(nbr not in past_cost)

	    if  nbr not in past_cost or tentative_past_cost < past_cost[nbr]:
		past_cost[nbr] = tentative_past_cost
		parent[nbr] =current
		new_cost = past_cost[nbr] + heuristic_distance(current,goal)
		open_list.append((new_cost, nbr))

    while current != start:
	path.append(current)
	current = parent[current]
	
    path.reverse()
    return path

