# Simple 2D Maze
# Written by: Kristina Miller

import numpy as np
from math import *

def problem():
	A_rect = np.array([[-1,0],
					   [1,0],
					   [0,-1],
					   [0,1]])
	b0 = np.array([[0], [1], [0], [1]])
	Theta = (A_rect, b0)
    

	b1 = np.array([[-4], [5], [-4], [5]])
	goal = (A_rect, b1)

	b2 = np.array([[-4], [6], [2], [0]])
	b3 = np.array([[-2], [4], [0], [2]])
	b4 = np.array([[0], [2], [-2], [4]])
	obstacles = [(A_rect, b2), (A_rect, b3), (A_rect, b4)]

	return obstacles, Theta, goal