# Simple 2D Maze
# Written by: Kristina Miller

import numpy as np
from math import *

def problem():
	A_rect = np.array([[-1,0],
					   [1,0],
					   [0,-1],
					   [0,1]])

	b0 = np.array([[-0.25], [0.75], [-0.25], [0.75]])
	Theta = (A_rect, b0)

	b1 = np.array([[-4], [5], [-4], [5]])
	goal = (A_rect, b1)

	b2 = np.array([[2], [15], [2], [0]])
	b3 = np.array([[2], [0], [0], [15]])
	b4 = np.array([[-1], [4], [-1], [14]])
	b5 = np.array([[-4], [14], [-1], [4]])
	b6 = np.array([[-5], [14], [-4], [10]])
	b7 = np.array([[-4], [14], [-11], [14]])
	obstacles = [(A_rect, b2), (A_rect, b3), (A_rect, b4),(A_rect,b5),(A_rect,b6),(A_rect,b7)]

	return obstacles, Theta, goal