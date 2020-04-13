# Simple 2D Zigzag
# Written by: Kristina Miller

from math import *
import numpy as np

def problem():
	A_rect = np.array([[-1,0],
					   [1,0],
					   [0,-1],
					   [0,1]])
	A_tri1 = np.array([[-1,-1],
					   [1,-1],
					   [0,1]])
	A_tri2 = np.array([[-1,1],
					   [1,1],
					   [0,-1]])

	b1 = np.array([[5], [20], [0]])*0.1
	b2 = np.array([[-10], [35], [0]])*0.1
	b3 = np.array([[-30], [0], [30]])*0.1
	b4 = np.array([[-15], [-15], [30]])*0.1
	b5 = np.array([[-45], [15], [30]])*0.1
	b6 = np.array([[15], [50], [1], [0]])*0.1
	b7 = np.array([[15], [50], [-30], [31]])*0.1
	b8 = np.array([[16], [-15], [0], [30]])*0.1
	b9 = np.array([[-50], [51], [0], [30]])*0.1

	obstacles = [(A_tri2, b1),
				 (A_tri2, b2),
				 (A_tri1, b3),
				 (A_tri1, b4),
				 (A_tri1, b5),
				 (A_rect, b6),
				 (A_rect, b7),
				 (A_rect, b8),
				 (A_rect, b9)]

	b0 = np.array([[7.5+6/sqrt(2)], [-(7.5-6/sqrt(2))], [-(7.5-6/sqrt(2))], [7.5+6/sqrt(2)]])*0.1
	Theta = (A_rect, b0)

	bf = np.array([[-40], [45], [-10], [15]])*0.1
	goal = (A_rect, bf)

	return obstacles, Theta, goal