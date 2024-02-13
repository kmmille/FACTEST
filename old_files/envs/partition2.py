# Partitioning Scenario
# Written by: Kristina Miller

import numpy as np
from math import *

def problem():
	A_rect = np.array([[-1,0],
					   [1,0],
					   [0,-1],
					   [0,1]])

	b1 = np.array([[-1.55], [1.9], [-0], [2.0]])
	b2 = np.array([[-1], [2.5], [-2.6], [3.0]])
	b3 = np.array([[0], [4.0], [.1], [0]])
	b4 = np.array([[0], [4.0], [-4.0], [4.1]])
	b5 = np.array([[-4.0], [4.1], [0], [4.0]])
	b6 = np.array([[.1], [0], [0], [4.0]])

	obstacles = [(A_rect, b1),
				 (A_rect, b2),
				 (A_rect, b3),
				 (A_rect, b4),
				 (A_rect, b5),
				 (A_rect, b6)]

	b0 = np.array([[-0.5], [1.5], [-1.5], [2.5]])
	Theta = (A_rect, b0)

	bf = np.array([[-3], [3.675], [-3], [3.675]])
	goal = (A_rect, bf)

	return obstacles, Theta, goal
