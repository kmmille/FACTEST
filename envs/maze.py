# Simple 2D Maze
# Written by: Kristina Miller

import numpy as np
from math import *

def problem():
	A_rect = np.array([[-1,0],
					   [1,0],
					   [0,-1],
					   [0,1]])

	b1 = np.array([[-10], [30], [-9], [10]])*0.1
	b2 = np.array([[-10], [50], [-39], [40]])*0.1
	b3 = np.array([[-9], [10], [-20], [30]])*0.1
	b4 = np.array([[-10], [20], [-29], [30]])*0.1
	b5 = np.array([[-19], [20], [-20], [30]])*0.1
	b6 = np.array([[-20], [40], [-19], [20]])*0.1
	b7 = np.array([[-39], [40], [-10], [30]])*0.1
	b8 = np.array([[-29], [30], [-30], [40]])*0.1
	b9 = np.array([[-40], [60], [-9], [10]])*0.1
	b10 = np.array([[-49], [50], [-20], [40]])*0.1
	b11 = np.array([[-50], [60], [-19], [20]])*0.1
	b13 = np.array([[-49], [50], [0], [10]])*0.1
	b12 = np.array([[-59], [60], [-20], [50]])*0.1
	b14 = np.array([[0], [70], [0], [1]])*0.1
	b15 = np.array([[0], [1], [0], [30]])*0.1
	b16 = np.array([[0], [1], [-40], [50]])*0.1
	b17 = np.array([[0], [60], [-49], [50]])*0.1
	b18 = np.array([[-69], [70], [0], [50]])*0.1
	b19 = np.array([[1], [0], [0], [50]])*0.1
	b20 = np.array([[-70], [71], [0], [50]])*0.1
	b21 = np.array([[0], [70], [1], [0]])*0.1
	b22 = np.array([[0], [70], [-50], [51]])*0.1

	# b = np.array([[0], [10], [0], [10]])

	obstacles = [(A_rect, b1),
				 (A_rect, b2),
				 (A_rect, b3),
				 (A_rect, b4),
				 (A_rect, b5),
				 (A_rect, b6),
				 (A_rect, b7),
				 (A_rect, b8),
				 (A_rect, b9),
				 (A_rect, b10),
				 (A_rect, b11),
				 (A_rect, b12),
				 (A_rect, b13),
				 (A_rect, b14),
				 (A_rect, b15),
				 (A_rect, b16),
				 (A_rect, b17),
				 (A_rect, b18),
				 (A_rect, b19),
				 (A_rect, b20),
				 (A_rect, b21),
				 (A_rect, b22)]
	# obstacles = [(A_rect, b)]

	b0 = np.array([[-(.5 - 0.2/sqrt(2))], [.5 + 0.2/sqrt(2)], [-(3.5 - 0.2/sqrt(2))], [3.5 + 0.2/sqrt(2)]])
	Theta = (A_rect, b0)

	bf = np.array([[-62.5], [67.5], [-45], [50]])*0.1
	goal = (A_rect, bf)


	return obstacles, Theta, goal