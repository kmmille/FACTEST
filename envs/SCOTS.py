# SCOTS scenario
# Written by: Kristina Miller

import numpy as np

def problem():
	A_rect = np.array([[-1,0],
					   [1,0],
					   [0,-1],
					   [0,1]])

	b1 = np.array([[-0.9], [1.3], [0.1], [9.1]])#*10
	b2 = np.array([[-2.1], [2.5], [-5.9], [10.1]])#*10
	b3 = np.array([[-2.1], [2.5], [0.1], [5.1]])#*10
	b4 = np.array([[-3.3], [3.7], [0.1], [9.1]])#*10
	b5 = np.array([[-4.5], [4.9], [-2.9], [10.1]])#*10
	b6 = np.array([[-5.7], [6.1], [0.1], [6.1]])#*10
	b7 = np.array([[-5.7], [6.1], [-6.9], [10.1]])#*10
	b8 = np.array([[-6.9], [7.3], [-0.9], [10.1]])#*10
	b9 = np.array([[-8.1], [8.5], [0.1], [8.7]])#*10
	b10 = np.array([[-8.5], [9.5], [-8.1], [8.7]])#*10
	b11 = np.array([[-8.5], [9.5], [-5.7], [6.3]])#*10
	b12 = np.array([[-8.5], [9.5], [-3.3], [3.9]])#*10
	b13 = np.array([[-8.9], [10.1], [-2.1], [2.7]])#*10
	b14 = np.array([[-8.9], [10.1], [-4.5], [5.1]])#*10
	b15 = np.array([[-8.9], [10.1], [-6.9], [7.5]])#*10
	b16 = np.array([[1], [0], [0], [100]])*0.1
	b17 = np.array([[-100], [101], [0], [100]])*0.1
	b18 = np.array([[0], [100], [1], [0]])*0.1
	b19 = np.array([[0], [100], [-100], [101]])*0.1

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
				 (A_rect, b19)]

	b0 = np.array([[-0.3], [0.4], [-0.3], [0.4]])#*10
	Theta = (A_rect, b0)

	bf = np.array([[-9], [9.5], [0], [0.5]])#*10
	goal = (A_rect, bf)

	return obstacles, Theta, goal
