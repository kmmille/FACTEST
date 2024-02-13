# Z-tunnel environment
# Written by: Kristina Miller

import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from util.plot_polytope3d import *
import numpy as np

def problem():
	A_cube = np.array([[-1, 0, 0],
					   [1, 0, 0],
					   [0, -1, 0],
					   [0, 1, 0],
					   [0, 0, -1],
					   [0, 0, 1]])
	
	# b1 = np.array([[-30], [50], [0], [45], [0], [50]])
	# b2 = np.array([[0], [50], [-45], [50], [0], [45]])
	# b3 = np.array([[0], [30], [-5], [45], [0], [45]])
	# b4 = np.array([[0], [25], [0], [5], [-5], [50]])
	# b5 = np.array([[0], [25],[-5], [50], [-45], [50]])
	# b6 = np.array([[0], [30],[-5], [50], [1], [0]])
	b1 = np.array([[0], [50], [0], [50], [1], [0]])
	b2 = np.array([[-30], [50], [0], [45], [1], [50]])
	b3 = np.array([[0], [50], [-45], [50], [1], [45]])
	b4 = np.array([[0], [25], [0], [50], [-45], [50]])
	b5 = np.array([[0], [30], [-5], [45], [0], [45]])
	b6 = np.array([[0], [25], [0], [5], [-5], [45]])
	obstacles = [(A_cube, b1),
				 (A_cube, b2),
				 (A_cube, b3),
				 (A_cube, b4),
				 (A_cube, b5),
				 (A_cube, b6)]

	b0 = np.array([[-0.5], [1.5], [-2], [3], [-2], [3]])
	Theta = (A_cube, b0)

	bf = np.array([[-48], [50], [-46.5], [48.5], [-46.5], [48.5]])
	Goal = (A_cube, bf)

	return obstacles, Theta, Goal

def plot_problem():
	fig = plt.figure()
	axes = fig.add_subplot(111, projection='3d')

	obstacles, Theta, Goal= problem()

	A_cube = np.array([[-1, 0, 0],
					   [1, 0, 0],
					   [0, -1, 0],
					   [0, 1, 0],
					   [0, 0, -1],
					   [0, 0, 1]])
	
	b1 = np.array([[0], [25], [0], [5], [0], [5]])
	b2 = np.array([[-25], [30], [0], [5], [0], [45]])
	b3 = np.array([[-25], [30], [0], [50], [-45], [50]])
	b4 = np.array([[-30], [50], [-45], [50], [-45], [50]])

	plot_polytope_3d(A_cube, b1, ax = axes, color = 'yellow', trans = 0.1)
	plot_polytope_3d(A_cube, b2, ax = axes, color = 'yellow', trans = 0.1)
	plot_polytope_3d(A_cube, b3, ax = axes, color = 'yellow', trans = 0.1)
	plot_polytope_3d(A_cube, b4, ax = axes, color = 'yellow', trans = 0.1)
	
	plot_polytope_3d(A_cube, Theta[1], ax = axes, color = 'blue', trans = 0.9)
	plot_polytope_3d(A_cube, Goal[1], ax = axes, color = 'green', trans = 0.9)

	axes.set_xlabel('x')
	axes.set_ylabel('y')
	axes.set_zlabel('z')

	return axes

if __name__ == '__main__':
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	plot_problem(ax)

	plt.show()