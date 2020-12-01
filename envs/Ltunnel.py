# L-tunnel environment
# Written by: Kristina Miller

import sys
sys.path.append("../")
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

	b1 = np.array([[0], [40], [0], [5], [-25], [40]])
	b2 = np.array([[0], [10], [0], [5], [-5], [25]])
	b3 = np.array([[-30], [40], [0], [5], [-5], [25]])
	b4 = np.array([[-15], [25], [0], [5], [0], [20]])
	b5 = np.array([[0], [40], [0], [5], [5], [0]])
	b6 = np.array([[0], [40], [0], [5], [-40], [45]])
	b7 = np.array([[0], [40], [-5], [10], [0], [40]])
	b8 = np.array([[0], [40], [5], [0], [0], [40]])

	obstacles = [(A_cube, b1),
				 (A_cube, b2),
				 (A_cube, b3),
				 (A_cube, b4),
				 (A_cube, b5),
				 (A_cube, b6),
				 (A_cube, b7),
				 (A_cube, b8)]

	b0 = np.array([[-0.5], [1.5], [-2], [3], [-2], [3]])
	Theta = (A_cube, b0)

	bf = np.array([[-38], [40], [-1.5], [3.5], [-1.5], [3.5]])
	Goal = (A_cube, bf)

	return obstacles, Theta, Goal

def plot_problem():
	fig = plt.figure()
	axes = fig.add_subplot(111, projection='3d')

	obstacles, Theta, Goal = problem()

	A_cube = np.array([[-1, 0, 0],
					   [1, 0, 0],
					   [0, -1, 0],
					   [0, 1, 0],
					   [0, 0, -1],
					   [0, 0, 1]])

	b1 = np.array([[0], [15], [0], [5], [0], [5]])
	b2 = np.array([[-10], [15], [0], [5], [-5], [20]])
	b3 = np.array([[-10], [30], [0], [5], [-20], [25]])
	b4 = np.array([[-25], [30], [0], [5], [-5], [20]])
	b5 = np.array([[-25], [40], [0], [5], [0], [5]])

	if axes == None:
		axes = a3.Axes3D(plt.figure())

	plot_polytope_3d(A_cube, b1, ax = axes, color = 'yellow', trans = 0.1)
	plot_polytope_3d(A_cube, b2, ax = axes, color = 'yellow', trans = 0.1)
	plot_polytope_3d(A_cube, b3, ax = axes, color = 'yellow', trans = 0.1)
	plot_polytope_3d(A_cube, b4, ax = axes, color = 'yellow', trans = 0.1)
	plot_polytope_3d(A_cube, b5, ax = axes, color = 'yellow', trans = 0.1)

	plot_polytope_3d(A_cube, Theta[1], ax = axes, color = 'blue', trans = 0.9)
	plot_polytope_3d(A_cube, Goal[1], ax = axes, color = 'green', trans = 0.9)

	axes.set_xlabel('x')
	axes.set_ylabel('y')
	axes.set_zlabel('z')

	return axes

if __name__ == '__main__':
	plot_problem()
	plt.show()
