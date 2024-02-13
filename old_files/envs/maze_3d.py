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

	b01 = np.array([[0], [45], [5], [10], [10], [-5]])
	b02 = np.array([[5], [0], [5], [10], [10], [30]])
	b03 = np.array([[-45], [50], [5], [10], [10], [30]])
	b04 = np.array([[0], [25], [5], [10], [5], [0]])
	b05 = np.array([[-30], [35], [5], [10], [0], [10]])
	b06 = np.array([[-25], [30], [0], [10], [0], [10]])
	b07 = np.array([[-35], [45], [5], [0], [0], [10]])
	b08 = np.array([[-30], [45], [5], [5], [-10], [25]])
	b09 = np.array([[-0], [45], [-5], [10], [-20], [25]])
	b10 = np.array([[-0], [45], [5], [0], [-20], [25]])
	b11 = np.array([[-15], [25], [5], [10], [0], [20]])
	b12 = np.array([[0], [10], [0], [5], [-5], [25]])
	b13 = np.array([[0], [15], [5], [0], [0], [20]])
	b14 = np.array([[0], [15], [-5], [10], [0], [20]])
	b15 = np.array([[0], [45], [5], [10], [-25], [30]])
	b16 = np.array([[5], [50], [10], [-5], [10], [30]])
	b17 = np.array([[5], [50], [-10], [15], [10], [30]])

	obstacles = [(A_cube, b01),
				 (A_cube, b02),
				 (A_cube, b03),
				 (A_cube, b04),
				 (A_cube, b05),
				 (A_cube, b06),
				 (A_cube, b07),
				 (A_cube, b08),
 				 (A_cube, b09),
 				 (A_cube, b10),
 				 (A_cube, b11),
 				 (A_cube, b12),
 				 (A_cube, b13),
 				 (A_cube, b14),
 				 (A_cube, b15),
				 (A_cube, b16),
 				 (A_cube, b17)]

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
	b4 = np.array([[-25], [30], [5], [10], [-10], [20]])
	b5 = np.array([[-25], [30], [5], [0], [5], [10]])
	b6 = np.array([[-30], [45], [-5], [10], [-10], [20]])
	b7 = np.array([[-30], [45], [5], [10], [5], [0]])
	b8 = np.array([[-35], [45], [0], [10], [0], [10]])

	b01 = np.array([[0], [45], [5], [10], [10], [-5]])
	b02 = np.array([[5], [0], [5], [10], [10], [30]])
	b03 = np.array([[-45], [50], [5], [10], [10], [30]])
	b04 = np.array([[0], [25], [5], [10], [5], [0]])
	b05 = np.array([[-30], [35], [5], [10], [0], [10]])
	b06 = np.array([[-25], [30], [0], [10], [0], [10]])
	b07 = np.array([[-35], [45], [5], [0], [0], [10]])
	b08 = np.array([[-30], [45], [5], [5], [-10], [25]])
	b09 = np.array([[-0], [45], [-5], [10], [-20], [25]])
	b10 = np.array([[-0], [45], [5], [0], [-20], [25]])
	b11 = np.array([[-15], [25], [5], [10], [0], [20]])
	b12 = np.array([[0], [10], [0], [5], [-5], [25]])
	b13 = np.array([[0], [15], [5], [0], [0], [20]])
	b14 = np.array([[0], [15], [-5], [10], [0], [20]])
	b15 = np.array([[0], [45], [5], [10], [-25], [30]])
	b16 = np.array([[5], [50], [10], [-5], [10], [30]])
	b17 = np.array([[5], [50], [-10], [15], [10], [30]])

	if axes == None:
		axes = a3.Axes3D(plt.figure())

	plot_polytope_3d(A_cube, b1, ax = axes, color = 'yellow', trans = 0.1)
	plot_polytope_3d(A_cube, b2, ax = axes, color = 'yellow', trans = 0.1)
	plot_polytope_3d(A_cube, b3, ax = axes, color = 'yellow', trans = 0.1)
	plot_polytope_3d(A_cube, b4, ax = axes, color = 'yellow', trans = 0.1)
	plot_polytope_3d(A_cube, b5, ax = axes, color = 'yellow', trans = 0.1)
	plot_polytope_3d(A_cube, b6, ax = axes, color = 'yellow', trans = 0.1)
	plot_polytope_3d(A_cube, b7, ax = axes, color = 'yellow', trans = 0.1)
	plot_polytope_3d(A_cube, b8, ax = axes, color = 'yellow', trans = 0.1)

	# plot_polytope_3d(A_cube, b01, ax = axes, color = 'red', trans = 0.1)
	# plot_polytope_3d(A_cube, b02, ax = axes, color = 'red', trans = 0.1)
	# plot_polytope_3d(A_cube, b03, ax = axes, color = 'red', trans = 0.1)
	# plot_polytope_3d(A_cube, b04, ax = axes, color = 'red', trans = 0.1)
	# plot_polytope_3d(A_cube, b05, ax = axes, color = 'red', trans = 0.1)
	# plot_polytope_3d(A_cube, b06, ax = axes, color = 'red', trans = 0.1)
	# plot_polytope_3d(A_cube, b07, ax = axes, color = 'red', trans = 0.1)
	# plot_polytope_3d(A_cube, b08, ax = axes, color = 'red', trans = 0.1)
	# plot_polytope_3d(A_cube, b09, ax = axes, color = 'red', trans = 0.1)
	# plot_polytope_3d(A_cube, b10, ax = axes, color = 'red', trans = 0.1)
	# plot_polytope_3d(A_cube, b11, ax = axes, color = 'red', trans = 0.1)
	# plot_polytope_3d(A_cube, b12, ax = axes, color = 'red', trans = 0.1)
	# plot_polytope_3d(A_cube, b13, ax = axes, color = 'red', trans = 0.1)
	# plot_polytope_3d(A_cube, b14, ax = axes, color = 'red', trans = 0.1)
	# plot_polytope_3d(A_cube, b15, ax = axes, color = 'red', trans = 0.1)
	# plot_polytope_3d(A_cube, b16, ax = axes, color = 'red', trans = 0.1)
	# plot_polytope_3d(A_cube, b17, ax = axes, color = 'red', trans = 0.1)

	plot_polytope_3d(A_cube, Theta[1], ax = axes, color = 'blue', trans = 0.9)
	plot_polytope_3d(A_cube, Goal[1], ax = axes, color = 'green', trans = 0.9)

	axes.set_xlabel('x')
	axes.set_ylabel('y')
	axes.set_zlabel('z')

	return axes

if __name__ == '__main__':
	plot_problem()
	plt.show()
