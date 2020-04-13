# Computations for Theta
# Written by: Kristina Miller

import numpy as np
import pypoman as ppm
from numpy.linalg import norm


def max_lyapunov(Theta):
	# Maximize the Lyapunov function over the entire start set
	# Return the max size and state
	A = Theta[0]
	b = Theta[1]

	verts = ppm.duality.compute_polytope_vertices(A, b)

	r = [[norm(vert1 - vert2) for vert1 in verts] for vert2 in verts]
	r = [max(lens) for lens in r]
	r = max(r)/2

	return r

def shrink_Theta(Theta):
	A = Theta[0]
	b = Theta[1]

	verts = ppm.duality.compute_polytope_vertices(A, b)
	x_list = [vert[0] for vert in verts]
	y_list = [vert[1] for vert in verts]
	widths = [[abs(x1 - x2) for x2 in x_list] for x1 in x_list]
	lens = [[abs(y1 - y2) for y2 in y_list] for y1 in y_list]
	rw = max(widths[0])
	rl = max(widths[1])

	b1 = np.array([[b[0][0]],[A[0][0]*b[0][0]+rw/2],
				   [-(A[2][1]*b[2][0]+rl/2)], [b[3][0]]])
	b2 = np.array([[b[0][0]],[A[0][0]*b[0][0]+rw/2],
			       [b[2][0]], [A[2][1]*b[2][0]+rl/2]])
	b3 = np.array([[-(A[0][0]*b[0][0]+rw/2)],[b[1][0]],
				   [-(A[2][1]*b[2][0]+rl/2)], [b[3][0]]])
	b4 = np.array([[-(A[0][0]*b[0][0]+rw/2)],[b[1][0]],
				   [b[2][0]], [A[2][1]*b[2][0]+rl/2]])
	return [(A, b1), (A, b2), (A, b3), (A, b4)]

def shrink_Theta3d(Theta):
	A = Theta[0]
	b = Theta[1]

	verts = ppm.duality.compute_polytope_vertices(A, b)
	x_list = [vert[0] for vert in verts]
	y_list = [vert[1] for vert in verts]
	z_list = [vert[2] for vert in verts]
	widths = [[abs(x1 - x2) for x2 in x_list] for x1 in x_list]
	lens = [[abs(y1 - y2) for y2 in y_list] for y1 in y_list]
	height = [[abs(z1 - z2) for z2 in z_list] for z1 in z_list]
	rw = max(widths[0])
	rl = max(lens[0])
	rh = max(height[0])

	print(rw, rl, rh)

	b1 = np.array([[b[0][0]], [b[0][0] + rw/2],
				   [b[2][0]], [b[2][0] + rl/2],
				   [b[4][0]], [b[4][0] + rh/2]])

	b2 = np.array([[b[0][0]], [b[0][0] + rw/2],
				   [-(b[2][0] + rl/2)], [b[3][0]],
				   [b[4][0]], [b[4][0] + rh/2]])

	b3 = np.array([[b[0][0]], [A[0][0]*b[0][0] + rw/2],
				   [-(b[2][0] + rl/2)], [b[3][0]],
				   [-(b[4][0] + rh/2)], [b[5][0]]])

	b4 = np.array([[b[0][0]], [b[0][0] + rw/2],
				   [b[2][0]], [b[2][0] + rl/2],
				   [-(b[4][0] + rh/2)], [b[5][0]]])

	b5 = np.array([[b[0][0] + rw/2], [b[1][0]],
				   [b[2][0]], [b[2][0] + rl/2],
				   [b[4][0]], [b[4][0] + rh/2]])

	b6 = np.array([[-(b[0][0] + rw/2)], [b[1][0]],
				   [-(b[2][0] + rl/2)], [b[3][0]],
				   [b[4][0]], [b[4][0] + rh/2]])

	b7 = np.array([[-(b[0][0] + rw/2)], [b[1][0]],
				   [b[2][0]], [b[2][0] + rl/2],
				   [-(b[4][0] + rh/2)], [b[5][0]]])

	b8 = np.array([[-(b[0][0] + rw/2)], [b[1][0]],
				   [-(b[2][0] + rl/2)], [b[3][0]],
				   [-(b[4][0] + rh/2)], [b[5][0]]])

	return [(A, b1), (A, b2), (A, b3), (A, b4), (A, b5), (A, b6), (A, b7), (A, b8)]

def randomly_sample(Theta):
	A = Theta[0]
	b = Theta[1]

	x_low = A[0][0]*b[0][0]
	x_high = A[1][0]*b[1][0]
	y_high = A[3][1]*b[3][0]
	y_low = A[2][1]*b[2][0]

	w = x_high - x_low
	h = y_high - y_low

	x_rand = np.random.random(1)[0]*w
	y_rand = np.random.random(1)[0]*h

	x = x_low + x_rand
	y = y_low + y_rand

	return [x, y]

def randomly_sample3d(Theta):
	A = Theta[0]
	b = Theta[1]

	x_low = A[0][0]*b[0][0]
	x_high = A[1][0]*b[1][0]
	y_high = A[3][1]*b[3][0]
	y_low = A[2][1]*b[2][0]
	z_high = A[5][2]*b[5][0]
	z_low = A[4][2]*b[4][0]

	w = x_high - x_low
	h = y_high - y_low
	l = z_high - z_low

	x_rand = np.random.random(1)[0]*w
	y_rand = np.random.random(1)[0]*h
	z_rand = np.random.random(1)[0]*l

	x = x_low + x_rand
	y = y_low + y_rand
	z = z_low + z_rand

	return [x, y, z]

if __name__ == '__main__':
	A = np.array([[-1, 0],
				  [1, 0],
				  [0, -1],
				  [0, 1]])
	b = np.array([[-1], [3], [2], [4]])
	Theta = (A, b)
