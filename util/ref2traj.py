# Ref2Traj
# Written by: Kristina Miller

from math import *
import numpy as np
from numpy.linalg import norm

import matplotlib.pyplot as plt

def get_xref(pt1, pt2, v):
	dim = len(pt1)

	dist = norm(np.array(pt2) - np.array(pt1)) # Find the length of the line segment
	T = dist/v # Find the time it takes to travel the line segment

	t = np.arange(0, T, 0.001)

	if dim == 2:
		mx = pt2[0] - pt1[0]
		bx = pt1[0]

		my = pt2[1] - pt1[1]
		by = pt1[1]

		theta = np.arctan2((np.array(pt2) - np.array(pt1))[1], (np.array(pt2) - np.array(pt1))[0])

		qref = [] # append the states to this list
		for time in t:
			xref = mx*(time/T) + bx
			yref = my*(time/T) + by
			qref.append([xref, yref, theta])

	elif dim == 3:
		mx = pt2[0] - pt1[0]
		bx = pt1[0]

		my = pt2[1] - pt1[1]
		by = pt1[1]

		mz = pt2[2] - pt1[2]
		bz = pt1[2]

		roll  = 0
		pitch = np.arctan2((np.array(pt2) - np.array(pt1))[2], norm(np.array(pt2)[0:2] - np.array(pt1)[0:2]))
		yaw = np.arctan2((np.array(pt2) - np.array(pt1))[1], (np.array(pt2) - np.array(pt1))[0])

		qref = []
		for time in t:
			xref = mx*(time/T) + bx
			yref = my*(time/T) + by
			zref = mz*(time/T) + bz
			qref.append([xref, yref, zref, roll, pitch, yaw])

	else:
		print('Nodes not in workspace!')

	return t, qref

def get_uref(v, t, dim):
	if dim == 2:
		uref = []
		for time in t:
			vref = v
			wref = 0
			uref.append([vref, wref])

	elif dim == 3:
		uref = []
		# uref for 3d is given by [vref, [wxref, wyref, wzref]]
		for time in t:
			vref = v
			wxref = 0
			wyref = 0
			wzref = 0
			uref.append([vref, [wxref, wyref, wzref]])

	else:
		print('Nodes not in workspace!')

	return uref

def ref2traj(nodes, v):
	dim = len(nodes[0])
	ref_controller = []

	for j in range(len(nodes)-1):
		pt1 = nodes[j]
		pt2 = nodes[j+1]

		t, qref = get_xref(pt1, pt2, v)
		uref = get_uref(v, t, dim)

		ref_controller.append([t, qref, uref])

	return ref_controller

if __name__ == '__main__':
	nodes = [[0,0], [1,1], [1,2], [2,3]]
	v = 1
	segs = ref2traj(nodes, v)

	print(segs[0])
	for seg in segs:
		x = [seg[1][i][0] for i in range(len(seg[1]))]
		y = [seg[1][i][1] for i in range(len(seg[1]))]

		plt.plot(x, y)

	plt.show()


