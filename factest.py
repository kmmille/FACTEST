# Run benchmarks
# Written by: Kristina Miller

import argparse
import sys
import time
import csv

import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import pypoman as ppm

from util.ref2traj import *
from util.xref_yices import *
from util.partition_theta import *
from util.plot_polytope3d import *

parser = argparse.ArgumentParser()
parser.add_argument("model")
parser.add_argument("env")
parser.add_argument("--segs")
parser.add_argument("--plot")
args = parser.parse_args()

# Import the model
if args.model == 'car':
	from models.kinematic_car import *
elif args.model == 'robot':
	from models.bijective_robot import *
elif args.model == 'hovercraft':
	from models.kinematic_model_3d import *
elif args.model == 'auv':
	from models.auv_6d import *
else:
	print('Not a valid model argument!')
	sys.exit()

# Import the environment
if args.env == 'zigzag1':
	from envs.zigzag import *
	dim = 2
elif args.env == 'zigzag2':
	from envs.zigzag2 import *
	dim = 2
elif args.env == 'zigzag3':
	from envs.zigzag3 import *
	dim = 2
elif args.env == 'maze':
	from envs.maze import *
	dim = 2
elif args.env == 'SCOTS':
	from envs.SCOTS import *
	dim = 2
elif args.env == 'barrier':
	from envs.partition2 import *
	dim = 2
elif args.env == 'ztunnel':
	from envs.ztunnel import *
	dim = 3
elif args.env == 'ltunnel':
	from envs.Ltunnel import *
	dim = 3
else:
	print('Not a valid env argument!')
	sys.exit()

# Make sure the model and the environment are compatible
if args.model in ['car', 'robot'] and args.env in ['ztunnel', 'ltunnel']:
	print("Can't run 2d model in 3d workspace!")
	sys.exit()
elif args.model not in ['car', 'robot'] and args.env not in ['ztunnel', 'ltunnel']:
	print("Can't run 3d model in 2d workspace!")
	sys.exit()

# Check the number of line segments
if args.segs == None:
	max_segs = 100
else:
	try:
		max_segs = int(args.segs)
	except:
		print('segs must be an integer!')
		sys.exit()

# Main controller algorithm
def main_algo():
	obs, Theta, goal = problem() # Get the problem
	theta_list = [Theta]
	final_theta = []

	ts = time.time()
	while theta_list != []:
		theta = theta_list.pop()
		alpha = max_lyapunov(theta)
		nodes = get_xref_yices(theta, goal, obs, max_segs, bloating, alpha) # Get the reference nodes

		if nodes != None:
			final_theta.append((theta, nodes))
		else:
			if alpha > 0.1: # This is the minimum size that Theta can be (Part_min)
				if dim == 2:
					theta_list.extend(shrink_Theta(theta))
				else:
					theta_list.extend(shrink_Theta3d(theta))
			else:
				final_theta.append((theta, 'FAIL'))

	final_controller = []
	for theta, nodes in final_theta:
		if nodes != 'FAIL':
			for node in nodes: # Convert to usable form
				for i in range(len(node)):
					node[i] = float(node[i])
			final_controller.append((theta, ref2traj(nodes, 1)))

	te = time.time()

	with open('results/synthesis-results/'+args.model+'_'+args.env+'.csv', 'w') as file:
		writer = csv.writer(file)
		writer.writerows([['model', 'env', 'Theta size', 'num obs', 'time', 'num parts'],
						  [args.model, args.env, max_lyapunov(Theta), len(obs), te-ts, len(final_controller)]])

	print('Saved results to results/synthesis-results/'+args.model+'_'+args.env+'.csv')
	# final_controller contains (theta, (t, xref, uref)_seg)
	return final_controller

# Plot the problem and the test runs
def plot_results(final_controller):
	obs, Theta, goal = problem()
	if dim == 2:
		for Ao, bo in obs:
			ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(Ao, bo), color = 'r')
		ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(goal[0], goal[1]), color = 'g')

		for theta, segs in final_controller:
			if segs == 'FAIL':
				ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(theta[0], theta[1]), color = 'r')
			else:
				ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(theta[0], theta[1]), color = 'b')
				if args.model == 'car':
					q0 = randomly_sample(theta)+[0]
				else:
					q0 = randomly_sample(theta)+[0,1]
				for seg in segs:
					xref = [seg[1][i][0] for i in range(len(seg[1]))]
					yref = [seg[1][i][1] for i in range(len(seg[1]))]
					plt.plot(xref, yref, color = 'k')

					t = seg[0]
					qref = seg[1]
					uref = seg[2]

					q = run_model(q0, t, qref, uref)
					q0 = q[-1]
					x = [q[i][0] for i in range(len(q))]
					y = [q[i][1] for i in range(len(q))]

					plt.plot(x, y, 'r--')
	else:
		ax = plot_problem()
		for theta, segs in final_controller:
			if segs != 'FAIL':
				if args.model == 'hovercraft':
					q0 = [segs[0][1][0][0], segs[0][1][0][1], segs[0][1][0][2], segs[0][1][0][4]]
				else:
					q0 = segs[0][1][0]

				for seg in segs:
					xref = [seg[1][i][0] for i in range(len(seg[1]))]
					yref = [seg[1][i][1] for i in range(len(seg[1]))]
					zref = [seg[1][i][2] for i in range(len(seg[1]))]
					ax.plot(xref, yref, zref, color = 'k')

					t = seg[0]
					qref = seg[1]
					uref = seg[2]

					q = run_model(q0, t, qref, uref)
					q0 = q[-1]
					x = [q[i][0] for i in range(len(q))]
					y = [q[i][1] for i in range(len(q))]
					z = [q[i][2] for i in range(len(q))]

					plt.plot(x, y, z, 'r--')

	if args.env == 'SCOTS':
		plt.xlim(0,10)
		plt.ylim(0,10)
	elif 'zigzag' in args.env:
		plt.xlim(-1.5,5)
		plt.ylim(0,3)
	elif args.env == 'maze':
		plt.xlim(0,7)
		plt.ylim(0,5)
	elif args.env == 'barrier':
		plt.xlim(0,4)
		plt.ylim(0,4)

	plt.title(args.model + '-' + args.env)
	plt.savefig('results/figures/'+args.model+'_'+args.env+'.png')
	print('Saved figure to results/figures/'+args.model+'_'+args.env+'.png')
	return None

if __name__ == '__main__':
	final_controller = main_algo()
	if args.plot == 'True':
		plot_results(final_controller)
		plt.show()
	else:
		sys.exit()
