# Run all FACTEST files
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
parser.add_argument("--plot")
args = parser.parse_args()

def main_algo(scenario, model, env, max_segs, bloating):
	if model == 'car' or model == 'robot':
		dim = 2
	else:
		dim = 3
	obs, Theta, goal = scenario() # Get the problem

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
			if dim == 2:
				theta_list.extend(shrink_Theta(theta))
			else:
				theta_list.extend(shrink_Theta3d(theta))

	final_controller = []
	for theta, nodes in final_theta:
		for node in nodes: # Convert to usable form
			for i in range(len(node)):
				node[i] = float(node[i])

		final_controller.append((theta, ref2traj(nodes, 1)))
	te = time.time()

	data_row = [model, env, max_lyapunov(Theta), len(obs), te-ts, len(final_controller)]
	return final_controller, data_row

# Plot the problem and the test runs
def plot_results(final_controller, scenario, model, env, run_model, plot_problem = None):
	obs, Theta, goal = scenario()
	if model == 'car' or model == 'robot':
		plt.figure()
		for Ao, bo in obs:
			ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(Ao, bo), color = 'r')
		ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(goal[0], goal[1]), color = 'g')

		for theta, segs in final_controller:
			ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(theta[0], theta[1]), color = 'b')
			if model == 'car':
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
			if model == 'hovercraft':
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

	if env == 'SCOTS':
		plt.xlim(0,10)
		plt.ylim(0,10)
	elif 'zigzag' in env:
		plt.xlim(-1.5,5)
		plt.ylim(0,3)
	elif env == 'maze':
		plt.xlim(0,7)
		plt.ylim(0,5)
	elif env == 'barrier':
		plt.xlim(0,4)
		plt.ylim(0,4)

	plt.title(model + '-' + env)
	plt.savefig('results/figures/'+model+'_'+env+'.png')
	print('Saved figure to results/figures/'+model+'_'+env+'.png')
	return None

if __name__ == '__main__':
	import envs.zigzag as zig1
	import envs.zigzag2 as zig2
	import envs.zigzag3 as zig3
	import envs.maze as maze
	import envs.partition2 as barrier
	import envs.SCOTS as SCOTS
	import envs.ztunnel as z
	import envs.Ltunnel as L

	import models.auv_6d as auv
	import models.bijective_robot as robot
	import models.kinematic_car as car
	import models.kinematic_model_3d as hover

	print('Running zigzag1 with car model...')
	t1 = time.time()
	control_z1_car, z1_car_data = main_algo(zig1.problem, 'car', 'zigzag1', 10, car.bloating)
	print('time: ', time.time()-t1)

	print('Running zigzag2 with car model...')
	t1 = time.time()
	control_z2_car, z2_car_data = main_algo(zig2.problem, 'car', 'zigzag2', 10, car.bloating)
	print('time: ', time.time()-t1)

	print('Running zigzag3 with car model...')
	t1 = time.time()
	control_z3_car, z3_car_data = main_algo(zig3.problem, 'car', 'zigzag3', 10, car.bloating)
	print('time: ', time.time()-t1)

	print('Running maze with car model...')
	t1 = time.time()
	control_maze_car, maze_car_data = main_algo(maze.problem, 'car', 'maze', 10, car.bloating)
	print('time: ', time.time()-t1)

	print('Running barrier with car model...')
	t1 = time.time()
	control_bar_car, bar_car_data = main_algo(barrier.problem, 'car', 'barrier', 10, car.bloating)
	print('time: ', time.time()-t1)

	print('Running SCOTS with car model...')
	t1 = time.time()
	control_scots_car, scots_car_data = main_algo(SCOTS.problem, 'car', 'SCOTS', 100, car.bloating)
	print('time: ', time.time()-t1)
###########################################################################################################
	print('Running zigzag1 with robot model...')
	t1 = time.time()
	control_z1_rob, z1_rob_data = main_algo(zig1.problem, 'robot', 'zigzag1', 10, robot.bloating)
	print('time: ', time.time()-t1)

	print('Running zigzag2 with robot model...')
	t1 = time.time()
	control_z2_rob, z2_rob_data = main_algo(zig2.problem, 'robot', 'zigzag2', 10, robot.bloating)
	print('time: ', time.time()-t1)

	print('Running zigzag3 with robot model...')
	t1 = time.time()
	control_z3_rob, z3_rob_data = main_algo(zig3.problem, 'robot', 'zigzag3', 10, robot.bloating)
	print('time: ', time.time()-t1)

	print('Running maze with robot model...')
	t1 = time.time()
	control_maze_rob, maze_rob_data = main_algo(maze.problem, 'robot', 'maze', 10, robot.bloating)
	print('time: ', time.time()-t1)

	print('Running barrier with robot model...')
	t1 = time.time()
	control_bar_rob, bar_rob_data = main_algo(barrier.problem, 'robot', 'barrier', 10, robot.bloating)
	print('time: ', time.time()-t1)

	print('Running SCOTS with robot model...')
	t1 = time.time()
	control_scots_rob, scots_rob_data = main_algo(SCOTS.problem, 'robot', 'SCOTS', 100, robot.bloating)
	print('time: ', time.time()-t1)
###########################################################################################################
	print('Running ztunnel with auv model...')
	t1 = time.time()
	control_z_auv, z_auv_data = main_algo(z.problem, 'auv', 'ztunnel', 10, auv.bloating)
	print('time: ', time.time()-t1)

	print('Running ltunnel with auv model...')
	t1 = time.time()
	control_l_auv, l_auv_data = main_algo(L.problem, 'auv', 'ltunnel', 10, auv.bloating)
	print('time: ', time.time()-t1)
###########################################################################################################
	print('Running ztunnel with hovercraft model...')
	t1 = time.time()
	control_z_hover, z_hover_data = main_algo(z.problem, 'hovercraft', 'ztunnel', 10, hover.bloating)
	print('time: ', time.time()-t1)

	print('Running ltunnel with hovercraft model...')
	t1 = time.time()
	control_l_hover, l_hover_data = main_algo(L.problem, 'hovercraft', 'ltunnel', 10, hover.bloating)
	print('time: ', time.time()-t1)
############################################################################################################
	# Save the data
	with open('results/synthesis-results/all_data.csv', 'w') as file:
		writer = csv.writer(file)
		writer.writerows([['model', 'env', 'Theta size', 'num obs', 'time', 'num parts'],
						   z1_car_data, z2_car_data, z3_car_data, maze_car_data, bar_car_data, scots_car_data,
						   z1_rob_data, z2_rob_data, z3_rob_data, maze_rob_data, bar_rob_data, scots_rob_data,
						   z_auv_data, l_auv_data, z_hover_data, l_hover_data])
	print('Saved results to results/synthesis-results/all_data.csv')
############################################################################################################
	if args.plot != None and args.plot == 'True':
		print('Plotting zigzag1 with car model')
		plot_results(control_z1_car, zig1.problem, 'car', 'zigzag1', car.run_model, plot_problem = None)

		print('Plotting zigzag2 with car model')
		plot_results(control_z2_car, zig2.problem, 'car', 'zigzag2', car.run_model, plot_problem = None)

		print('Plotting zigzag3 with car model')
		plot_results(control_z3_car, zig3.problem, 'car', 'zigzag3', car.run_model, plot_problem = None)

		print('Plotting maze with car model')
		plot_results(control_maze_car, maze.problem, 'car', 'maze', car.run_model, plot_problem = None)

		print('Plotting barrier with car model')
		plot_results(control_bar_car, barrier.problem, 'car', 'barrier', car.run_model, plot_problem = None)

		print('Plotting SCOTS with car model')
		plot_results(control_scots_car, SCOTS.problem, 'car', 'SCOTS', car.run_model, plot_problem = None)
##############################################################################################################
		print('Plotting zigzag1 with robot model')
		plot_results(control_z1_rob, zig1.problem, 'robot', 'zigzag1', robot.run_model, plot_problem = None)

		print('Plotting zigzag2 with robot model')
		plot_results(control_z2_rob, zig2.problem, 'robot', 'zigzag2', robot.run_model, plot_problem = None)

		print('Plotting zigzag3 with robot model')
		plot_results(control_z3_rob, zig3.problem, 'robot', 'zigzag3', robot.run_model, plot_problem = None)

		print('Plotting maze with robot model')
		plot_results(control_maze_rob, maze.problem, 'robot', 'maze', robot.run_model, plot_problem = None)

		print('Plotting barrier with robot model')
		plot_results(control_bar_rob, barrier.problem, 'robot', 'barrier', robot.run_model, plot_problem = None)

		print('Plotting SCOTS with robot model')
		plot_results(control_scots_rob, SCOTS.problem, 'robot', 'SCOTS', robot.run_model, plot_problem = None)
###############################################################################################################
		print('Plotting ztunnel with auv model')
		plot_results(control_z_auv, z.problem, 'auv', 'ztunnel', auv.run_model, plot_problem = z.plot_problem)

		print('Plotting ltunnel with auv model')
		plot_results(control_l_auv, L.problem, 'auv', 'ltunnel', auv.run_model, plot_problem = L.plot_problem)
###############################################################################################################
		print('Plotting ztunnel with hovercraft model')
		plot_results(control_z_hover, z.problem, 'hovercraft', 'ztunnel', hover.run_model, plot_problem = z.plot_problem)

		print('Plotting ltunnel with hovercraft model')
		plot_results(control_l_hover, L.problem, 'hovercraft', 'ltunnel', hover.run_model, plot_problem = L.plot_problem)

		plt.show()
