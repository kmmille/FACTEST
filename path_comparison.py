# SAT-plan vs. RRT
# Written by: Kristina Miller

import argparse
import sys
import time

import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams.update({'errorbar.capsize': 2})

from util.rrt import *
from util.xref_yices import *
from statistics import stdev

parser = argparse.ArgumentParser()
parser.add_argument("--env")
parser.add_argument("--plot")
args = parser.parse_args()

scenario = args.env
if scenario == None:
	scenario = 'all'

if scenario not in ['all', 'zigzag', 'easy', 'hard', 'barrier', 'maze', 'SCOTS']:
	print('Not a valid env name!')
	sys.exit()

def plot_comparison_all(rrt_stats, sat_stats):
	labels = ['easy', 'hard', 'zigzag', 'barrier', 'maze', 'SCOTS']

	sat_avg_time = [0, 0, 0, 0, 0, 0]
	sat_stdev_time = [0, 0, 0, 0, 0, 0]
	sat_min_time = [0, 0, 0, 0, 0, 0]
	sat_max_time = [0, 0, 0, 0, 0, 0]
	sat_avg_iter = [0, 0, 0, 0, 0, 0]
	sat_stdev_iter = [0, 0, 0, 0, 0, 0]
	sat_min_iter = [0, 0, 0, 0, 0, 0]
	sat_max_iter = [0, 0, 0, 0, 0, 0]

	rrt_avg_time = [0, 0, 0, 0, 0, 0]
	rrt_stdev_time = [0, 0, 0, 0, 0, 0]
	rrt_min_time = [0, 0, 0, 0, 0, 0]
	rrt_max_time = [0, 0, 0, 0, 0, 0]
	rrt_avg_iter = [0, 0, 0, 0, 0, 0]
	rrt_stdev_iter = [0, 0, 0, 0, 0, 0]
	rrt_min_iter = [0, 0, 0, 0, 0, 0]
	rrt_max_iter = [0, 0, 0, 0, 0, 0]

	for i in range(len(rrt_stats)):
		times = [stat[1] for stat in rrt_stats[i][2:]]
		iters = [stat[2] for stat in rrt_stats[i][2:]]

		rrt_avg_time[i] = sum(times)/len(times)
		rrt_min_time[i] = min(times)
		rrt_max_time[i] = max(times)
		rrt_stdev_time[i] = stdev(times)

		rrt_avg_iter[i] = sum(iters)/len(iters)
		rrt_min_iter[i] = min(iters)
		rrt_max_iter[i] = max(iters)
		rrt_stdev_iter[i] = stdev(iters)

	for i in range(len(sat_stats)):
		times = [stat[1] for stat in sat_stats[i][2:]]
		iters = [stat[2] for stat in sat_stats[i][2:]]

		sat_avg_time[i] = sum(times)/len(times)
		sat_min_time[i] = min(times)
		sat_max_time[i] = max(times)
		sat_stdev_time[i] = stdev(times)

		sat_avg_iter[i] = sum(iters)/len(iters)
		sat_min_iter[i] = min(iters)
		sat_max_iter[i] = max(iters)
		sat_stdev_iter[i] = stdev(iters)

	x = np.arange(len(labels))  # the label locations
	width = 0.35  # the width of the bars

	fig, ax = plt.subplots()
	rrt_err_time = np.array([rrt_min_time, rrt_max_time])
	sat_err_time = np.array([sat_min_time, sat_max_time])

	rects1 = ax.bar(x - width/2, sat_avg_time, width, yerr = sat_err_time, label='SAT-Plan')
	rects2 = ax.bar(x + width/2, rrt_avg_time, width, yerr = rrt_err_time, label='RRT')

	# Add some text for labels, title and custom x-axis tick labels, etc.
	ax.set_ylabel('Time (s)')
	# ax.set_ylim([0, 1])
	ax.set_title('Average time to find a path')
	ax.set_xticks(x)
	ax.set_xticklabels(labels)
	ax.legend(loc = 'upper left')
	ax.set_yscale('log')
	plt.grid()

	fig.tight_layout()
	plt.savefig('results/figures/'+scenario+'_time_comparison.png')
	print('Saved figure to results/figures/'+scenario+'_time_comparison.png')

	fig2, ax2 = plt.subplots()
	rrt_err_iter = np.array([rrt_min_iter, rrt_max_iter])
	sat_err_iter = np.array([sat_min_iter, sat_max_iter])

	rects1 = ax2.bar(x - width/2, sat_avg_iter, width, yerr = sat_stdev_iter, label='SAT-Plan')
	rects2 = ax2.bar(x + width/2, rrt_avg_iter, width, yerr = rrt_err_iter, label='RRT')

	# Add some text for labels, title and custom x-axis tick labels, etc.
	ax2.set_ylabel('Iterations')
	# ax.set_ylim([0, 1])
	ax2.set_title('Average iterations to find a path')
	ax2.set_xticks(x)
	ax2.set_xticklabels(labels)
	ax2.legend(loc = 'upper center')
	ax2.set_yscale('log')
	plt.grid()

	fig2.tight_layout()
	plt.savefig('results/figures/'+scenario+'_iter_comparison.png')
	print('Saved figure to results/figures/'+scenario+'_iter_comparison.png')

	plt.show()

def plot_comparison(rrt_stats, sat_stats):
	if rrt_stats != 'FAIL':
		rrt_times = [stat[1] for stat in rrt_stats[2:]]
		rrt_iters = [stat[2] for stat in rrt_stats[2:]]

		rrt_avg_time = [sum(rrt_times)/len(rrt_times)]
		rrt_min_time = [min(rrt_times)]
		rrt_max_time = [max(rrt_times)]
		rrt_stdev_time = [stdev(rrt_times)]

		rrt_avg_iter = [sum(rrt_iters)/len(rrt_iters)]
		rrt_min_iter = [min(rrt_iters)]
		rrt_max_iter = [max(rrt_iters)]
		rrt_stdev_iter = [stdev(rrt_iters)]
	else:
		rrt_avg_time = [0]
		rrt_min_time = [0]
		rrt_max_time = [0]
		rrt_stdev_time = [0]

		rrt_avg_iter = [0]
		rrt_min_iter = [0]
		rrt_max_iter = [0]
		rrt_stdev_iter = [0]

	sat_times = [stat[1] for stat in sat_stats[2:]]
	sat_iters = [stat[2] for stat in sat_stats[2:]]

	sat_avg_time = [sum(sat_times)/len(sat_times)]
	sat_min_time = [min(sat_times)]
	sat_max_time = [max(sat_times)]
	sat_stdev_time = [stdev(sat_times)]

	sat_avg_iter = [sum(sat_iters)/len(sat_iters)]
	sat_min_iter = [min(sat_iters)]
	sat_max_iter = [max(sat_iters)]
	sat_stdev_iter = [stdev(sat_iters)]

	x = np.array([1])  # the label locations
	width = 0.2  # the width of the bars

	fig, ax = plt.subplots()
	rrt_err_time = np.array([rrt_min_time, rrt_max_time])
	sat_err_time = np.array([sat_min_time, sat_max_time])

	rects1 = ax.bar(x - width/2, sat_avg_time, width, yerr = sat_err_time, label='SAT-Plan')
	rects2 = ax.bar(x + width/2, rrt_avg_time, width, yerr = rrt_err_time, label='RRT')

	# Add some text for labels, title and custom x-axis tick labels, etc.
	ax.set_ylabel('Time (s)')
	# ax.set_ylim([0, 1])
	ax.set_title('Average time to find a path')
	ax.set_xticks(x)
	ax.set_xticklabels([scenario])
	plt.xlim(0, 2)
	ax.legend(loc = 'upper left')
	ax.set_yscale('log')
	plt.grid()

	fig.tight_layout()
	plt.savefig('results/figures/'+scenario+'_time_comparison.png')
	print('Saved figure to results/figures/'+scenario+'_time_comparison.png')

	fig2, ax2 = plt.subplots()
	rrt_err_iter = np.array([rrt_min_iter, rrt_max_iter])
	sat_err_iter = np.array([sat_min_iter, sat_max_iter])

	rects1 = ax2.bar(x - width/2, sat_avg_iter, width, yerr = sat_stdev_iter, label='SAT-Plan')
	rects2 = ax2.bar(x + width/2, rrt_avg_iter, width, yerr = rrt_err_iter, label='RRT')

	# Add some text for labels, title and custom x-axis tick labels, etc.
	ax2.set_ylabel('Iterations')
	# ax.set_ylim([0, 1])
	ax2.set_title('Average iterations to find a path')
	ax2.set_xticks(x)
	ax2.set_xticklabels([scenario])
	plt.xlim(0, 2)
	ax2.legend(loc = 'upper center')
	ax2.set_yscale('log')
	plt.grid()

	fig2.tight_layout()
	plt.savefig('results/figures/'+scenario+'_iter_comparison.png')
	print('Saved figure to results/figures/'+scenario+'_iter_comparison.png')

	plt.show()

if __name__ == '__main__':
	if scenario == 'zigzag' or scenario == 'all':
		from envs.zigzag import *
		print('Running RRT on zigzag 100 times ...')
		rrt_zig_stats = run_rrt(problem, [-2, 6], 'zigzag')
		print('Running SAT-Plan on zigzag 100 times ...')
		sat_zig_stats = run_yices(problem, [-2, 6], 'zigzag')

		if scenario == 'zigzag':
			sat_stats = sat_zig_stats
			rrt_stats = rrt_zig_stats

	if scenario == 'easy' or scenario == 'all':
		from envs.easy import *
		print('Running RRT on easy 100 times ...')
		rrt_easy_stats = run_rrt(problem, [-2, 15], 'easy')
		print('Running SAT-plan on easy 100 times ...')
		sat_easy_stats = run_yices(problem, [-2, 15], 'easy')

		if scenario == 'easy':
			sat_stats = sat_easy_stats
			rrt_stats = rrt_easy_stats

	if scenario == 'hard' or scenario == 'all':
		from envs.hard import *
		print('Running RRT on hard 100 times ...')
		rrt_hard_stats = run_rrt(problem, [-2, 15], 'hard')
		print('Running SAT-Plan on hard 100 times ...')
		sat_hard_stats = run_yices(problem, [-2, 15], 'zigzag1')

		if scenario == 'hard':
			sat_stats = sat_hard_stats
			rrt_stats = rrt_hard_stats

	if scenario == 'barrier' or scenario == 'all':
		from envs.partition2 import *
		print ('Running RRT on barrier 100 times ...')
		rrt_bar_stats = run_rrt(problem, [0, 4], 'barrier')
		print ('Running SAT-Plan on barrier 100 times ...')
		sat_bar_stats = run_yices(problem, [0, 4], 'barrier')

		if scenario == 'barrier':
			sat_stats = sat_bar_stats
			rrt_stats = rrt_bar_stats

	if scenario == 'maze' or scenario == 'all':
		from envs.maze import *
		print ('Running RRT on maze 100 times ...')
		rrt_maze_stats = run_rrt(problem, [0, 7], 'maze')
		print ('Running SAT-Plan on maze 100 times ...')
		sat_maze_stats = run_yices(problem, [0, 7], 'maze')

		if scenario == 'maze':
			sat_stats = sat_maze_stats
			rrt_stats = rrt_maze_stats

	if scenario == 'SCOTS' or scenario == 'all':
		from envs.SCOTS import *
		print ('Skipping RRT on SCOTS due to TIMEOUT')
		# Since the RRT times out every time for SCOTS, the scenario is not run using RRT
		print ('Running SAT-Plan on SCOTS 100 times ...')
		sat_SCOTS_stats = run_yices(problem, [0, 10], 'SCOTS')

		if scenario == 'SCOTS':
			sat_stats = sat_SCOTS_stats
			rrt_stats = 'FAIL'

	if args.plot == 'True':
		if scenario == 'all':
			rrt_stats = [rrt_easy_stats, rrt_hard_stats, rrt_zig_stats, rrt_bar_stats, rrt_maze_stats]
			sat_stats = [sat_easy_stats, sat_hard_stats, sat_zig_stats, sat_bar_stats, sat_maze_stats, sat_SCOTS_stats]
			plot_comparison_all(rrt_stats, sat_stats)
		else:
			plot_comparison(rrt_stats, sat_stats)
