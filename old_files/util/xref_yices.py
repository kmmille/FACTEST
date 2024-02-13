# Get the x_ref using yices
# Written by: Kristina Miller

from __future__ import division
import numpy as np
import polytope as pc
from yices import *
from math import *

import csv
import time

def add_initial_constraint(Theta):
	A0 = Theta[0]
	b0 = Theta[1]
	x_dim = len(A0[0])

	fmlas = []
	str0 = ''

	poly = pc.Polytope(A0, b0)
	pt = poly.chebXc # Find the center of the initial set

	for dim in range(x_dim): # Make sure that starting point is at the center of the initial set
		str0 = '(= x_%sref[0] %s)'%(dim+1, pt[dim])
		fmlas.append(str0)

	return fmlas


def add_final_constraint(Goal, N, bloat_func, alpha):
	Af = Goal[0]
	bf = Goal[1]
	x_dim = len(Af[0])

	poly = pc.Polytope(Af, bf)
	pt = poly.chebXc # Find the center of the goal set

	fmlas = []
	str0 = ''

	for dim in range(x_dim): # Make sure that ending point is at the center of the goal set
		str0 = '(= x_%sref[%s] %s)'%(dim+1, N, pt[dim])
		fmlas.append(str0)

	# Additionally, make sure all the error is contained within the goal set
	r = bloat_func(N-1, alpha)
	for row in range(len(Af)):
		b = bf[row][0] - r
		for dim in range(x_dim):
			a = Af[row][dim]
			str1 = '(* %s x_%sref[%s])'%(a, dim+1, N)
			if dim == 0:
				str0 = str1
			else:
				str0 = '(+ %s %s)'%(str0, str1)

		fmla = '(<= %s %s)'%(str0, b)
		fmlas.append(fmla)

	return fmlas


def add_xref_constraints(O, N, bloat_func, alpha):
	if O == []:
		return None
	else:
		# Add the constraints to avoid the obstacles
		x_dim = len(O[0][0][0])

		str0 = ''
		str1 = ''
		fmlas = []
		for (Ao, bo) in O:
			for i in range(N):
				fmla = []
				for row in range(len(Ao)):
					r = bloat_func(i, alpha)

					b = bo[row][0] + r
					for dim in range(x_dim):
						a = Ao[row][dim]
						str2 = '(* %s x_%sref[%s])'%(a, dim+1, i)
						str3 = '(* %s x_%sref[%s])'%(a, dim+1, i+1)
						if dim == 0:
							str0 = str2
							str1 = str3
						else:
							str0 = '(+ %s %s)'%(str0, str2)
							str1 = '(+ %s %s)'%(str1, str3)

					fmla0 = '(> %s %s)'%(str0, b)
					fmla1 = '(> %s %s)'%(str1, b)
					fmla2 = '(and %s %s)'%(fmla0, fmla1)
					fmla.append(fmla2)
				fmla4 = ' '.join(fmla)
				fmlas.append('(or '+fmla4+')')
		return fmlas


def get_constraints(Theta, Goal, O, N, bloat_func, alpha):
	# Get all the constraints in one function
	ref_constraint = add_xref_constraints(O, N, bloat_func, alpha)
	final_constraint = add_final_constraint(Goal, N, bloat_func, alpha)
	initial_constraint = add_initial_constraint(Theta)
	if ref_constraint == None:
		constraints = final_constraint + initial_constraint
	else:
		constraints = ref_constraint + final_constraint + initial_constraint
	return constraints

def get_xref_yices(Theta, Goal, O, N, bloat_func, alpha, N_min = 1):
	# Theta is the initial set
	# Goal is the final set
	# O is a list of obstacles of the form Ax < b
	# N is the maximum number of line segments

	x_dim = len(Goal[0][0])

	cfg = Config()
	cfg.default_config_for_logic('QF_LRA')
	ctx = Context(cfg)

	real_t = Types.real_type()

	x_ref = [[Terms.new_uninterpreted_term(real_t, "x_%sref[0]" %(j+1)) for j in range(x_dim)],
			 [Terms.new_uninterpreted_term(real_t, "x_%sref[1]" %(j+1)) for j in range(x_dim)]]

	for i in range(N_min, N):
		constraints = get_constraints(Theta, Goal, O, i, bloat_func, alpha)
		fmlas = [Terms.parse_term(j) for j in constraints]

		ctx.assert_formulas(fmlas)

		status = ctx.check_context()
		if status == Status.SAT:
			n = i
			break
		else:
			ctx.dispose()
			ctx = Context(cfg)
			x_ref.append([Terms.new_uninterpreted_term(real_t, "x_%sref[%s]" %(j+1, i+1)) for j in range(x_dim)])

	x_path = []
	if status == Status.SAT:
		model = Model.from_context(ctx, 1)
		for i in range(n+1):
			x_new = []
			for j in range(x_dim):
				xval = model.get_value(x_ref[i][j])
				x_new.append(xval)
			x_path.append(x_new)
	else:
		x_path = None

	return x_path

def example_bloat(n, alpha): # Bloating for comparing RRT and SAT-Plan
	return 0

def run_yices(scenario, search_area, envname):
	O, Theta, Goal = scenario()

	stats = [['SAT-Plan', envname],['path?', 'time', 'iters', 'segments']]
	for i in range(100):
		t_start = time.time()
		nodes = get_xref_yices(Theta, Goal, O, 100, example_bloat, 0)
		t_end = time.time()
		t = t_end - t_start

		if nodes != None:
			path_bool = True
		else:
			path_bool = False

		j = len(nodes)
		segs = len(nodes)

		stats.append([path_bool, t, j, segs])

	with open('results/RRTvSAT-Plan/yices_'+envname+'.csv', 'w', newline='') as file:
		writer = csv.writer(file)
		writer.writerows(stats)
	print('Saved SAT-Plan results to results/RRTvSAT-Plan/yices_'+envname+'.csv')
	return stats

if __name__ == '__main__':
	A0 = np.array([[-1, 0], [1, 0], [0, -1], [0, 1]])
	b0 = np.array([[0], [1], [0], [1]])
	Theta = (A0, b0)

	A1 = np.array([[-1, 0], [1, 0], [0, -1], [0, 1]])
	b1 = np.array([[-4], [5], [-4], [5]])
	Goal = (A1, b1)

	A2 = np.array([[-1, 0], [1, 0], [0, -1], [0, 1]])
	b2 = np.array([[-3], [3.5], [0], [5]])
	b3 = np.array([[-3], [7], [-5.5], [6]])
	b4 = np.array([[-3], [7], [1], [0]])
	O = [(A2, b2), (A2, b3), (A2, b4)]

	alpha = 1

	x_path = get_xref_yices(Theta, Goal, O, 10, bloating, alpha)

	print(x_path)
