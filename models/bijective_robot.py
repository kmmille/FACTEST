# Bijective mobile robot
# Written by: Kristina Miller

from math import *
import numpy as np
from scipy.misc import derivative
from scipy.linalg import norm
from scipy.integrate import odeint
import matplotlib.pyplot as plt

k = [1000, 1000, 1000]
a = 1000
b = 1

def model_name():
	return 'Bijective Robot'

def model(q, t, u):
	x, y, s, c = q
	v, w = u

	xdot = c*v
	ydot = s*v
	sdot = c*w
	cdot = -s*w

	return [xdot, ydot, sdot, cdot]

def controller(q, qref, uref):
	x, y, s, c = q
	xref, yref, thetaref = qref
	vref, wref = uref
	k1, kx, ks = [100, 100, 100]
	a = 4

	xe = c*(xref - x) + s*(yref - y)
	ye = -s*(xref - x) + s*(yref - y)
	se = sin(thetaref)*c - cos(thetaref)*s
	ce = cos(thetaref)*c + sin(thetaref)*s

	v = kx*xe
	w = k1*vref*ye*(1+ce/a)**2 + ks*se*((1+ce/a)**2)**b

	return [v, w]

def bloating(n, alpha):
	k1, kx, ks = k
	if n != 0:
		return sqrt(alpha**2 + n*8/(k1*(a-2)))
	else:
		return sqrt(alpha**2)

def run_model(q0, t, qref, uref):
	q = [q0]
	u0 = [0, 0]
	for i in range(0,len(t)):
		t_step = np.linspace(t[i-1], t[i], 11)
		q1 = odeint(model, q0, t_step, args = (u0,))
		q0 = q1[1]
		q.append(q0)
		u0 = controller(q0, qref[i], uref[i])
	return q
