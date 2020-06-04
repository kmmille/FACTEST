# Kinematic Car Model
# Written by: Kristina Miller

import numpy as np
from math import *
from numpy.linalg import norm
from scipy.misc import derivative
from scipy.integrate import odeint
import matplotlib.pyplot as plt

k = [10000, 10000, 10000]

def model_name():
	return "Car Model"

def model(q, t, u):
	x, y, theta = q
	v, w = u

	dxdt = v*cos(theta)
	dydt = v*sin(theta)
	dthetadt = w
	return [dxdt, dydt, dthetadt]

def controller(q, qref, uref, k_plot = [100, 100, 100]):
	x, y, theta = q
	xref, yref, thetaref = qref
	vref, wref = uref
	k1, k2, k3 = k_plot # For plotting purposes only

	xe = cos(theta)*(xref - x) + sin(theta)*(yref - y)
	ye = -sin(theta)*(xref - x) + cos(theta)*(yref - y)
	thetae = thetaref - theta

	v = vref*cos(thetae) + k1*xe
	w = wref + vref*(k2*ye + k3*sin(thetae))

	return [v, w]

def bloating(n, alpha):
	k1, k2, k3 = k
	if n != 0 and n != 1:
		return sqrt(alpha**2 + 4*n/k2)
	else:
		return sqrt(alpha**2 + 4/k2)

def run_model(q0, t, qref, uref, k_plot = [100, 100, 100]):
	q = [q0]
	u0 = [0, 0]
	for i in range(0,len(t)):
		t_step = np.linspace(t[i-1], t[i], 5)
		q1 = odeint(model, q0, t_step, args = (u0,))
		q0 = q1[1]
		q.append(q0)
		u0 = controller(q0, qref[i], uref[i], k_plot)
	return q
