# 3d Kinematic Model
# Written by: Kristina Miller

from math import *
from mpl_toolkits import mplot3d

import numpy as np
from numpy.linalg import norm
from scipy.misc import derivative
import matplotlib.pyplot as plt
from scipy.integrate import odeint

k = [1000,1000,1000,1000]

def model_name():
	return 'Hovercraft Model'

def model(q, t, u):
	x,y,z,theta = q
	v,vz, w = u

	dxdt = v*cos(theta)
	dydt = v*sin(theta)
	dzdt = vz
	dthetadt = w
	return [dxdt, dydt, dzdt, dthetadt]

def controller(q, qref, uref):
	x,y,z,theta = q

	xref, yref, zref, rollref, pitchref, yawref = qref
	thetref = pitchref

	vref = uref[0]*cos(pitchref)
	vzref = uref[0]*sin(pitchref)

	wxref, wyref, wzref = uref[1]
	omegaref = wzref

	k1,k2,k3,kp = [2,2,1,1] # For plotting purposes only

	xe = cos(theta)*(xref - x) + sin(theta)*(yref - y)
	ye = -sin(theta)*(xref - x) + cos(theta)*(yref - y)
	ze = zref - z
	thetae = thetref - theta

	v = vref*cos(thetae) + k1*xe
	omega = omegaref + vref*(k2*ye + k3*sin(thetae))
	vz = vzref + kp*ze

	return [v, vz, omega]

def bloating(n, alpha):
	k1,k2,k3,kp = k
	if n != 0:
		return sqrt(alpha**2 + 4*n/k2)
	else:
		return sqrt(alpha**2)

def run_model(q0, t, qref, uref):
	q = [q0]
	u0 = [0, 0, 0]
	for i in range(0,len(t)):
		t_step = [t[i-1], t[i]]
		q1 = odeint(model, q0, t_step, args = (u0,))
		q0 = q1[1]
		q.append(q0)
		u0 = controller(q0, qref[i], uref[i])
	return q
