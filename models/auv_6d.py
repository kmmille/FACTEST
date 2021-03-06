# 6D AUV
# Written by: Kristina Miller

from math import *
from mpl_toolkits import mplot3d
import numpy as np
from numpy.linalg import inv, norm
from scipy.misc import derivative
from scipy.integrate import odeint
import matplotlib.pyplot as plt

k = [0.001, 0.001, 0.001]

def model_name():
	return 'Autonomous Underwater Vehicle'

def model(q, t, u):
	#x, y, z, roll, pitch, yaw
	x, y, z, phi, theta, psi = q
	v = u[0]
	wx, wy, wz = u[1]

	xdot = v*cos(psi)*cos(theta)
	ydot = v*sin(psi)*cos(theta)
	zdot = v*sin(theta)
	phidot = wx + wy*sin(phi)*tan(theta) + wz*cos(phi)*tan(theta)
	thetadot = wy*cos(phi) - wz*sin(phi)
	psidot = wy*sin(phi)*(1/cos(theta)) + wz*cos(phi)*(1/cos(theta))

	return [xdot, ydot, zdot, phidot, thetadot, psidot]

def calculate_error(q, qref, uref, R):
	X = np.array(q[0:3])
	Theta = np.array(q[3:6])

	# X_d = trajectory(t[idx], T, params)
	X_d = np.array(qref[0:3])
	Theta_d = np.array(qref[3:6])

	v_d = uref[0]
	wx,wy,wz = uref[1]

	X_e = np.transpose(R)*(X_d - X)
	Theta_e = Theta_d - Theta

	return [X_e[0][0], X_e[1][1], X_e[2][2], Theta_e[0], Theta_e[1], Theta_e[2]]

def controller(q, qref, uref):
	x, y, z, phi, theta, psi = q
	c_phi = cos(phi)
	c_theta = cos(theta)
	c_psi = cos(psi)
	s_phi = sin(phi)
	s_theta = sin(theta)
	s_psi = sin(psi)

	k_1, k_2, k_3 = [1,1,1] # For plotting purposes only

	R = np.array([[c_psi*c_theta, c_psi*s_theta*s_phi - s_psi*c_phi, c_psi*s_theta*c_phi + s_psi*s_phi],
				  [s_psi*c_theta, s_psi*s_theta*s_phi + c_psi*c_phi, s_psi*s_theta*c_phi - c_psi*s_phi],
				  [-s_theta, c_theta*s_phi, c_theta*c_phi]])
	B_2 = np.array([[1, sin(phi)*tan(theta), cos(phi)*tan(theta)],
					[0, cos(phi), -sin(phi)],
					[0, sin(phi)*(1/cos(theta)), cos(phi)*(1/cos(theta))]])

	x_d, y_d, z_d, phi_d, theta_d, psi_d = qref
	u_2d = np.array([[phi_d]])

	v_d = uref[0]
	# The error is calculated here. This is the error between the waypoint and
	# the current state
	e_x, e_y, e_z, e_phi, e_theta, e_psi = calculate_error(q, qref, uref, R)
	u_2d = np.transpose(np.array(uref[1]))

	B_2d = np.array([[1, sin(phi_d)*tan(theta_d), cos(phi_d)*tan(theta_d)],
					[0, cos(phi_d), -sin(phi_d)],
					[0, sin(phi_d)*(1/cos(theta_d)), cos(phi_d)*(1/cos(theta_d))]])

	Q = np.transpose(np.array([0, e_z*v_d/k_2, e_y*v_d*cos(e_theta)/k_3]))#te
	P_e = np.transpose(np.array([k_1*sin(e_phi), k_2*sin(e_theta), k_3*sin(e_psi)]))

	# This is the control law
	v_b = v_d*(cos(e_psi)*cos(e_theta) - 1) + e_x*1**2
	u_2b = inv(B_2)*(Q + (B_2d - B_2)*u_2d + P_e)

	u_1 = v_d+v_b
	u_2 = u_2d + u_2b
	u_2 = [u_2[0][0], u_2[1][1], u_2[2][2]]

	return [u_1, u_2]

def bloating(n, alpha):
	k1,k2,k3 = k
	if n != 0:
		return sqrt(alpha**2 + 4*n*(k1+k2+k3))
	else:
		return sqrt(alpha**2)

def run_model(q0, t, qref, uref):
	q = [q0]
	u0 = [0,[0, 0, 0]]
	for i in range(0,len(t)):
		t_step = [t[i-1], t[i]]
		q1 = odeint(model, q0, t_step, args = (u0,))
		q0 = q1[1]
		q.append(q0)
		u0 = controller(q0, qref[i], uref[i])
	return q
