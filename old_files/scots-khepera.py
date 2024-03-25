# SCOTS HSCC 2 Comparison
import sys, os
currFile = os.path.abspath(__file__)
modelPath = currFile.replace('/synthesis/FACTEST/omega_FACTEST/scots-khepera.py', '')
sys.path.append(modelPath)
sys.path.append(modelPath+'/utils/')

from models.dubins.dubinsAgent_2d import dubinsAgent2d_kinematic
from models.dubins.bijectiveAgent import bijectiveAgent
from simulation import simulation
from plotting import plotPoly
from ltl_to_tba import getBuchi
from tba_to_hybrid import getHybrid, constructFullHybrid

from math import sqrt, ceil, pi
import polytope as pc
import numpy as np
import matplotlib.pyplot as plt

import time

A = np.array([[-1,0],[1,0],[0,-1],[0,1]])

b1 = np.array([-0.0, 0.5, - 8.2, 13])
b2 = np.array([-0.5, 1.5, -10.5, 13])
b3 = np.array([-1.5, 2.2, - 8.2, 13])

b4 = np.array([-1.5, 2.2,  -6.3, 7.0])

b5 = np.array([-0.0, 0.5,  -1.2, 5])
b6 = np.array([-0.5, 1.5,  -1.2, 3])
b7 = np.array([-1.5, 2.2,  -1.2, 5])

b8 = np.array([-3.0, 3.3,  - 0.0, 1.5])
b9 = np.array([-3.0, 3.3,  - 3.7, 5.2])
b10 = np.array([-3.0, 3.3,  - 7.5, 8.9])
b11 = np.array([-3.0, 3.3,  -10.8, 12.2])

b12 = np.array([-4.2, 6.8,  -10.8, 11.3])
b13 = np.array([-6.2, 6.8,  -11.3, 13])
b14 = np.array([-4.2, 6.8,  - 9.6, 10.0])
b15 = np.array([-4.2, 6.8,  - 7.9,  8.5])
b16 = np.array([-4.2, 4.8,  - 4.9,  7.9])
b17 = np.array([-4.2, 6.8,  - 3.5,  6.0])
b18 = np.array([-5.1, 6.8,  - 2.4,  2.8])
b19 = np.array([-4.2, 6.8,  - 0.0,  1.5])

b20 = np.array([-7.7, 8.0,  - 11 ,   13])
b21 = np.array([-7.7, 8.0,  - 3.8,  8.1])
b22 = np.array([-7.7, 8.0,  - 0.0,  1.3])

b23 = np.array([-9.1,  11.2,  -10.8, 13])
b24 = np.array([-9.1,  11.2,  - 9.6, 10.0])
b25 = np.array([-9.1,  11.2,  - 6.8,  8.5])
b26 = np.array([-9.1,  11.2,  - 3.8,  4.4])
b27 = np.array([-9.1,  10.2,  - 2.4,  2.8])
b28 = np.array([-8.9,  15.0,  - 0.0,  1.5])

b29 = np.array([-12.0, 12.3,  - 11 ,   13])
b30 = np.array([-12.0, 12.3,  - 6.2,  8.1])
b31 = np.array([-12.0, 12.3,  - 3.8,  4.9])

b32 = np.array([-13.2,   15,  - 6.8,  11.5])
b33 = np.array([-13.2,   14,  - 5.6,   5.9])
b34 = np.array([-13.2,   15,  - 3.6,  4.7])

# Goal
b35 = np.array([-3.0, 3.3,  - 2.2, 2.6])

# Initial
b36 = np.array([-12.4,12.6,-9.9,10.1])


E1 = pc.Polytope(A, b1)
E2 = pc.Polytope(A, b2)
E3 = pc.Polytope(A, b3)
E4 = pc.Polytope(A, b4)
E5 = pc.Polytope(A, b5)
E6 = pc.Polytope(A, b6)
E7 = pc.Polytope(A, b7)
E8 = pc.Polytope(A, b8)
E9 = pc.Polytope(A, b9)
E10 = pc.Polytope(A, b10)
E11 = pc.Polytope(A, b11)
E12 = pc.Polytope(A, b12)
E13 = pc.Polytope(A, b13)
E14 = pc.Polytope(A, b14)
E15 = pc.Polytope(A, b15)
E16 = pc.Polytope(A, b16)
E17 = pc.Polytope(A, b17)
E18 = pc.Polytope(A, b18)
E19 = pc.Polytope(A, b19)
E20 = pc.Polytope(A, b20)
E21 = pc.Polytope(A, b21)
E22 = pc.Polytope(A, b22)
E23 = pc.Polytope(A, b23)
E24 = pc.Polytope(A, b24)
E25 = pc.Polytope(A, b25)
E26 = pc.Polytope(A, b26)
E27 = pc.Polytope(A, b27)
E28 = pc.Polytope(A, b28)
E29 = pc.Polytope(A, b29)
E30 = pc.Polytope(A, b30)
E31 = pc.Polytope(A, b31)
E32 = pc.Polytope(A, b32)
E33 = pc.Polytope(A, b33)
E34 = pc.Polytope(A, b34)

E35 = pc.Polytope(A,b35)

E36 = pc.Polytope(A,b36)

fig,ax = plt.subplots()

plotPoly(E1,ax,'red')
plotPoly(E2,ax,'red')
plotPoly(E3,ax,'red')
plotPoly(E4,ax,'red')
plotPoly(E5,ax,'red')
plotPoly(E6,ax,'red')
plotPoly(E7,ax,'red')
plotPoly(E8,ax,'red')
plotPoly(E9,ax,'red')
plotPoly(E10,ax,'red')
plotPoly(E11,ax,'red')
plotPoly(E12,ax,'red')
plotPoly(E13,ax,'red')
plotPoly(E14,ax,'red')
plotPoly(E15,ax,'red')
plotPoly(E16,ax,'red')
plotPoly(E17,ax,'red')
plotPoly(E18,ax,'red')
plotPoly(E19,ax,'red')
plotPoly(E20,ax,'red')
plotPoly(E21,ax,'red')
plotPoly(E22,ax,'red')
plotPoly(E23,ax,'red')
plotPoly(E24,ax,'red')
plotPoly(E25,ax,'red')
plotPoly(E26,ax,'red')
plotPoly(E27,ax,'red')
plotPoly(E28,ax,'red')
plotPoly(E29,ax,'red')
plotPoly(E30,ax,'red')
plotPoly(E31,ax,'red')
plotPoly(E32,ax,'red')
plotPoly(E33,ax,'red')
plotPoly(E34,ax,'red')

plotPoly(E35,ax,'green')

plotPoly(E36,ax,'blue')

ax.set_xlim(0,15)
ax.set_ylim(0,15)
ax.xaxis.set_tick_params(labelbottom=False)
ax.yaxis.set_tick_params(labelleft=False)

plt.show()
sys.exit()
