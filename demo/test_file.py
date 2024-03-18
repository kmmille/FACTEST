#######################################
# Import required libraries and files #
#######################################

## Set up file paths ##
#######################
import sys, os
currFile = os.path.abspath(__file__)
modelPath = currFile.replace('/demo/test_file.py', '')
sys.path.append(modelPath)
factestPath = currFile.replace('/demo/test_file.py', '/factest/synthesis')
sys.path.append(factestPath)

## Import Python libraries ##
#############################
import numpy as np
import matplotlib.pyplot as plt

## Import FACTEST files ##
##########################
from factest.synthesis.factest_base_z3 import FACTEST_Z3
from models.dubins_plane import dubins_plane
#TODO: Need to make a 3d testing file

model = dubins_plane()


state = [0,0,0,0,0]
ref_state = [1,1,1,0,0]
ref_input = [1,0,0,0]

T = 10

xref = [[1,1,1,0,0], [10,10,10,0,0]]
states = model.run_simulation(xref, 1, state, T)

x_sim = [state[0] for state in states]
y_sim = [state[1] for state in states]

xref_1 = [xval[0] for xval in xref]
xref_2 = [xval[1] for xval in xref]


fig, ax = plt.subplots()
ax.plot(xref_1, xref_2, marker = 'o')
ax.plot(x_sim, y_sim, linestyle = '--')
ax.set_xlim(-10,10)
ax.set_ylim(-10,10)
plt.show()