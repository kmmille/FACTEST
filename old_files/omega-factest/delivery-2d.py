import sys, os
currFile = os.path.abspath(__file__)
modelPath = currFile.replace('/synthesis/FACTEST/omega_FACTEST/delivery-2d.py', '')
sys.path.append(modelPath)
sys.path.append(modelPath+'/utils/')
modelPath = currFile.replace('/delivery-2d.py', '')
sys.path.append(modelPath)

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

#####################
# Set up the system #
#####################

system = 'robot'

if system == 'car':
    my_id = "myCar"
    my_code = None
    my_filename = currFile.replace('synthesis/FACTEST/omega_FACTEST/delivery-2d.py', 'models/dubins/dubinsControl_2dKinematic.py')

    x0 = 1
    y0 = -1
    theta0 = 0
    my_initial_state = [x0,y0,theta0]

    my_initial_mode = None
    my_static_param = None
    my_uncertain_param = None

    my_car = dubinsAgent2d_kinematic(id=my_id, code=my_code, filename=my_filename, initial_state=my_initial_state, initial_mode=my_initial_mode, static_param=my_static_param, uncertain_param=my_uncertain_param)
else:
    my_id = "myRobot"
    my_code = None
    my_filename = currFile.replace('synthesis/FACTEST/omega_FACTEST/delivery-2d.py', 'models/dubins/bijectiveControl.py')

    x0 = 1
    y0 = -1
    s0 = 0
    c0 = 1
    my_initial_state = [x0,y0,s0,c0]

    my_initial_mode = None
    my_static_param = None
    my_uncertain_param = None

    my_car = bijectiveAgent(id=my_id, code=my_code, filename=my_filename, initial_state=my_initial_state, initial_mode=my_initial_mode, static_param=my_static_param, uncertain_param=my_uncertain_param)

##########################
# Set up the environment #
##########################

A = np.array([[-1,0],[1,0],[0,-1],[0,1]])

b_init = np.array([0.5,0.5,0.5,0.5])

b_goal1 = np.array([-5,7,-5,7]) # d3 - delivery3
b_goal2 = np.array([7,-5,-5,7]) # d1 - delivery1
b_goal3 = np.array([1,1,7,-5]) # p - pickup
b_goal4 = np.array([1,1,-5,7]) # d2 - delivery 2

b_unsafe1 = np.array([11,-10,11,11])
b_unsafe2 = np.array([-10,11,11,11])
b_unsafe3 = np.array([11,11,-10,11])
b_unsafe4 = np.array([11,11,11,-10])

b_unsafe5 = np.array([11,-2,11,-2])
b_unsafe6 = np.array([-2,11,11,-2])
b_unsafe7 = np.array([-2,4,-2,11])
b_unsafe8 = np.array([4,-2,-2,11])

initial_set = pc.Polytope(A, b_init)
d3 = pc.Polytope(A, b_goal1) # goal set 1
d1 = pc.Polytope(A, b_goal2) # goal set 2
p = pc.Polytope(A, b_goal3) # goal set 3
d2 = pc.Polytope(A, b_goal4) # goal set 4

E5 = pc.Polytope(A, b_unsafe5) # unsafe set 1
E6 = pc.Polytope(A, b_unsafe6) # unsafe set 2
E7 = pc.Polytope(A, b_unsafe7) # unsafe set 3
E8 = pc.Polytope(A, b_unsafe8) # unsafe set 4

E9 = pc.Polytope(A, b_unsafe1) # unsafe border 1
E10 = pc.Polytope(A, b_unsafe2) # unsafe border 2
E11 = pc.Polytope(A, b_unsafe3) # unsafe border 3
E12 = pc.Polytope(A, b_unsafe4) # unsafe border 4

env = {'p':p, 'd1':d1, 'd2':d2, 'd3':d3, 'o1':E5, 'o2':E6, 'o3':E7, 'o4':E8}
borders = [E9, E10, E11, E12]

plot_env = False
if plot_env:
    fig,ax = plt.subplots()

    plotPoly(initial_set,ax,'blue')

    plotPoly(p,ax,'green')
    plotPoly(d1,ax,'green')
    plotPoly(d2,ax,'green')
    plotPoly(d3,ax,'green')

    plotPoly(E5,ax,'red')
    plotPoly(E6,ax,'red')
    plotPoly(E7,ax,'red')
    plotPoly(E8,ax,'red')
    plotPoly(E9,ax,'red')
    plotPoly(E10,ax,'red')
    plotPoly(E11,ax,'red')
    plotPoly(E12,ax,'red')

    ax.set_xlim(-10,10)
    ax.set_ylim(-10,10)
    ax.xaxis.set_tick_params(labelbottom=False)
    ax.yaxis.set_tick_params(labelleft=False)

    plt.show()

###############################
# LTL formula for reach-avoid #
###############################

reach_str = """F p & F d1 & F d2 & F d3 & ((!d1 & !d2 & !d3) U p) &
               G (p -> (X (!p U (d1 | d2 | d3)) & (F d1 & F d2 & F d3))) &
               G (d1 -> (X (!d1 U (d2 | d3)) & X ((!d1 & !d2 & !d3) U p) & (F p & F d2 & F d3))) &
               G (d2 -> (X (!d2 U (d1 | d3)) & X ((!d1 & !d2 & !d3) U p) & (F d1 & F p & F d3))) &
               G (d3 -> (X (!d3 U (d1 | d2)) & X ((!d1 & !d2 & !d3) U p) & (F d1 & F d2 & F p)))"""
               
avoid_str = 'G (!o1 & !o2 & !o3 & !o4)'
ltl_formula = reach_str + ' & ' + avoid_str

###############################
# Construct discrete automata #
###############################

myBuchi = getBuchi(ltl_formula=ltl_formula,env=env)
buchi_states, buchi_inits, buchi_AP, buchi_alphabet, buchi_transitions, buchi_acceptances, buchi_run = myBuchi.getBuchi()

###############################
# Construct discrete automata #
###############################

# myBuchi = getBuchi(ltl_formula=ltl_formula,env=env)
# buchi_states, buchi_inits, buchi_AP, buchi_alphabet, buchi_transitions, buchi_acceptances, buchi_run = myBuchi.getBuchi()

##############################
# Construct hybrid automaton #
##############################

# hybridAutomaton = getHybrid(buchi_states, buchi_inits, buchi_AP, buchi_alphabet, buchi_transitions, buchi_acceptances, buchi_run, env, my_car, initial_set, border_sets=borders)
# flows, no_cover, cover = hybridAutomaton.getFlow()

#####################
# Get full automata #
#####################

print('starting!')
hybrid = constructFullHybrid(ltl_formula=ltl_formula, env=env, system=my_car, initial_set=initial_set, border_sets=borders)

init_time = time.time()
hybrid.constrctFullAutomaton()
construction_time = time.time() - init_time

###################################
# Get data for table in CAV paper #
###################################

num_flows = 0
num_transitions = 0
num_states = 0
max_segs = 1
min_segs = 1
max_flows = 0
for state in list(hybrid.flows.keys()):
    for transition in list(hybrid.flows[state].keys()):
        num_flows += len(hybrid.flows[state][transition])
    for flow in hybrid.flows[state][transition]:
        max_segs = max(max_segs, len(flow['xref'])-1)
        min_segs = min(min_segs, len(flow['xref'])-1)

for state in list(hybrid.buchi_transitions.keys()):
    num_transitions += len(hybrid.buchi_transitions[state])
    
    max_flows += len(hybrid.buchi_transitions[state])*len(hybrid.reset[state])*16

# print(len(hybrid.buchi_states))

# print('automaton states', hybrid.buchi_states)
# print('initial states', hybrid.buchi_inits)
# print('atomic props', hybrid.buchi_AP)
# print('alphabet', hybrid.buchi_alphabet)
# print('transitions', hybrid.buchi_transitions)
# print('acceptances', hybrid.buchi_acceptances)
# print('run', hybrid.buchi_run)
# print('env', hybrid.env)
# print('reset', hybrid.reset)

# print('construction time: ', construction_time)
# print('num transitions: ', num_transitions)
# print('num flows: ', num_flows)
# print('max segs: ', max_segs)
# print('min segs: ', min_segs)

import csv
with open('/Users/krismiller/Desktop/dissertation/synthesis/FACTEST/omega_FACTEST/results/delivery_'+system+'.csv', 'w', newline='') as csvfile:
    spamwriter = csv.writer(csvfile, delimiter=',')

    spamwriter.writerow(['automaton states', hybrid.buchi_states])
    spamwriter.writerow(['initial states', hybrid.buchi_inits])
    spamwriter.writerow(['atomic props', hybrid.buchi_AP])
    spamwriter.writerow(['alphabet', hybrid.buchi_alphabet])
    spamwriter.writerow(['transitions', hybrid.buchi_transitions])
    spamwriter.writerow(['acceptances', hybrid.buchi_acceptances])
    spamwriter.writerow(['run', hybrid.buchi_run])
    spamwriter.writerow(['env', hybrid.env])
    spamwriter.writerow(['reset', hybrid.reset])
    spamwriter.writerow(['construction time', construction_time])
    spamwriter.writerow(['num transitions', num_transitions])
    spamwriter.writerow(['num flows', num_flows])
    spamwriter.writerow(['max flows', max_flows])
    spamwriter.writerow(['max segs', max_segs])
    spamwriter.writerow(['min segs', min_segs])
# ####################
# # Execute automata #
# ####################

# # hybrid.execute()

# #####################################################
# # Figure for CAV paper - synthesizing flows example #
# #####################################################

# fig,ax = plt.subplots(1,3)

# ax1 = ax[0]
# ax2 = ax[1]
# ax3 = ax[2]

# ax1.set_xlim(-10,10)
# ax1.set_ylim(-10,10)
# ax1.xaxis.set_tick_params(labelbottom=False)
# ax1.yaxis.set_tick_params(labelleft=False)

# ax2.set_xlim(-10,10)
# ax2.set_ylim(-10,10)
# ax2.xaxis.set_tick_params(labelbottom=False)
# ax2.yaxis.set_tick_params(labelleft=False)

# ax3.set_xlim(-10,10)
# ax3.set_ylim(-10,10)
# ax3.xaxis.set_tick_params(labelbottom=False)
# ax3.yaxis.set_tick_params(labelleft=False)

# initial_plot = False
# E1_plot = False
# E2_plot = False

# for state in list(hybrid.flows.keys()):
#     for letter in list(hybrid.flows[state].keys()):
#         print(letter == str(['E1']))
#         for flow_dict in hybrid.flows[state][letter]:
#             # Plot initial set
#             init_poly = flow_dict['init']
#             plotPoly(init_poly,ax1,'gray')
#             plotPoly(init_poly,ax2,'gray')
#             plotPoly(init_poly,ax3,'gray')

#             if len(init_poly.intersect(initial_set).A) != 0 and not initial_plot:
#                 x_ref = [state[0] for state in flow_dict['xref']]
#                 y_ref = [state[1] for state in flow_dict['xref']]
#                 ax1.plot(x_ref, y_ref, color = 'blue', linestyle = 'dotted')
#                 ax1.plot(x_ref[0], y_ref[0], color = 'blue', marker = 'o')
#                 initial_plot = True
#             elif len(init_poly.intersect(E1).A) != 0 and not E1_plot and letter == str(['E2']):
#                 x_ref = [state[0] for state in flow_dict['xref']]
#                 y_ref = [state[1] for state in flow_dict['xref']]
#                 ax2.plot(x_ref, y_ref, color = 'gray', linestyle = 'dashed')
#                 ax2.plot(x_ref[0], y_ref[0], color = 'gray', marker = 'o')
#                 E1_plot = True
#             elif len(init_poly.intersect(E2).A) != 0 and not E2_plot and letter == str(['E1']):
#                 x_ref = [state[0] for state in flow_dict['xref']]
#                 y_ref = [state[1] for state in flow_dict['xref']]
#                 ax3.plot(x_ref, y_ref, color = 'gray', linestyle = 'dashdot', label='E2 to E1')
#                 ax3.plot(x_ref[0], y_ref[0], color = 'gray', marker = 'o')
#                 E2_plot = True

# plotPoly(initial_set,ax1,'blue')
# # 
# plotPoly(E1,ax1,'green')
# plotPoly(E2,ax1,'green')
# # 
# plotPoly(E3,ax1,'red')
# plotPoly(E4,ax1,'red')
# plotPoly(E5,ax1,'red')
# plotPoly(E6,ax1,'red')
# plotPoly(E7,ax1,'red')
# plotPoly(E8,ax1,'red')


# plotPoly(initial_set,ax2,'blue')
# # 
# plotPoly(E1,ax2,'green')
# plotPoly(E2,ax2,'green')
# # 
# plotPoly(E3,ax2,'red')
# plotPoly(E4,ax2,'red')
# plotPoly(E5,ax2,'red')
# plotPoly(E6,ax2,'red')
# plotPoly(E7,ax2,'red')
# plotPoly(E8,ax2,'red')


# plotPoly(initial_set,ax3,'blue')
# # 
# plotPoly(E1,ax3,'green')
# plotPoly(E2,ax3,'green')
# # 
# plotPoly(E3,ax3,'red')
# plotPoly(E4,ax3,'red')
# plotPoly(E5,ax3,'red')
# plotPoly(E6,ax3,'red')
# plotPoly(E7,ax3,'red')
# plotPoly(E8,ax3,'red')


# plt.show()

















# ########################
# # Plotting environment #
# ########################

# # initial_poly = pc.Polytope(A, b_init)
# # goal_polys = [pc.Polytope(A, b_goal1), pc.Polytope(A, b_goal2)]
# # unsafe_polys = [pc.Polytope(A, b_unsafe1),pc.Polytope(A, b_unsafe2),pc.Polytope(A, b_unsafe3),pc.Polytope(A, b_unsafe4),pc.Polytope(A, b_unsafe5),pc.Polytope(A, b_unsafe6)]
# # fig,ax = plt.subplots()
# # plotPoly(initial_poly,ax,'blue')
# # plotPoly(unsafe_polys,ax,'red')
# # plotPoly(goal_polys,ax,'green')
# # ax.set_xlim(-10,10)
# # ax.set_ylim(-10,10)
# # ax.xaxis.set_tick_params(labelbottom=False)
# # ax.yaxis.set_tick_params(labelleft=False)
# # plt.show()

# ##########################
# # Testing car kinematics #
# ##########################

# # T = 50
# # dt = 1

# # agents = [my_car]

# # v = 2

# # myRefTrace = [[v*i/sqrt(2), v*i/sqrt(2), pi/4] for i in range(ceil(T/dt)+1)]
# # yourRefTrace = [[-10, 10, 0] for i in range(ceil(T/dt)+1)]

# # def myStateRef(curr_idx):
# #     return myRefTrace[curr_idx]    

# # def myInputRef(t):
# #     return [v,0]


# # my_car.state_ref_traj = myStateRef
# # my_car.input_ref_traj = myInputRef

# # trackingSim = simulation()
# # trackingSim.addAgents(agents)
# # tracking_ref = trackingSim.runScenario(T, dt, True, False, [-10, 110], [-10, 110], [-10, 10], 'carSim.mp4')
