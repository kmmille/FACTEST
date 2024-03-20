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
import polytope as pc

## Import FACTEST files ##
##########################
from factest.synthesis.factest_base_z3 import FACTEST_Z3
from factest.synthesis.factest_base_gurobi import dynamic_FACTEST_gurobi
from factest.synthesis.omega_factest_z3 import hybrid_from_ltl
from models.dubins_plane import dubins_plane
from models.dubins_car import dubins_car
#TODO: Need to make a 3d testing file

## Import plotting stuff ##
###########################
from factest.plotting.plot_polytopes import plotPoly


#########################
# TESTS THAT CAN BE RUN #
#########################
to_test = "plane"
to_test = "dynamic_simulation"
to_test = "omega"

#############################
# TESTING FOR DUBIN'S PLANE #
#############################
if to_test == "plane":
    print('testing plane')
    
    model = dubins_plane()

    state = [0,0,0,0,0]
    ref_state = [1,1,1,0,0]
    ref_input = [1,0,0,0]

    T = 10

    xref = [[1,1,1,0,0], [10,10,10,0,0]]
    vref = 1
    states = model.run_simulation(xref, vref, state, T)

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

##########################################################
# SOME TESTING STUFF FOR RUNNING TIMED/DYNAMIC SCENARIOS #
##########################################################
elif to_test == "dynamic_simulation":
    print('testing dynamic simulation')

    model = dubins_car()

    A = np.array([[-1, 0],
                    [ 1, 0],
                    [ 0,-1],
                    [0, 1]])
    b_init = np.array([[ -0.5], [ 0.75], [ -0.5], [ 0.75]])
    b_goal = np.array([[-4], [ 5], [-4], [ 5]])
    b_unsafe1 = np.array([[-3], [3.5], [   0], [5]])
    b_unsafe2 = np.array([[-3], [  7], [-5.5], [6]])
    b_unsafe3 = np.array([[-3], [  7], [   1], [0]])
    b_workspace = np.array([0,7,1,7])

    initial_poly = pc.Polytope(A, b_init)
    goal_poly = pc.Polytope(A, b_goal)
    unsafe_polys = [pc.Polytope(A, b_unsafe1), pc.Polytope(A, b_unsafe2), pc.Polytope(A, b_unsafe3)]
    workspace_poly = pc.Polytope(A, b_workspace)

    FACTEST_prob = dynamic_FACTEST_gurobi(initial_poly, goal_poly, unsafe_polys, workspace=workspace_poly, model=model)
    result_dict = FACTEST_prob.run()
    result_keys = list(result_dict.keys())
    xref = result_dict[result_keys[0]]['xref']

    model.set_timed_ref(xref)

    initial_state = [0.5,0.5,0]
    T = xref[-1][-1]

    states = model.run_simulation(xref, initial_state, T, vref = 1, sim_type = "timed")

    x_sim = [state[0] for state in states]
    y_sim = [state[1] for state in states]

    xref_1 = [xval[0] for xval in xref]
    xref_2 = [xval[1] for xval in xref]

    fig, ax = plt.subplots()
    plotPoly(workspace_poly,ax,'yellow')
    plotPoly(initial_poly,ax,'blue')
    plotPoly(goal_poly,ax,'green')
    plotPoly(unsafe_polys,ax,'red')
    ax.plot(xref_1, xref_2, marker = 'o')
    ax.plot(x_sim, y_sim, linestyle = '--')
    ax.set_xlim(-10,10)
    ax.set_ylim(-10,10)
    plt.show()

#############################
# Omega tracking simulation #
#############################
elif to_test == "omega":
    print("testing omega tracking")

    model = dubins_car()

    A = np.array([[-1,0],[1,0],[0,-1],[0,1]])

    b_goal1 = np.array([-5,7,-5,7])
    b_goal2 = np.array([7,-5,7,-5])

    b_unsafe1 = np.array([11,-10,11,11])
    b_unsafe2 = np.array([-10,11,11,11])
    b_unsafe3 = np.array([11,11,-10,11])
    b_unsafe4 = np.array([11,11,11,-10])
    b_unsafe5 = np.array([11,-2,1,1])
    b_unsafe6 = np.array([-2,11,1,1])

    b_workspace = np.array([10,10,10,10])

    workspace_poly = pc.Polytope(A, b_workspace)

    E1 = pc.Polytope(A, b_goal1) # goal set 1
    E2 = pc.Polytope(A, b_goal2) # goal set 2
    E3 = pc.Polytope(A, b_unsafe5) # unsafe set 1
    E4 = pc.Polytope(A, b_unsafe6) # unsafe set 2

    env = {'E1':E1,'E2':E2,'E3':E3,'E4':E4}   

    reach_str = 'F E1 & F E2 & G (E1 -> F E2) & G (E2 -> F E1)'
    avoid_str = 'G !E3 & G !E4'
    ltl_formula = reach_str + ' & ' + avoid_str

    model = dubins_car()
    myHybrid = hybrid_from_ltl(ltl_formula=ltl_formula,env=env, model=model, workspace=workspace_poly)

    curr_state = [6,6,0]

    all_states = model.run_omega_simulation(myHybrid, curr_state)

    fig = plt.figure()
    
    ax = fig.add_subplot(111)
    plotPoly(E1,ax,'green')
    plotPoly(E2,ax,'green')

    plotPoly(E3,ax,'red')
    plotPoly(E4,ax,'red')

    xsim = [state[0] for state in all_states]
    ysim = [state[1] for state in all_states]

    # xref = E2_flows[0]['xref']
    # xvals = [state[0] for state in xref]
    # yvals = [state[1] for state in xref]

    ax.plot(xsim,ysim, linestyle = '--')

    ax.set_xlim(-15,15)
    ax.set_ylim(-15,15)
    ax.xaxis.set_tick_params(labelbottom=False)
    ax.yaxis.set_tick_params(labelleft=False)

    plt.show()

    # ax = fig.add_subplot(222)
    # plotPoly(E1,ax,'green')
    # plotPoly(E2,ax,'green')

    # plotPoly(E3,ax,'red')
    # plotPoly(E4,ax,'red')

    # ax.set_xlim(-10,10)
    # ax.set_ylim(-10,10)
    # ax.xaxis.set_tick_params(labelbottom=False)
    # ax.yaxis.set_tick_params(labelleft=False)

    # ax = fig.add_subplot(223)
    # plotPoly(E1,ax,'green')
    # plotPoly(E2,ax,'green')

    # plotPoly(E3,ax,'red')
    # plotPoly(E4,ax,'red')

    # ax.set_xlim(-10,10)
    # ax.set_ylim(-10,10)
    # ax.xaxis.set_tick_params(labelbottom=False)
    # ax.yaxis.set_tick_params(labelleft=False)

    # ax = fig.add_subplot(224)
    # plotPoly(E1,ax,'green')
    # plotPoly(E2,ax,'green')

    # plotPoly(E3,ax,'red')
    # plotPoly(E4,ax,'red')

    # ax.set_xlim(-10,10)
    # ax.set_ylim(-10,10)
    # ax.xaxis.set_tick_params(labelbottom=False)
    # ax.yaxis.set_tick_params(labelleft=False)

    # plt.show()
