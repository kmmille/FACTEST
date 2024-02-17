import sys, os
currFile = os.path.abspath(__file__)
modelPath = currFile.replace('/demo/demo_cav20.py', '')
sys.path.append(modelPath)

import argparse

#TODO: Update choices as new environments and models created
parser = argparse.ArgumentParser()
parser.add_argument("model", choices=['car']) #TODO: add options as more are created
parser.add_argument("env", choices=['maze_2d', 'scots_hscc16']) #TODO: add options as more are created
parser.add_argument("--solver", choices=['z3', 'yices', 'gurobi'], default='z3')
parser.add_argument("--segs", type=int, default=10)
parser.add_argument("--parts", type=int, default=2)
parser.add_argument("--print", type=bool, default=True)
parser.add_argument("--plot", type=bool, default=False)
args = parser.parse_args()

print('Running FACTEST for '+str(args.model)+' in '+str(args.env)+' with '+str(args.segs)+' segments and solver '+str(args.solver))
print('Plotting: '+str(args.plot))

#TODO: Load in different models based on what the user specified
if args.model == 'car':
    from demo.models.dubins_car import dubins_car
    model = dubins_car()

#TODO: Load in different environments based on what the user specified
if args.env == 'maze_2d':
    from demo.envs.maze_2d import Theta, G, O, workspace
elif args.env == 'scots_hscc16':
    from demo.envs.scots_hscc16 import Theta, G, O, workspace

#NOTE: Do not change the rest of this script
if args.solver == 'yices':
    from factest.synthesis.factest_base_yices import FACTEST_yices

    FACTEST_prob = FACTEST_yices(Theta, G, O, workspace, seg_max = args.segs, part_max = args.parts, print_statements = args.print)
    result_dict = FACTEST_prob.run()
    result_keys = list(result_dict.keys())
    xref = result_dict[result_keys[0]]['xref']
else:
    from factest.synthesis.factest_base import FACTEST_Z3 #TODO: Need to update as MILP solver becomes available

    FACTEST_prob = FACTEST_Z3(Theta, G, O, workspace = workspace, model = model, seg_max = args.segs, part_max = args.parts, print_statements = args.print)
    result_dict = FACTEST_prob.run()
    result_keys = list(result_dict.keys())
    xref = result_dict[result_keys[0]]['xref']

if args.plot:
    import matplotlib.pyplot as plt
    from factest.plotting.plot_polytopes import plotPoly

    xref_1 = [xval[0] for xval in xref]
    xref_2 = [xval[1] for xval in xref]

    fig, ax = plt.subplots()
    plotPoly(workspace,ax,'yellow')
    plotPoly(Theta,ax,'blue')
    plotPoly(G,ax,'green')
    plotPoly(O,ax,'red')
    ax.plot(xref_1, xref_2, marker = 'o')
    ax.set_xlim(-10,10)
    ax.set_ylim(-10,10)
    plt.show()
