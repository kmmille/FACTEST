import polytope as pc
import numpy as np

from math import sqrt

A = np.array([[-1, 0],
              [ 1, 0],
              [ 0,-1],
              [ 0, 1]])

b1 = np.array([-1,3,-0.9,1])
b2 = np.array([-1,5,-3.9,4])
b3 = np.array([-0.9,1,-2,3])
b4 = np.array([-1,2,-2.9,3])
b5 = np.array([-1.9,2,-2,3])
b6 = np.array([-2,4,-1.9,2])
b7 = np.array([-3.9,4,-1,3])
b8 = np.array([-2.9,3,-3,4])
b9 = np.array([-4,6,-0.9,1])
b10 = np.array([-4.9,5,-2,4])
b11 = np.array([-5,6,-1.9,2])
b13 = np.array([-4.9,5,0,1])
b12 = np.array([-5.9,6,-2,5])
b14 = np.array([0,7,0,0.1])
b15 = np.array([0,0.1,0,3])
b16 = np.array([0,0.1,-4,5])
b17 = np.array([0,6,-4.9,5])
b18 = np.array([-6.9,7,0,5])
# Border sets -- keeping for now but may not be used in the future
# b19 = np.array([0.1,0,0,5])
# b20 = np.array([-7,7.1,0,5])
# b21 = np.array([0,7,0.1,0])
# b22 = np.array([0,7,-5,5.1])

O1 = pc.Polytope(A, b1)
O2 = pc.Polytope(A, b2)
O3 = pc.Polytope(A, b3)
O4 = pc.Polytope(A, b4)
O5 = pc.Polytope(A, b5)
O6 = pc.Polytope(A, b6)
O7 = pc.Polytope(A, b7)
O8 = pc.Polytope(A, b8)
O9 = pc.Polytope(A, b9)
O10 = pc.Polytope(A, b10)
O11 = pc.Polytope(A, b11)
O12 = pc.Polytope(A, b12)
O13 = pc.Polytope(A, b13)
O14 = pc.Polytope(A, b14)
O15 = pc.Polytope(A, b15)
O16 = pc.Polytope(A, b16)
O17 = pc.Polytope(A, b17)
O18 = pc.Polytope(A, b18)
# Border sets -- keeping for now but may not be used in the future
# O19 = pc.Polytope(A, b19)
# O20 = pc.Polytope(A, b20)
# O21 = pc.Polytope(A, b21)
# O22 = pc.Polytope(A, b22)

O = [O1,O2,O3,O4,O5,O6,O7,O8,O9,O10,O11,O12,O13,O14,O15,O16,O17,O18] # Border sets: O19,O20,O21,O22]

b_init = np.array([-(0.5-0.2/sqrt(2)),0.5 + 0.2/sqrt(2),-(3.5 - 0.2/sqrt(2)), 3.5 + 0.2/sqrt(2)])
Theta = pc.Polytope(A, b_init)

b_goal = np.array([-6.25, 6.75, -4.5, 5.0])
G = pc.Polytope(A, b_goal)

b_workspace = np.array([0,7,0,5])
workspace = pc.Polytope(A, b_workspace)

if __name__=="__main__":
    import sys, os
    currFile = os.path.abspath(__file__)
    modelPath = currFile.replace('/demo/envs/maze_2d.py', '')
    sys.path.append(modelPath)

    import matplotlib.pyplot as plt

    from factest.plotting.plot_polytopes import plotPoly

    fig, ax = plt.subplots()

    plotPoly(workspace,ax,'yellow')

    plotPoly(G,ax,'green')
    plotPoly(Theta,ax,'blue')
    
    i = 1
    for obstacle in O:
        print('plotting poly #',i)
        plotPoly(obstacle,ax,'red')
        i+=1

    ax.set_xlim(-1,11)
    ax.set_ylim(-1,11)
    plt.show()