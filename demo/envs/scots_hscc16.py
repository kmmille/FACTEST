import polytope as pc
import numpy as np

A = np.array([[-1, 0],
              [ 1, 0],
              [ 0,-1],
              [ 0, 1]])

b1 = np.array([-1,1.2,0,9])
b2 = np.array([-2.2,2.4,-0,5])
b3 = np.array([-2.2,2.4,-6,10])
b4 = np.array([-3.4,3.6,-0,9])
b5 = np.array([-4.6 ,4.8,-1,10])
b6 = np.array([-5.8,6,-0,6])
b7 = np.array([-5.8,6,-7,10])
b8 = np.array([-7,7.2,-1,10])
b9 = np.array([-8.2,8.4,-0,8.5])
b10 = np.array([-8.4,9.3,-8.3,8.5])
b11 = np.array([-9.3,10,-7.1,7.3])
b12 = np.array([-8.4,9.3,-5.9,6.1])
b13 = np.array([-9.3,10 ,-4.7,4.9])
b14 = np.array([-8.4,9.3,-3.5,3.7])
b15 = np.array([-9.3,10 ,-2.3,2.5])

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

O = [O1, O2, O3, O4, O5, O6, O7, O8, O9, O10, O11, O12, O13, O14, O15]

b_goal = np.array([-9.3,9.7,-0.4,0.8])
G = pc.Polytope(A, b_goal)

b_init = np.array([-0.3, 0.7, -0.3, 0.7])
Theta = pc.Polytope(A, b_init)

b_workspace = np.array([0,10,0,10])
workspace = pc.Polytope(A, b_workspace)

if __name__=="__main__":
    import sys, os
    currFile = os.path.abspath(__file__)
    modelPath = currFile.replace('/demo/envs/scots_hscc16.py', '')
    sys.path.append(modelPath)

    import matplotlib.pyplot as plt

    from factest.plotting.plot_polytopes import plotPoly

    fig, ax = plt.subplots()

    plotPoly(workspace,ax,'yellow')

    plotPoly(G,ax,'green')
    plotPoly(Theta,ax,'blue')
    for obstacle in O:
        plotPoly(obstacle,ax,'red')

    ax.set_xlim(-1,11)
    ax.set_ylim(-1,11)
    plt.show()
