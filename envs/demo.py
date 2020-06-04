# Demo
# Written by: Kristina Miller
import numpy as np
import pypoman as ppm
import matplotlib.pyplot as plt

def problem():
    A_rect = np.array([[-1,0],
                       [1,0],
                       [0,-1],
                       [0,1]])
    b1 = np.array([[0.1], [0], [0], [10]])
    b2 = np.array([[-10], [10.1], [0], [10]])
    b3 = np.array([[0], [10], [0.1], [0]])
    b4 = np.array([[0], [10], [-10], [10.1]])
    b5 = np.array([[-3], [7], [-3], [7]])
    # TODO: Create more obstacles

    obstacles = [(A_rect, b1),
                 (A_rect, b2),
                 (A_rect, b3),
                 (A_rect, b4),
                 (A_rect, b5)]

    #TODO: Create initial set
    b0 = np.array([[-1.5], [2], [-1.5], [2]])
    Theta = (A_rect, b0)
    #TODO: Create goal set
    bg = np.array([[-8.5], [9.5], [-8.5], [9.5]])
    goal = (A_rect, bg)
    return obstacles , Theta, goal

if __name__ == '__main__':
    obs, Theta, goal = problem()

    for A,b in obs:
        ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(A,b), color = 'red')

    ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(goal[0],goal[1]), color = 'green')

    ppm.polygon.plot_polygon(ppm.duality.compute_polytope_vertices(Theta[0],Theta[1]), color = 'blue')
    plt.xlim(-0.2, 10.2)
    plt.ylim(-0.2, 10.2)
    plt.grid()
    plt.show()
