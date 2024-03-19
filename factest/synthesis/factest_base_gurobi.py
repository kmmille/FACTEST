import sys, os
currFile = os.path.abspath(__file__)
modelPath = currFile.replace('/factest/synthesis/factest_base_gurobi.py', '')
sys.path.append(modelPath)

import numpy as np
import polytope as pc
import gurobipy as gp

from common_functions import partition_polytope

class FACTEST_gurobi():
    def __init__(self, initial_poly, goal_poly, unsafe_polys, model = None, workspace = None, seg_max = 3, part_max = 2, print_statements = True):
        self.initial_parts = {0:{'poly':initial_poly,'depth':0, 'xref':None}}
        self.final_parts = {}
        self.goal_poly = goal_poly
        self.unsafe_polys = unsafe_polys
        self.workspace = workspace

        self.model = model

        self.dims = len(self.goal_poly.A[0])
        self.seg_max = seg_max
        self.part_max = part_max
        self.print_statements = print_statements

    def add_initial_constraints(self, init_poly):
        init_center = init_poly.chebXc

        for j in range(self.dims):
            self.s.addConstr(self.xlist[0][j] == init_center[j])

    def add_goal_constraints(self, num_segs, err_bounds):
        A_goal = self.goal_poly.A
        b_goal = self.goal_poly.b

        err = err_bounds[-1]
        
        for row in range(len(A_goal)):
            A_row = A_goal[row]
            b_val = b_goal[row] - np.linalg.norm(A_row)*err

            row_sum = 0
            for j in range(self.dims):
                row_sum += self.xlist[num_segs][j]*A_row[j]
            
            self.s.addConstr(row_sum <= b_val)

    def add_unsafe_constraints(self, num_segs, err_bounds):
        for seg in range(num_segs):
            err = err_bounds[seg]
    
            for obstacle in self.unsafe_polys:
                A_obs = obstacle.A
                b_obs = obstacle.b

                edges = len(b_obs)
                alpha = self.s.addVars(edges, vtype = gp.GRB.BINARY)
                M = 1e3

                for row in range(len(A_obs)):
                    A_row = A_obs[row]
                    b_val = b_obs[row] + np.linalg.norm(A_row)*err

                    row_sum_0 = 0
                    row_sum_1 = 0
                    for j in range(self.dims):
                        row_sum_0 += self.xlist[seg][j]*A_row[j]
                        row_sum_1 += self.xlist[seg+1][j]*A_row[j]

                    self.s.addConstr(b_val - row_sum_0 <= M*(1 - alpha[row]))
                    self.s.addConstr(b_val - row_sum_1 <= M*(1 - alpha[row]))

                self.s.addConstr(alpha.sum() >= 1)

    def add_workspace_constraints(self, num_segs, err_bounds):
        if type(self.workspace) != None:
            A_workspace = self.workspace.A
            b_workspace = self.workspace.b

            for i in range(num_segs+1):
                err = err_bounds[i]
                for row in range(len(A_workspace)):
                    A_row = A_workspace[row]
                    b_val = b_workspace[row] - np.linalg.norm(A_row)*err

                    row_sum = 0
                    for j in range(self.dims):
                        row_sum += self.xlist[i][j]*A_row[j]

                    self.s.addConstr(row_sum <= b_val)

    def get_xref(self, init_poly):
        for num_segs in range(1, self.seg_max+1):

            self.xlist = []
            self.s = gp.Model("xref")
            self.s.setParam(gp.GRB.Param.OutputFlag, 0)
            
            # Setting the objective and creating xref variables #
            #####################################################
            obj = 0
            for i in range(num_segs+1):
                xnew = self.s.addVars(self.dims)
                self.xlist.append(xnew) 

                if i > 0:
                    if self.dims == 2:
                        tem_obj = (self.xlist[i][0] - self.xlist[i-1][0])*(self.xlist[i][0] - self.xlist[i-1][0]) + (self.xlist[i][1] - self.xlist[i-1][1])*(self.xlist[i][1] - self.xlist[i-1][1])
                    elif self.dims == 3:
                        tem_obj = (self.xlist[i][0] - self.xlist[i-1][0])*(self.xlist[i][0] - self.xlist[i-1][0]) + (self.xlist[i][1] - self.xlist[i-1][1])*(self.xlist[i][1] - self.xlist[i-1][1]) + (self.xlist[i][2] - self.xlist[i-1][2])*(self.xlist[i][2] - self.xlist[i-1][2])

                    obj += tem_obj

            self.s.setObjective(obj, gp.GRB.MINIMIZE)
            self.s.setParam(gp.GRB.Param.OutputFlag, 0)
            
            if self.model != None:
                err_bounds = [self.model.errBound(init_poly, i) for i in range(num_segs+1)]
            else:
                err_bounds = [0 for i in range(num_segs+1)]

            self.add_workspace_constraints(num_segs, err_bounds)
            self.add_initial_constraints(init_poly)
            self.add_goal_constraints(num_segs, err_bounds)
            self.add_unsafe_constraints(num_segs, err_bounds)

            self.s.update()
            self.s.optimize()

            try:
                wps_list = []
                for x in self.xlist:
                    if self.dims == 2:
                        x_pt = x[0].X
                        y_pt = x[1].X
                        
                        wps_list.append([x_pt, y_pt])
                    else:
                        x_pt = x[0].X
                        y_pt = x[1].X
                        z_pt = x[2].X

                        wps_list.append([x_pt, y_pt, z_pt])
                self.s.dispose()
                return wps_list
            
            except:
                self.s.dispose()


    def run(self, force_partition = False):
        if not force_partition:
            init_poly = self.initial_parts[0]['poly']
            depth = self.initial_parts[0]['depth']
            
            xref = self.get_xref(init_poly)

            self.initial_parts[0] = {'poly':init_poly, 'depth':depth, 'xref':xref}

        k = 0
        while len(self.final_parts.keys()) != len(self.initial_parts.keys()):
            poly_keys = list(self.initial_parts.keys())
            i = 0
            new_dict = {}
            for key in poly_keys:
                init_poly = self.initial_parts[key]['poly']
                depth = self.initial_parts[key]['depth']
                xref = self.initial_parts[key]['xref']

                if depth >= self.part_max or xref != None:
                    self.final_parts[k] = self.initial_parts[key]
                    k += 1
                    new_dict[i] = self.initial_parts[key]
                    i += 1
                else:
                    new_polys = partition_polytope(init_poly, self.dims)
                    for poly in new_polys:
                        xref = self.get_xref(poly)
                        new_dict[i] = {'poly':poly, 'depth':depth+1, 'xref':xref}
                        i+=1
            
            self.initial_parts = new_dict

        return self.final_parts

#####################################
# Timed version for dynamic FACTEST #
#####################################
class dynamic_FACTEST_gurobi(FACTEST_gurobi):
    def __init__(self, initial_poly, goal_poly, unsafe_polys, timed_unsafe_polys = [], dt = 1, max_length = 5, model=None, workspace=None, seg_max=6, part_max=2, print_statements=True):
        super().__init__(initial_poly, goal_poly, unsafe_polys, model, workspace, seg_max, part_max, print_statements)

        self.dt = dt
        self.max_length = max_length
        self.timed_unsafe_polys = timed_unsafe_polys

    def add_timing_constraints(self, num_segs):
        self.s.addConstr(self.xlist[0][self.dims] == 0)
        for i in range(1,num_segs+1):
            self.s.addConstr(self.xlist[i][self.dims] == self.xlist[i-1][self.dims] + self.dt)

    def add_length_constraints(self, num_segs):
        for i in range(1, num_segs+1):
            self.s.addConstr((self.xlist[i][0] - self.xlist[i-1][0])**2 + (self.xlist[i][1] - self.xlist[i-1][1])**2 <= self.max_length**2)

    def add_timed_unsafe_constraints(self, num_segs, err_bounds):
        for seg in range(num_segs):
            err = err_bounds[seg]
    
            for obstacle in self.timed_unsafe_polys:
                A_obs = obstacle.A
                b_obs = obstacle.b

                edges = len(b_obs)
                alpha = self.s.addVars(edges, vtype = gp.GRB.BINARY)
                M = 1e3

                for row in range(len(A_obs)):
                    A_row = A_obs[row]
                    b_val = b_obs[row] + np.linalg.norm(A_row)*err

                    row_sum_0 = 0
                    row_sum_1 = 0
                    for j in range(self.dims+1):
                        row_sum_0 += self.xlist[seg][j]*A_row[j]
                        row_sum_1 += self.xlist[seg+1][j]*A_row[j]

                    self.s.addConstr(b_val - row_sum_0 <= M*(1 - alpha[row]))
                    self.s.addConstr(b_val - row_sum_1 <= M*(1 - alpha[row]))

                self.s.addConstr(alpha.sum() >= 1)

    def get_xref(self, init_poly):
        for num_segs in range(1, self.seg_max+1):

            self.xlist = []
            self.s = gp.Model("xref")
            self.s.setParam(gp.GRB.Param.OutputFlag, 0)
            
            # Setting the objective and creating xref variables #
            #####################################################
            obj = 0
            for i in range(num_segs+1):
                xnew = self.s.addVars(self.dims+1)
                self.xlist.append(xnew) 

                if i > 0:
                    if self.dims == 2:
                        tem_obj = (self.xlist[i][0] - self.xlist[i-1][0])*(self.xlist[i][0] - self.xlist[i-1][0]) + (self.xlist[i][1] - self.xlist[i-1][1])*(self.xlist[i][1] - self.xlist[i-1][1])
                    elif self.dims == 3:
                        tem_obj = (self.xlist[i][0] - self.xlist[i-1][0])*(self.xlist[i][0] - self.xlist[i-1][0]) + (self.xlist[i][1] - self.xlist[i-1][1])*(self.xlist[i][1] - self.xlist[i-1][1]) + (self.xlist[i][2] - self.xlist[i-1][2])*(self.xlist[i][2] - self.xlist[i-1][2])

                    obj += tem_obj

            self.s.setObjective(obj, gp.GRB.MINIMIZE)
            self.s.setParam(gp.GRB.Param.OutputFlag, 0)
            
            if self.model != None:
                err_bounds = [self.model.errBound(init_poly, i) for i in range(num_segs+1)]
            else:
                err_bounds = [0 for i in range(num_segs+1)]

            self.add_workspace_constraints(num_segs, err_bounds)
            self.add_initial_constraints(init_poly)
            self.add_goal_constraints(num_segs, err_bounds)
            self.add_unsafe_constraints(num_segs, err_bounds)
            self.add_timing_constraints(num_segs)
            self.add_length_constraints(num_segs)
            self.add_timed_unsafe_constraints(num_segs, err_bounds)

            self.s.update()
            self.s.optimize()

            try:
                wps_list = []
                for x in self.xlist:
                    if self.dims == 2:
                        x_pt = x[0].X
                        y_pt = x[1].X
                        t_pt = x[2].X
                        
                        wps_list.append([x_pt, y_pt, t_pt])
                    else:
                        x_pt = x[0].X
                        y_pt = x[1].X
                        z_pt = x[2].X
                        t_pt = x[3].X

                        wps_list.append([x_pt, y_pt, z_pt, t_pt])
                self.s.dispose()
                return wps_list
            
            except:
                self.s.dispose()

if __name__=="__main__":
    #TODO: Make sure that this section is clean and works with current FACTEST setup
    import matplotlib.pyplot as plt
    from factest.plotting.plot_polytopes import plotPoly
    print('testing!')

    A = np.array([[-1, 0],
                  [ 1, 0],
                  [ 0,-1],
                  [0, 1]])
    b_init = np.array([[ 0], [ 1], [ 0], [ 1]])
    b_goal = np.array([[-4], [ 5], [-4], [ 5]])
    b_unsafe1 = np.array([[-3], [3.5], [   0], [5]])
    b_unsafe2 = np.array([[-3], [  7], [-5.5], [6]])
    b_unsafe3 = np.array([[-3], [  7], [   1], [0]])
    b_workspace = np.array([0,7,1,7])

    initial_poly = pc.Polytope(A, b_init)
    goal_poly = pc.Polytope(A, b_goal)
    unsafe_polys = [pc.Polytope(A, b_unsafe1), pc.Polytope(A, b_unsafe2), pc.Polytope(A, b_unsafe3)]
    workspace_poly = pc.Polytope(A, b_workspace)

    # FACTEST_prob = FACTEST_gurobi(initial_poly, goal_poly, unsafe_polys, workspace=workspace_poly)
    FACTEST_prob = dynamic_FACTEST_gurobi(initial_poly, goal_poly, unsafe_polys, workspace=workspace_poly)
    result_dict = FACTEST_prob.run()
    result_keys = list(result_dict.keys())
    xref = result_dict[result_keys[0]]['xref']

    xref_1 = [xval[0] for xval in xref]
    xref_2 = [xval[1] for xval in xref]
    tref = [xval[2] for xval in xref]

    print(xref_1, xref_2, tref)

    fig, ax = plt.subplots()
    plotPoly(workspace_poly,ax,'yellow')
    plotPoly(initial_poly,ax,'blue')
    plotPoly(goal_poly,ax,'green')
    plotPoly(unsafe_polys,ax,'red')
    ax.plot(xref_1, xref_2, marker = 'o')
    ax.set_xlim(-10,10)
    ax.set_ylim(-10,10)
    plt.show()