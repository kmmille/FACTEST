import sys, os
currFile = os.path.abspath(__file__)
modelPath = currFile.replace('/factest/synthesis/factest_base.py', '')
sys.path.append(modelPath)

import numpy as np
import polytope as pc
import z3

class FACTEST_Z3():
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

    def partition_polytope(self,poly):
        print('partitioning!')
        new_polys = []
        if self.dims != 2 and self.dims != 3:
            raise Exception('Can only handle workspaces of dimensions 2 or 3! Current workspace dimension: %s' %(self.dims))
        else:
            center = poly.chebXc
            x_center = np.array([center[0]])
            y_center = np.array([center[1]])
            if self.dims == 2:
                x_less_than = np.array([1,0])
                x_greater_than = np.array([-1,0])
                y_less_than = np.array([0,1])
                y_greater_than = np.array([0,-1])

                A_ul = np.vstack((poly.A, np.vstack((x_less_than, y_greater_than))))
                b_ul = np.hstack((poly.b, np.hstack((x_center, -1*y_center))))
                part_ul = pc.reduce(pc.Polytope(A_ul, b_ul))
                new_polys.append(part_ul)

                A_ur = np.vstack((poly.A, np.vstack((x_greater_than, y_greater_than))))
                b_ur = np.hstack((poly.b, np.hstack((-1*x_center, -1*y_center))))
                part_ur = pc.reduce(pc.Polytope(A_ur, b_ur))
                new_polys.append(part_ur)

                A_ll = np.vstack((poly.A, np.vstack((x_less_than, y_less_than))))
                b_ll = np.hstack((poly.b, np.hstack((x_center, y_center))))
                part_ll = pc.reduce(pc.Polytope(A_ll, b_ll))
                new_polys.append(part_ll)

                A_lr = np.vstack((poly.A, np.vstack((x_less_than, y_less_than))))
                b_lr = np.hstack((poly.b, np.hstack((x_center, y_center))))
                part_lr = pc.reduce(pc.Polytope(A_lr, b_lr))
                new_polys.append(part_lr)

                return new_polys

            else:
                raise Exception('Workspace dimension 3 not yet implemented!') #TODO: Implement the 3d polytope partition
                z_center = np.array([center[2]])
                x_less_than = np.array([1,0,0])
                x_greater_than = np.array([-1,0,0])
                y_less_than = np.array([0,1,0])
                y_greater_than = np.array([0,-1,0])
                z_less_than = np.array([0,0,1])
                z_greater_than = np.array([0,0,-1])

    def add_initial_constraints(self, init_poly):
        init_center = init_poly.chebXc
        for j in range(self.dims):
            self.s.add(self.x_ref_terms[0][j] == init_center[j])

    def add_goal_constraints(self, num_segs, err_bounds):
        A_goal = self.goal_poly.A
        b_goal = self.goal_poly.b

        err = err_bounds[-1]
        
        for row in range(len(A_goal)):
            A_row = A_goal[row]
            b_val = b_goal[row] #TODO: Need to deal with the bloating

            row_sum = 0
            for j in range(self.dims):
                row_sum += self.x_ref_terms[num_segs][j]*A_row[j]
            
            self.s.add(row_sum <= b_val)

    def add_unsafe_constraints(self, num_segs, err_bounds):
        for seg in range(num_segs):
            err = err_bounds[seg]
    
            for obstacle in self.unsafe_polys:
                A_obs = obstacle.A
                b_obs = obstacle.b

                obs_constraints = []
                for row in range(len(A_obs)):
                    A_row = A_obs[row]
                    b_val = b_obs[row] + np.linalg.norm(A_row)*err #TODO: Need to deal with the bloating

                    row_sum_0 = 0
                    row_sum_1 = 0
                    for j in range(self.dims):
                        row_sum_0 += self.x_ref_terms[seg][j]*A_row[j]
                        row_sum_1 += self.x_ref_terms[seg+1][j]*A_row[j]
                    row_constraint = z3.And(row_sum_0 > b_val, row_sum_1 > b_val)
                    obs_constraints.append(row_constraint)

                self.s.add(z3.Or(tuple(obs_constraints)))

    def add_workspace_constraints(self, num_segs):
        if type(self.workspace) != None:
            A_workspace = self.workspace.A
            b_workspace = self.workspace.b

            for i in range(num_segs+1):
                for row in range(len(A_workspace)):
                    A_row = A_workspace[row]
                    b_val = b_workspace[row] #TODO: Need to deal with the bloating

                    row_sum = 0
                    for j in range(self.dims):
                        row_sum += self.x_ref_terms[i][j]*A_row[j]

                    self.s.add(row_sum <= b_val)

    def get_xref(self, init_poly):
        for num_segs in range(1, self.seg_max+1):
            self.x_ref_terms = [[z3.Real('xref_%s[%s]'%(j+1,i)) for j in range(self.dims)] for i in range(num_segs+1)]
            self.s = z3.Solver()

            if type(self.model) != None:
                err_bounds = [self.model.errBound(init_poly, i) for i in range(num_segs)]
            else:
                err_bounds = [0 for i in range(num_segs)]
            
            self.add_initial_constraints(init_poly)
            self.add_goal_constraints(num_segs, err_bounds)
            self.add_unsafe_constraints(num_segs, err_bounds)
            self.add_workspace_constraints(num_segs)

            x_ref = None
            if self.s.check() == z3.sat:
                x_ref = []
                if self.print_statements:
                    print('SAT for %s segments' %(num_segs))
                m = self.s.model()
                for x_val_term in self.x_ref_terms:
                    x_val = [m[x_val_term[i]] for i in range(self.dims)]
                    x_ref.append([float(x_val[i].as_fraction()) for i in range(self.dims)])

                return x_ref
            else:
                if self.print_statements:
                    print('UNSAT for %s segments' %(num_segs))
                pass

        return x_ref

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
                    new_polys = self.partition_polytope(init_poly)
                    for poly in new_polys:
                        xref = self.get_xref(poly)
                        new_dict[i] = {'poly':poly, 'depth':depth+1, 'xref':xref}
                        i+=1
            
            self.initial_parts = new_dict

        return self.final_parts

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

    FACTEST_prob = FACTEST_Z3(initial_poly, goal_poly, unsafe_polys, workspace=workspace_poly)
    result_dict = FACTEST_prob.run()
    result_keys = list(result_dict.keys())
    xref = result_dict[result_keys[0]]['xref']

    xref_1 = [xval[0] for xval in xref]
    xref_2 = [xval[1] for xval in xref]

    fig, ax = plt.subplots()
    plotPoly(workspace_poly,ax,'yellow')
    plotPoly(initial_poly,ax,'blue')
    plotPoly(goal_poly,ax,'green')
    plotPoly(unsafe_polys,ax,'red')
    ax.plot(xref_1, xref_2, marker = 'o')
    ax.set_xlim(-10,10)
    ax.set_ylim(-10,10)
    plt.show()