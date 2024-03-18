import numpy as np
import polytope as pc

def partition_polytope(self,poly,dims):
        print('partitioning!')
        new_polys = []
        if dims != 2 and dims != 3:
            raise Exception('Can only handle workspaces of dimensions 2 or 3! Current workspace dimension: %s' %(dims))
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