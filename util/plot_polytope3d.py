# Plot polytope in 3d
# Written by: Kristina Miller

import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d as a3

from scipy.spatial import ConvexHull
import pypoman as ppm

class Faces():
	def __init__(self,tri, sig_dig=12, method="convexhull"):
		self.method=method
		self.tri = np.around(np.array(tri), sig_dig)
		self.grpinx = list(range(len(tri)))
		norms = np.around([self.norm(s) for s in self.tri], sig_dig)
		_, self.inv = np.unique(norms,return_inverse=True, axis=0)

	def norm(self,sq):
		cr = np.cross(sq[2]-sq[0],sq[1]-sq[0])
		return np.abs(cr/np.linalg.norm(cr))

	def isneighbor(self, tr1,tr2):
		a = np.concatenate((tr1,tr2), axis=0)
		return len(a) == len(np.unique(a, axis=0))+2

	def order(self, v):
		if len(v) <= 3:
			return v
		v = np.unique(v, axis=0)
		n = self.norm(v[:3])
		y = np.cross(n,v[1]-v[0])
		y = y/np.linalg.norm(y)
		c = np.dot(v, np.c_[v[1]-v[0],y])
		if self.method == "convexhull":
			h = ConvexHull(c)
			return v[h.vertices]
		else:
			mean = np.mean(c,axis=0)
			d = c-mean
			s = np.arctan2(d[:,0], d[:,1])
			return v[np.argsort(s)]

	def simplify(self):
		for i, tri1 in enumerate(self.tri):
			for j,tri2 in enumerate(self.tri):
				if j > i:
					if self.isneighbor(tri1,tri2) and \
					   self.inv[i]==self.inv[j]:
						self.grpinx[j] = self.grpinx[i]
		groups = []
		for i in np.unique(self.grpinx):
			u = self.tri[self.grpinx == i]
			u = np.concatenate([d for d in u])
			u = self.order(u)
			groups.append(u)
		return groups

def plot_polytope_3d(A, b, ax = None, color = 'red', trans = 0.2):
	verts = np.array(ppm.compute_polytope_vertices(A, b))
	# compute the triangles that make up the convex hull of the data points
	hull = ConvexHull(verts)
	triangles = [verts[s] for s in hull.simplices]
	# combine co-planar triangles into a single face
	faces = Faces(triangles, sig_dig=1).simplify()
	# plot
	if ax == None:
		ax = a3.Axes3D(plt.figure())

	pc = a3.art3d.Poly3DCollection(faces,
								   facecolor=color,
								   edgecolor="k", alpha=trans)
	ax.add_collection3d(pc)
	# define view
	yllim, ytlim = ax.get_ylim()
	xllim, xtlim = ax.get_xlim()
	zllim, ztlim = ax.get_zlim()
	x = verts[:,0]
	x = np.append(x, [xllim, xtlim])
	y = verts[:,1]
	y = np.append(y, [yllim, ytlim])
	z = verts[:,2]
	z = np.append(z, [zllim, ztlim])
	ax.set_xlim(np.min(x)-1, np.max(x)+1)
	ax.set_ylim(np.min(y)-1, np.max(y)+1)
	ax.set_zlim(np.min(z)-1, np.max(z)+1)


if __name__ == '__main__':
	A = np.array([[-1, 0, 0],
				  [1, 0, 0],
				  [0, -1, 0],
				  [0, 1, 0],
				  [0, 0, -1],
				  [0, 0, 1]])
	b = np.array([[1], [1], [1], [1], [1], [1]])
	b2 = np.array([[-1], [2], [-1], [2], [-1], [2]])
	ax1 = a3.Axes3D(plt.figure())
	plot_polytope_3d(A, b, ax = ax1, color = 'red')
	plot_polytope_3d(A, b2, ax = ax1, color = 'green')
	plt.show()