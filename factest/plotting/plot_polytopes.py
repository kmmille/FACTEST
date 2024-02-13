import polytope as pc
import numpy as np
from matplotlib.patches import Polygon
from scipy.spatial import ConvexHull
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def plotPoly(poly, ax = None, color = 'red'): # Takes in tulip polytope and plots it
    if ax == None:
        fig,ax = plt.subplots()

    if type(poly) != list:
        poly_verts = pc.extreme(poly)
        polyPatch = Polygon(poly_verts, facecolor = color, edgecolor = color, alpha = 0.25)
        ax.add_patch(polyPatch)

    else:
        for polygon in poly:
            poly_verts = pc.extreme(polygon)
            polyPatch = Polygon(poly_verts, facecolor = color, edgecolor = color, alpha = 0.25)
            ax.add_patch(polyPatch)


def plotPoly_3d(poly, ax, color='r'):
    cubes = [pc.extreme(poly)]
    for cube in cubes:
        hull = ConvexHull(cube)

        # draw the polygons of the convex hull
        for s in hull.simplices:
            tri = Poly3DCollection([cube[s]])
            tri.set_edgecolor(color)
            tri.set_facecolor(color)
            tri.set_alpha(0.5)
            ax.add_collection3d(tri)