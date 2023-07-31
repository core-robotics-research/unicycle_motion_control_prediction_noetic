import numpy as np
from scipy.spatial import ConvexHull


def Convhull(points):
    
    H = ConvexHull(points)

    return H


def convexhull_polygon(points):

    points = np.array(points, dtype=np.float64)

    # add a small random number to all points to avoid singularities
    points += np.random.rand(points.shape[0], points.shape[1]) * 1e-6
        
    H = Convhull(points)

    polygon = H.points[H.vertices]
    
    return polygon


def circle_polygon(radius, resolution):

    # check if radius is zero if it is then return a single point
    if radius == 0:
        polygon = np.array([[0, 0]])
        return polygon

    angles = np.linspace(0, 2 * np.pi, resolution, endpoint=False)
    x = radius * np.cos(angles)
    y = radius * np.sin(angles)
    circle_vertices = np.column_stack((x, y))
    polygon = convexhull_polygon(circle_vertices)

    return polygon

