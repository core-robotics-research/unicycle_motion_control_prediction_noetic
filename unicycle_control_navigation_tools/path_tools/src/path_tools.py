"""
Python Module for Path Tools typically used by the Reference Governor Framework
Author: Aykut Isleyen, a.isleyen@tue.nl
Created: May 2023
"""

import numpy as np


def path_distance(path, position):
    """
    Computes the distance of a point to a piecewise linear path

    Usage:
        distance, coords = path_distance(path, position)
    Input:
        path - Piecewise linear path points in n-dimensional space, m x n matrix
        position - Point in n-dimensional space, k x n vector
    Output:
        distance - Distance of the point to the path, k x 1 vector
        coords - Coordinates of the closest point on the path, k x n matrix
    Example:
        path = np.random.rand(10,2)
        position = np.random.rand(5,2)
        distance, coords = path_distance(path, position)
    """

    position = np.array(position)
    path = np.array(path)


    if path.shape[0] == 0:
            distance = np.full((position.shape[0],1), np.nan)
            coords = np.full((position.shape[0], path.shape[1]), np.nan)
    elif path.shape[0] == 1:
            distance = np.sqrt(np.sum((position - path)**2, axis=1))
            distance = distance.reshape((distance.shape[0],1))
            coords = np.tile(path, (position.shape[0], 1))
    else:

            distance = np.full((position.shape[0],1), np.nan)
            coords = np.full((position.shape[0], path.shape[1]), np.nan)

            Pbase = path[0:-1,:]
            Pdiff = np.diff(path, axis=0)
            Pmag2 = np.sum(Pdiff**2, axis=1)

            goal = position.copy()
            for k in range(position.shape[0]):
                    Gbase = position[k,:] - Pbase
                    Gmag2 = np.sum(Gbase**2, axis=1)

                    GProj = np.sum(Pdiff*Gbase, axis=1)
                    W = GProj/Pmag2
                    W = np.maximum(np.minimum(W, 1), 0)
                    D = (W**2)*Pmag2 - 2*W*GProj + Gmag2
                    I = np.where(D == np.min(D))[0]
                    I = I[-1]

                    distance[k] = np.sqrt(D[I])

                    w1 = GProj[I]/Pmag2[I]
                    w1 = np.maximum(np.minimum(w1, 1), 0)

                    coords[k,:] = w1*Pdiff[I,:] + Pbase[I,:]

    return distance, coords
