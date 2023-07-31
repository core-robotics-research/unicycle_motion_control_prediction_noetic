"""
Occupancy Grid Map Python Module

Author: Omur Arslan, omurarslan.phd@gmail.com
Modified by: Aykut Isleyen, a.isleyen@tue.nl
Created: March 2021
Modified: May 2023
"""

from __future__ import division
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import distance_transform_edt
from skimage.draw import polygon

class OccupancyGridMap2D:

    def __init__(self, width=100, height=100, resolution=1.0, origin=[0.0, 0.0]):

        # Private map properties that can be set once at the initialization
        self.__width = float(width) # Map width in meters
        self.__height = float(height) # Map height in meters
        self.__resolution = float(resolution) # Map resolution in meters
        self.__origin = np.array(origin, dtype=np.float64) # Map origin, the location of left-bottom corner, in meters

        # Public configurable map properties
        self.free_threshold = 0.2 # free probability threshold below which grid cells are considered obstacle-free
        self.occupied_threshold = 0.8 # occupied probability threshold above which grid cells are consider obstacle-occupied
        self.min_probability = 0.001 # Minimum grid cell probability to avoid oversaturation
        self.max_probability = 0.999 # Maximum grid cell probability  to avoid oversaturation

        # Dependent private map properties
        self.__rows = int(max(math.ceil(self.__height/self.__resolution), 1)) # Map grid row size
        self.__cols = int(max(math.ceil(self.__width/self.__resolution), 1)) # Map grid  column size

        # Private map properties
        self.__logodd_map = np.zeros((self.__cols, self.__rows), dtype=np.float64) # internal logodd representation of occupancy grid map
        self.__distance_field = np.zeros((self.__cols, self.__rows), dtype=np.float64) # Distance field (trasnform) of the binary occupancy grid map
        self.__nearest_neighbor = np.zeros((self.__cols, self.__rows), dtype=np.float64) # Linear (column-major) index of the nearest nonzero grid 
        self.__isvalid_distance_field = False # Checks if the distance field is up-to-date


    def width(self):
        # Returns map width in meters
        return self.__width

    def height(self):
        # Returns map height in meters
        return self.__height

    def resolution(self):
        # Returns map resolution in meters
        return self.__resolution

    def origin(self):
        # Returns the map origin, the left-bottom corner of the map, in meters
        return self.__origin

    def xworldlimits(self):
        # Returns the minimum and maximum x-coordinate limits of the map in meters
        return self.__origin[0] + np.array([0, self.__resolution*self.__cols])

    def yworldlimits(self):
        # Returns the minimum and maximum y-coordinate limits of the map in meters
        return self.__origin[1] + np.array([0, self.__resolution*self.__rows])

    def shape(self):
        # Returns the occupancy grid map shape, (rows, columns)
        return self.__logodd_map.shape

    def gridsize(self):
        # Returns the occupancy grid map size, (rows, columns)
        return self.__logodd_map.shape

    def world2grid(self, xy):
        # Converts world coordinates in meters to grid indices

        xy = np.reshape(xy, (-1,2))

        if xy.size == 0:
            ij = xy
        else:
            ij = np.floor((xy - self.__origin)/self.__resolution)
        ij = ij[:,[1, 0]]  # Convert from (columns,rows) to (rows, columns) order

        return np.int64(ij)

    def grid2world(self, ij):
        # Converts grid indices to world coordinates in meters

        ij = np.reshape(ij, (-1,2))
        ij = ij[:,[1, 0]] # Convert from (rows,columns) to (columns,rows) order

        if ij.size == 0:
            xy = np.empty((0,2), dtype=np.float64)
        else:
            xy = ij*self.__resolution + (self.__origin + 0.5*self.__resolution)

        return xy
    
    def poly2grid(self, polygon_vertices, frame):
        # Converts polygon to a binary mask in grid coordinates

        polygon_vertices = np.reshape(polygon_vertices, (-1,2))

        if np.char.lower(frame) == 'grid':
            polygon_vertices = np.array(polygon_vertices, dtype=np.int64)
        else:
            polygon_vertices = self.world2grid(polygon_vertices)

        # get the row and column indices of the polygon mask
        rr, cc = polygon(polygon_vertices[:,0], polygon_vertices[:,1], self.gridsize())

        # concatanate row and column indices as a n by 2 matrix
        ij = np.concatenate((rr.reshape(-1,1), cc.reshape(-1,1)), axis=1)

        return ij


    def get_occupancy(self, xy, frame='world'):
        """
        get_occupancy(ogm, xy) returns the occupancy probability values of the
        occupancy grid map ogm at the xy world locations in meters.

        get_occupancy(ogm, ij, 'grid') returns the occupancy probability values
        of the occupancy grid map ogm at the ij grid coordinates.
        """

        xy = np.reshape(xy,(-1,2))

        if np.char.lower(frame) == 'grid':
            ij = np.array(xy, dtype=np.int64)
        else:
            ij = self.world2grid(xy)

        occval = np.full((ij.shape[0],1), np.nan)

        I = self.__isvalid_grid(ij)

        logodd = self.__logodd_map[ij[I,0], ij[I,1]]
        occval[I,0] = self.__logodd2prob(logodd)

        return occval

    def set_occupancy(self, xy, occval, frame='world'):
        """
        set_occupancy(ogm, xy, occval) sets the occupancy probability values of
        the occupancy grid map ogm at the xy world coordinates with an array of
        occupancy probabilities occval

        set_occupancy(ogm, ij, occval, 'grid') sets the occupancy probability
        values of the occupancy grid map ogm at the ij grid coordinates with
        an array of occupancy probabilities occval
        """

        xy = np.reshape(xy, (-1,2))

        if np.char.lower(frame) == 'grid':
            ij = np.int64(xy)
        else:
            ij = self.world2grid(xy)

        I = self.__isvalid_grid(ij)
        self.__logodd_map[ij[I,0], ij[I,1]] = self.__prob2logodd(occval)

        self.__isvalid_distance_field = False

    def update_occupancy(self, xy, occval, frame='world'):
        """
        update_occupancy(ogm, xy, occval) integrates the occupancy probability
        values occval to the occupancy grid map ogm at the specified xy world
        coordinates

        update_occupancy(ogm, ij, occval, 'grid') integrates the occupancy
        probability values occval to the occupancy grid map ogm at the specified
        ij grid  coordinates
        """

        xy = np.reshape(xy, (-1,2))

        if np.char.lower(frame) == 'grid':
            ij = np.int64(xy)
        else:
            ij = self.world2grid(xy)

        I = self.__isvalid_grid(ij)
        self.__logodd_map[ij[I,0], ij[I,1]] = self.__logodd_map[ij[I,0], ij[I,1]] + self.__prob2logodd(occval)

        self.__isvalid_distance_field = False

    def occupancy_matrix(self):
        # Returns the occupancy matrix reprensetation of the occupancy grid map

        mat = self.__logodd2prob(self.__logodd_map)

        return mat

    def raycast(self, pose, range, angle):
        """
        midgrids, endgrids = raycast(map, pose, range, angle) computes the
        midpoint and end point cell indices of a map along a ray starting at a
        specific pose with specific range and angle
        """

        pose = np.reshape(pose, (-1,))[0:3]
        range = np.reshape(range, (-1,))[0]
        angle = np.reshape(angle, (-1,))[0]

        position = pose[0:2]
        orientation = pose[2]
        rayvect = np.hstack((np.cos(orientation+angle), np.sin(orientation+angle)))

        startpt = position
        endpoint = position + range*rayvect

        startgrid = self.world2grid(startpt)[0]
        endgrid = self.world2grid(endpoint)[0]

        numsample = np.floor(np.sqrt(np.sum(np.power(endgrid - startgrid,2)))).astype('int64') + 2

        xraygrids = np.round(np.linspace(startgrid[0], endgrid[0], numsample)).astype('int64')
        yraygrids = np.round(np.linspace(startgrid[1], endgrid[1], numsample)).astype('int64')

        midgrids = np.column_stack((xraygrids[0:(numsample-1)],yraygrids[0:(numsample-1)]))
        endgrids = np.array([xraygrids[numsample-1], yraygrids[numsample-1]])

        return midgrids, endgrids

    def rayinsert(self, pose, ranges, angles, maxrange=10.0):
        """
        rayinsert(map, pose, ranges, angles, maxrange)
        integrates scan observations at a specific pose with specific ranges and
        angles assuming a certain maxrange and inverse sensor model with
        obstacle-free sensing probability self.free_threshold and obstacle-occupied sensing
        probability self.occupied_threshold
        """

        ranges = np.reshape(ranges, (-1,))
        angles = np.reshape(angles, (-1,))

        # Limit maximum sensor readings
        ranges[ranges>maxrange] = maxrange

        # Limit sensing range
        ranges[ranges > maxrange] = maxrange

        # Integrate each observation in the occupancy grid map
        for k in np.arange(np.size(ranges)):
            midgrids, endgrids = self.raycast(pose, ranges[k], angles[k])
            self.update_occupancy(midgrids, self.free_threshold, 'grid')
            if ranges[k] < maxrange:
                self.update_occupancy(endgrids, self.occupied_threshold, 'grid')

    
    def distance_field(self):
        # Computes the distance field of the occupancy grid map

        if self.__isvalid_distance_field:
            return self.__distance_field, self.__nearest_neighbor
        else:
            # Convert the occupancy grid map to a binary matrix using self.occupied_threshold
            bin_occ_mat = self.occupancy_matrix()
            bin_occ_mat[bin_occ_mat < self.occupied_threshold] = 0
            bin_occ_mat[bin_occ_mat >= self.occupied_threshold] = 1

            # Invert the occupancy grid map to compute the distance transform
            inv_mat = np.logical_not(bin_occ_mat)


            # First check if any of the cells is occupied. If not, "distance_transform_edt" will return error
            if np.all(inv_mat):
                D = np.zeros(np.shape(bin_occ_mat)) + 1e10
                IDX = np.zeros(np.shape(bin_occ_mat), dtype=int)
            else:
                # Compute the distance transform with the indices of the closest occupied cells to each cell
                D, IDX = distance_transform_edt(inv_mat, return_indices=True)
                # Convert the nearest occupied cell subscripts to column-major indices (same as the sub2ind function in MATLAB)
                IDX = np.ravel_multi_index(IDX, np.shape(bin_occ_mat), order='F')

            # Update the private properties of distance field and nearest neighbor indices
            self.__nearest_neighbor = IDX
            self.__distance_field = D
            self.__isvalid_distance_field = True

            print("Distance field is updated")

            return D, IDX
        
    def distance_to_collision(self, position, frame='world'):
        """
        D, X = dist2collision(map, pose) computes the distance to collision D
        and the collision point X in the given coordinate frame at a specific
        pose in the occupancy grid map
        D : Distance to collision at given position
        X : Collision point in the given coordinate frame
        frame : Coordinate frame of input position, 'world' (default) | 'grid'
        """



        position = np.reshape(position, (-1, 2))

        if np.char.lower(frame) == 'grid':
            position = np.int64(position)
        else:
            position = self.world2grid(position)

        IN = self.__isvalid_grid(position)

        D = np.zeros((position.shape[0], 1))
        X = np.zeros((position.shape[0], 2))

        # Compute distance field and nearest neighbors
        DF, NN = self.distance_field()

        if np.any(IN):
            # Get the distance field values at the given positions and their nearest neighbors if they are inside the map    
            D[IN] = np.reshape(DF[position[IN, 0], position[IN, 1]], (-1, 1))
            X[IN, :] = np.transpose(np.unravel_index(NN[position[IN, 0], position[IN, 1]], np.shape(DF), order='F'))

        if np.any(~IN):
            # Positions outside the map if any with if statement
            OUT = np.logical_not(IN)
            X[OUT, :] = np.minimum(np.maximum(position[OUT, :], 1), np.shape(DF))
            D[OUT] = np.reshape(-np.sqrt(np.sum((position[OUT, :] - X[OUT, :]) ** 2, axis=1)) * self.__resolution, (-1, 1) )

        # Convert collision points back to the specified coordinate frame
        if np.char.lower(frame) == 'world':
            X = self.grid2world(X)

        D = D * self.__resolution

        return D, X

    def distance_to_collision_polygon(self, polygon_vertices, frame='world'):
        """
        Computes the distance to collision of a polygon with the occupancy grid map
        D, XPolygon, XMap = dist2collision(map, polygon_vertices) computes the distance to collision D
        and the closest points XPolygon and XMap in the given coordinate frame

        D : Distance to collision of the given polygon with the occupancy grid map
        XPolygon : Points cloest to the occupancy grid map in the polygon
        XMap : Points closest to the polygon in the occupancy grid map
        frame : Coordinate frame of input position, 'world' (default) | 'grid'
        """

        polygon_vertices = np.reshape(polygon_vertices, (-1, 2))

        # Get polygon indices
        polygon_mask= self.poly2grid(polygon_vertices, frame)

        # Get distance to collision of the polygon mask
        distance, XMap = self.distance_to_collision(polygon_mask, 'grid')


        distance, I = np.min(distance), np.argmin(distance)
        XMap = XMap[I, :]
        XPolygon = polygon_mask[I, :]

        # Convert collision points back to the specified coordinate frame
        if np.char.lower(frame) == 'world':
            XMap = self.grid2world(XMap)
            XPolygon = self.grid2world(XPolygon)

        return distance, XPolygon, XMap

    """ Private Methods"""
    def __isvalid_grid(self, ij):
        # Checks if given grid indices are valid
        ij = np.reshape(ij, (-1,2))
        I = (ij[:,0]>= 0) & (ij[:,0]< self.__rows) & (ij[:,1]>=0) & (ij[:,1] < self.__cols)
        return I

    def __logodd2prob(self, logodd):
        # Converts from log-odd values to probability values

        min_logodd =  np.log(self.min_probability/(1.0-self.min_probability))
        max_logodd =  np.log(self.max_probability/(1.0-self.max_probability))

        logodd = np.maximum(logodd, min_logodd)
        logodd = np.minimum(logodd, max_logodd)

        odd = np.exp(logodd)
        prob = odd/(1.0 + odd)

        prob = np.maximum(prob, self.min_probability)
        prob = np.minimum(prob, self.max_probability)

        return prob

    def __prob2logodd(self, prob):
        # Converts from probability values to log-odd values

        prob = np.maximum(prob, self.min_probability)
        prob = np.minimum(prob, self.max_probability)
        logodd = np.log(prob/(1.0 - prob))

        return logodd

    """
    ## To-Be-Added Future Functions ##

    def plot(self):

        mat = self.occupancy_matrix()

        extent = np.concatenate((self.xworldlimits(),self.yworldlimits()))

        plt.imshow(mat, cmap='gray', extent=np.concatenate((self.xworldlimits(),self.yworldlimits())))
        plt.ylim(self.yworldlimits())
        plt.xlim(self.xworldlimits())
        plt.show()
    """
