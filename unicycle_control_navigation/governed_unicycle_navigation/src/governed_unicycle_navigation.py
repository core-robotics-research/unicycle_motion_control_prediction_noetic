"""
Reference Governor Python Module
"""
import numpy as np
from scipy.integrate import solve_ivp
from path_tools import path_distance
from matplotlib import pyplot as plt
from skimage import measure


def governor_goal(governor_position, safety_level, path):
        # returns the goal position for the current governor configuration

        governor_position = np.array(governor_position)
        safety_level = np.array(safety_level)
        path = np.array(path)
        safety_level[safety_level < 1e-3] = 0

        if np.ndim(safety_level) == 0:
            safety_level = np.full((governor_position.shape[0],), safety_level)


        if path.shape[0] == 0:
                goal = governor_position
        elif path.shape[0] == 1:
                goal = governor_position.copy()
                I = np.sum((governor_position - path) ** 2, axis=1) < safety_level ** 2
                goal[I] = np.tile(path, (np.sum(I), 1))
        else:
                
                Pbase = path[0:-1,:]
                Pdiff = np.diff(path, axis=0)
                Pmag2 = np.sum(Pdiff**2, axis=1)

                goal = governor_position.copy()
                for k in range(governor_position.shape[0]):
                        Gbase = governor_position[k,:] - Pbase
                        Gmag2 = np.sum(Gbase**2, axis=1)

                        GProj = np.sum(Pdiff*Gbase, axis=1)
                        W = GProj/Pmag2
                        W = np.maximum(np.minimum(W, 1), 0)
                        D = (W**2)*Pmag2 - 2*W*GProj + Gmag2

                        I = np.where(D < (safety_level[k]**2))[0]

                        if not(I.size == 0):
                                
                                I = I[-1]
                                w1 = GProj[I]/Pmag2[I]
                                distAB = np.linalg.norm(w1*Pdiff[I,:] - Gbase[I,:])

                                w2 = np.sqrt((safety_level[k]**2 - distAB**2)/Pmag2[I])
                                w = w1 + w2
                                
                                w = np.maximum(np.minimum(w, 1), 0)
                                goal[k,:] = w*Pdiff[I,:] + Pbase[I,:]

        return goal

def governor_vector_field(governor_position, governor_gain, safety_level, path):
        # returns the governor velocity vector only
        
        governor_position = np.array(governor_position)
        path = np.array(path)

        goal = governor_goal(governor_position, safety_level, path)
        
        dgov = governor_gain*(goal - governor_position)

        return dgov

def is_in_domain(ogm, path, position, frame='world'):

        position = np.array(position)
        path = np.array(path)

        if frame == 'grid':
                position = ogm.grid2world(position)

        collDist, coordinates = ogm.distance_to_collision(position)
        pathDist, path_coordinates = path_distance(path, position)

        I = (pathDist <= collDist)

        return I

def plot_domain(ogm, path, **kwargs):
        
        if 'resolution' in kwargs:
                domain_resolution = kwargs['resolution']
        else:
                domain_resolution = 200

        path = np.array(path)

        XLim = ogm.xworldlimits()
        YLim = ogm.yworldlimits()

        X, Y = np.meshgrid(np.linspace(XLim[0], XLim[1], domain_resolution), np.linspace(YLim[0], YLim[1], domain_resolution))
        XY = np.column_stack((X.flatten(), Y.flatten()))
        IN = is_in_domain(ogm, path, XY, frame='world')
        IN = IN.reshape(X.shape)

        contours = measure.find_contours(IN, 0.5)
        I = np.ravel_multi_index((contours[0][:, 0].astype(int), contours[0][:, 1].astype(int)), X.shape)

        resolution = ogm.resolution()
        map = ogm.occupancy_matrix()
        map_size = np.shape(map)[0] * resolution
        plt.figure()
        # plt.imshow(IN, extent=[0, map_size, 0, map_size], cmap='binary', origin="lower")
        plt.imshow(map, extent=[0, map_size, 0, map_size], cmap='binary', origin="lower")
        h = plt.fill(X.flatten()[I], Y.flatten()[I], color = [1 , 0, 0], alpha=0.5)
        plt.plot(path[:,0], path[:,1], 'k', linewidth=2)
        plt.plot(path[0,0], path[0,1], 'ro', markersize=10)
        plt.plot(path[-1,0], path[-1,1], 'go', markersize=10)
        plt.axis('equal')
        plt.show()

def plot_field(ogm, path, **kwargs):

        if 'resolution' in kwargs:
                field_resolution = kwargs['resolution']
        else:
                field_resolution = 10

        path = np.array(path)

        XLim = ogm.xworldlimits()
        YLim = ogm.yworldlimits()


        X, Y = np.meshgrid(np.linspace(XLim[0], XLim[1], field_resolution), np.linspace(YLim[0], YLim[1], field_resolution))
        XY = np.column_stack((X.flatten(), Y.flatten()))
        IN = is_in_domain(ogm, path, XY, frame='world')        
        XY = XY[IN.flatten(),:]
        collision_distances, coordinates = ogm.distance_to_collision(XY)

        UV = governor_vector_field(XY, 1, collision_distances, path)
        UV = UV / np.sqrt(np.sum(UV**2, axis=1)[:, np.newaxis])

        map_resolution = ogm.resolution()
        map = ogm.occupancy_matrix()
        map_size = np.shape(map)[0] * map_resolution
        plt.figure()
        plt.imshow(map, extent=[0, map_size, 0, map_size], cmap='binary', origin="lower")
        # plt.imshow(IN.reshape(X.shape), extent=[0, map_size, 0, map_size], cmap='binary', origin="lower")
        plt.quiver(XY[:,0], XY[:,1], UV[:,0], UV[:,1])
        plt.plot(path[:,0], path[:,1], 'k', linewidth=2)
        plt.plot(path[0,0], path[0,1], 'ro', markersize=10)
        plt.plot(path[-1,0], path[-1,1], 'go', markersize=10)
        plt.axis('equal')
        plt.xlim([0, map_size])
        plt.ylim([0, map_size])
        plt.show()





def robot_governor_vector_field(X, governor_gain, robot_radius, OccGrid2D, path, **kwargs):
        
        robot_position = np.array(X[0:2])
        robot_orientation = X[2]
        governor_position = np.array([X[3:5]])


        path = np.array(path)


        # make all kwarg keys lower case
        kwargs = {k.lower(): v for k, v in kwargs.items()}
        
        # set low level control vector field
        control_fnc = kwargs['control']
        linvel, angvel = control_fnc(robot_position, robot_orientation, governor_position,**kwargs['control_args'])
        drobot = np.array([linvel*np.cos(robot_orientation), linvel*np.sin(robot_orientation), angvel])
        
        # get motion prediction function
        motion_prediction_fnc = kwargs['motion_prediction']
        motion_polygon = motion_prediction_fnc(robot_position, robot_orientation, governor_position,  **kwargs['motion_prediction_args'])

        # get safety level
        frame = 'world'
        distance, XPolygon, XMap = OccGrid2D.distance_to_collision_polygon(motion_polygon, frame)
        safety_level = distance - robot_radius

        # set governor vector field
        dgov = governor_vector_field(governor_position, governor_gain, safety_level, path)

        # concatenate the two vector fields
        dX = np.concatenate((drobot.flatten(), dgov.flatten()), axis=0)

        return dX

def robot_governor_trajectory(robot_position, robot_orientation, governor_position, governor_gain, robot_radius, OccGrid2D, path, duration, **kwargs):
        
        robot_position = np.array(robot_position)
        robot_orientation = np.array(robot_orientation)
        governor_position = np.array(governor_position)

        path = np.array(path)

        robot_position = robot_position.flatten()
        robot_orientation = robot_orientation.flatten()
        governor_position = governor_position.flatten()
        initial_configuration = np.array([robot_position[0], robot_position[1], robot_orientation[0], governor_position[0], governor_position[1]])


        governed_controller_dynamics = lambda t, X: robot_governor_vector_field(X, governor_gain, robot_radius, OccGrid2D, path, **kwargs)
        tspan = [0, duration]

        sol = solve_ivp(governed_controller_dynamics, tspan, initial_configuration, **kwargs)

        return sol.t, sol.y

def robot_governor_discrete_update_ode45(robot_position, robot_orientation, governor_position, governor_gain, robot_radius, OccGrid2D, path, duration, **kwargs):
        
        robot_position = np.array(robot_position)
        governor_position = np.array(governor_position)

        T, X = robot_governor_trajectory(robot_position, robot_orientation, governor_position, governor_gain, robot_radius, OccGrid2D, path, duration, **kwargs)

        new_governor_position = X[3:5,-1]
        # make all kwarg keys lower case
        kwargs = {k.lower(): v for k, v in kwargs.items()}

        # get motion prediction function
        motion_prediction_fnc = kwargs['motion_prediction']
        motion_polygon = motion_prediction_fnc(robot_position, robot_orientation, new_governor_position,  **kwargs['motion_prediction_args'])

        # get safety level
        frame = 'world'
        distance, XPolygon, XMap = OccGrid2D.distance_to_collision_polygon(motion_polygon, frame)
        safety_level = distance - robot_radius

        if safety_level < 0:
                return governor_position

        return X[3:5,-1]