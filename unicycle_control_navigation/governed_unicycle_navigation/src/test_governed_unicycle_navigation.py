import numpy as np
from governed_unicycle_navigation import is_in_domain, plot_domain, plot_field, governor_goal, robot_governor_trajectory, robot_governor_discrete_update_ode45, governor_goal
from unicycle_motion_control import distance_gradient_ctrl, dgc_circle_motion_prediction, dgc_boundedcone_motion_prediction, dgc_icecreamcone_motion_prediction, dgc_truncatedicecreamcone_motion_prediction
from map_tools import OccupancyGridMap2D
from matplotlib import pyplot as plt
import matplotlib.patches as patches

def test_plot_domain(map, path, resolution=200):

    # position = np.array([[0.0, 0.0]])
    # I = is_in_domain(map, path, position, frame='world')
    # print("I", I)

    plot_domain(map, path, resolution=resolution)

def test_plot_field(map, path, resolution=50):

    plot_field(map, path, resolution=resolution)

def test_governor_goal(governor_position, safety_level, path):
    
    goal = governor_goal(governor_position, safety_level, path)
    

    if path.shape[0] > 0:
        plt.plot(path[:,0], path[:,1], 'k')
        plt.plot(path[0,0], path[0,1], 'ro')
        plt.plot(path[-1,0], path[-1,1], 'go')
    plt.plot(governor_position[:,0], governor_position[:,1], 'r.')
    plt.plot(goal[:,0], goal[:,1], 'bo')

    for k in range(governor_position.shape[0]):
        safe_circle = governor_position[k,:] + safety_level*np.array([np.cos(np.linspace(0, 2*np.pi, 100)), np.sin(np.linspace(0, 2*np.pi, 100))]).T
        plt.plot(safe_circle[:,0], safe_circle[:,1], 'r--')

    print("goal", goal)

    plt.axis('equal')
    plt.show(block=False)
    plt.pause(3)

def test_robot_governor_trajectory(robot_position, robot_orientation, governor_position, governor_gain, robot_radius, ogm, path, duration, **kwargs):

    T, X = robot_governor_trajectory(robot_position, robot_orientation, governor_position, governor_gain, robot_radius, ogm, path, duration, **kwargs)

    map = ogm.occupancy_matrix()
    resolution = ogm.resolution()
    map_size = np.shape(map)[0] * resolution 
    

    # Create Figure for Visualization
    plt.ion()
    fig = plt.figure()
    # fig.set(**self.figure_options)
    ax = fig.add_subplot(1,1,1)
    # ax.set(**self.axes_options)
    # ax.grid(**self.grid_options)

    plt.imshow(map, extent=[0, map_size, 0, map_size], cmap='binary', origin="lower")
    plt.plot(path[:,0], path[:,1], 'k', linewidth=2)
    plt.plot(path[0,0], path[0,1], 'ro', markersize=10)
    plt.plot(path[-1,0], path[-1,1], 'go', markersize=10)
    
    plt.plot(X[0,:], X[1,:], color=[0, 0, 0.75], linewidth=2)
    plt.plot(X[3,:], X[4,:], color=[0, 0.5, 0], linewidth=2)
    plt.axis('equal')

    # draw motion prediction
    control_fnc = kwargs['control']
    motion_prediction_fnc = kwargs['motion_prediction']
    motion_polygon = motion_prediction_fnc(X[0:2,0], X[2, 0], X[3:5,0],  **kwargs['motion_prediction_args'])
    motion_polygon_plot = patches.Polygon(motion_polygon, facecolor=[0.75, 0.75, 0.0])
    ax.add_patch(motion_polygon_plot)

    # draw governor
    governor_body = patches.Circle((X[3, 0], X[4, 0]), radius=robot_radius, facecolor=[0, 0.6, 0])
    ax.add_patch(governor_body)


    # draw robot
    R = robot_radius
    thwl = np.linspace(5 * np.pi / 6, np.pi / 6, 60)  # Angle samples for the visualization of the left robot wheel
    thwr = np.linspace(7 * np.pi / 6, 11 * np.pi / 6, 60)  # Angle sample for the visualization of the right robot wheel

    robot_body_plot = patches.Circle((X[0, 0], X[1, 0]), radius=R, facecolor=[0, 0.75, 0.75])
    robot_left_wheel_plot = patches.Polygon(np.array([X[0, 0] + R * np.cos(thwl + X[2, 0]), X[1, 0] + R * np.sin(thwl + X[2, 0])]).T, facecolor='k')
    robot_right_wheel_plot = patches.Polygon(np.array([X[0, 0] + R * np.cos(thwr + X[2, 0]), X[1, 0] + R * np.sin(thwr + X[2, 0])]).T, facecolor='k')
    ax.add_patch(robot_body_plot)
    ax.add_patch(robot_left_wheel_plot)
    ax.add_patch(robot_right_wheel_plot)    

    thick = 0.05
    thdir = np.concatenate((np.array([np.pi / 2, -np.pi / 2]), np.linspace(-np.arcsin(thick / R), np.arcsin(thick / R), 60)))
    mdir = np.concatenate((np.array([thick, thick]), np.tile(R, 60)))
    robot_direction_plot = patches.Polygon(np.array([X[0, 0] + mdir * np.cos(thdir + X[2, 0]), X[1, 0] + mdir * np.sin(thdir + X[2, 0])]).T, facecolor='k')
    ax.add_patch(robot_direction_plot)


    # governor_position = np.array([X[3:5,0]])
    # frame = 'world'
    # distance, XPolygon, XMap = ogm.distance_to_collision_polygon(motion_polygon, frame)
    # safety_level = distance - robot_radius
    # goal = governor_goal(governor_position, safety_level, path)
    # goal_plot = patches.Circle((goal[0,0], goal[0,1]), radius=0.1, facecolor=[1, 0.0, 0])
    # ax.add_patch(goal_plot)

    # animate the robot
    for k in range(T.shape[0]):

        # update robot and governor body positions
        governor_body.center = (X[3, k], X[4, k])
        robot_body_plot.center = (X[0, k], X[1, k])

        # update robot wheel positions
        robot_left_wheel_plot.set_xy(np.array([X[0, k] + R * np.cos(thwl + X[2, k]), X[1, k] + R * np.sin(thwl + X[2, k])]).T)
        robot_right_wheel_plot.set_xy(np.array([X[0, k] + R * np.cos(thwr + X[2, k]), X[1, k] + R * np.sin(thwr + X[2, k])]).T)

        # update robot direction
        robot_direction_plot.set_xy(np.array([X[0, k] + mdir * np.cos(thdir + X[2, k]), X[1, k] + mdir * np.sin(thdir + X[2, k])]).T)

        # update motion prediction
        motion_polygon = motion_prediction_fnc(X[0:2,k], X[2, k], X[3:5,k],  **kwargs['motion_prediction_args'])
        motion_polygon_plot.set_xy(motion_polygon)

        frame = 'world'
        distance, XPolygon, XMap = ogm.distance_to_collision_polygon(motion_polygon, frame)
        safety_level = distance - robot_radius
        
        # governor_position = np.array([X[3:5,k]])
        # goal = governor_goal(governor_position, safety_level, path)
        # goal_plot.center = (goal[0,0], goal[0,1])

        plt.draw()
        plt.pause(0.05)
        
    plt.pause(2)

    return T, X

def test_robot_governor_discrete_update_ode45(robot_position, robot_orientation, governor_position, governor_gain, robot_radius, ogm, path, duration, **kwargs):

    govX = robot_governor_discrete_update_ode45(robot_position, robot_orientation, governor_position, governor_gain, robot_radius, ogm, path, duration, **kwargs)

    print("govX", govX)
    return govX



if __name__ == '__main__':

    # fix the seed for reproducibility
    np.random.seed(101)

    governor_gain = 4.0
    duration = 7.0

    robot_position = np.array([[2.0, 2.0]])
    robot_orientation = np.pi/2
    robot_radius = 0.25

    governor_position = np.array([[2.0, 2.0]])



    ################################################################################################
    #################################### SET UP THE PATH ##########################################
    ################################################################################################

    # empty path
    path = np.array([])
    # point path
    path = np.array([[0, 0]])
    # z-shaped path
    path = np.array([[0.0, 1.0], [1.0, 1.0], [0.0, 0.0], [1.0, 0.0]])
    # stair path
    path = np.array([[2.1, 2.1], [2.1, 3.1], [3.1, 3.1], [3.1, 4.1], [4.1, 4.1], [4.1, 5.1], [5.1, 5.1]])



    ################################################################################################
    ################################### SET UP THE MAP #############################################
    ################################################################################################

    width=8
    height=8
    resolution=0.05
    origin=[0.0, 0.0]
    frame = 'world'
    ogm = OccupancyGridMap2D(width, height, resolution, origin)
    map = ogm.occupancy_matrix()

    # create n number of obstacles positioned on a line with angle theta and length L starting at position p
    n = 100
    theta = np.pi/4
    L = 5.0
    p = np.array([4.0, 2.0])
    obstacle_list = []
    for k in range(n):
        w = k/n
        obstacle_list.append(p + L*w*np.array([np.cos(theta), np.sin(theta)]))
    # create n number of obstacles positioned on a line with angle theta and length L starting at position p
    n = 20
    theta = 0.0
    L = 5.0
    p = np.array([0.1, 0.1])
    for k in range(n):
        w = k/n
        obstacle_list.append(p + L*w*np.array([np.cos(theta), np.sin(theta)]))
    # create n number of obstacles positioned on a line with angle theta and length L starting at position p
    n = 20
    theta = np.pi/2
    L = 8.0
    p = np.array([0.1, 0.1])
    for k in range(n):
        w = k/n
        obstacle_list.append(p + L*w*np.array([np.cos(theta), np.sin(theta)]))
    # create n number of obstacles positioned on a line with angle theta and length L starting at position p
    n = 20
    theta = 0
    L = 8.0
    p = np.array([0.1, 7.1])
    for k in range(n):
        w = k/n
        obstacle_list.append(p + L*w*np.array([np.cos(theta), np.sin(theta)]))
    # create n number of obstacles positioned on a line with angle theta and length L starting at position p
    n = 20
    theta = np.pi/2
    L = 8.0
    p = np.array([6.1, 0.1])
    for k in range(n):
        w = k/n
        obstacle_list.append(p + L*w*np.array([np.cos(theta), np.sin(theta)]))

    ogm.set_occupancy(obstacle_list, 1.0)



    ################################################################################################
    #################################### TEST GOVERNOR GOAL ########################################
    ################################################################################################
    
    # test_plot_domain(ogm, path, resolution=200)

    # safety_level = 0.25
    # governor_position = np.random.rand(4,2)
    # test_governor_goal(governor_position, safety_level, path)

    # test_plot_field(ogm, path, resolution=50)



    ################################################################################################
    ########################### TEST GOVERNOR TRAJECTORY and DISCRETE UPDATE #######################
    ################################################################################################

    control_fnc = distance_gradient_ctrl 
    control_args = {'lingain': 1.0, 'anggain': 1.0, 'tol': 1e-3, 'motion_direction': 'Bidirectional'}
    
    # dgc_circle_motion_prediction, dgc_boundedcone_motion_prediction, dgc_icecreamcone_motion_prediction, dgc_truncatedicecreamcone_motion_prediction
    motion_prediction_fnc = dgc_truncatedicecreamcone_motion_prediction
    motion_prediction_args = {'res': 60, 'motion_direction': 'Bidirectional'}


    T, X = test_robot_governor_trajectory(robot_position, robot_orientation, governor_position, governor_gain, robot_radius, ogm, path, duration, control = control_fnc, control_args = control_args, motion_prediction = motion_prediction_fnc, motion_prediction_args = motion_prediction_args, max_step = 5e-2)  
    # govX = test_robot_governor_discrete_update_ode45(robot_position, robot_orientation, governor_position, governor_gain, robot_radius, ogm, path, duration, control = control_fnc, control_args = control_args, motion_prediction = motion_prediction_fnc, motion_prediction_args = motion_prediction_args, max_step = 5e-2)  