import numpy as np
import matplotlib.pyplot as plt
from unicycle_motion_control import (distance_gradient_ctrl, dgc_vector_field, dgc_trajectory, dgc_discrete_update,
                                       dgc_circle_motion_prediction,
                                       dgc_boundedcone_motion_prediction,
                                       dgc_icecreamcone_motion_prediction,
                                       dgc_truncatedicecreamcone_motion_prediction)

###################################################################################################
############################ TEST DISTANCE GRADIENT CONTROL FUNCTIONS ############################
###################################################################################################

def test_dgc_controller(position, orientation, goal, lingain, anggain, tol, motion_direction):
    

    v, w = distance_gradient_ctrl(position, orientation, goal, lingain, anggain, tol, motion_direction)
    
    print(v)
    print(w)

    return None

def test_dgc_vector_field(position, orientation, goal, lingain, anggain, tol, motion_direction):

    dX = dgc_vector_field(position, orientation, goal, lingain, anggain, tol, motion_direction)

    print(dX)

    return None

def test_dgc_trajectory_solver(position, orientation, goal, duration, lingain, anggain, tol, motion_direction):
    
    T, Y = dgc_trajectory(position, orientation, goal, [0, duration], lingain, anggain, tol, motion_direction)

    fig, ax = plt.subplots()
    ax.plot(Y[:,0], Y[:,1], '-')
    ax.set_aspect('equal')
    plt.show()
    
    return None

def test_dgc_discrete_update(position, orientation, goal, duration, lingain, anggain, tol, motion_direction):

    position, orientation = dgc_discrete_update(position, orientation, goal, duration, lingain, anggain, tol, motion_direction)
    
    print(position)
    print(orientation)

    return None

def test_dgc_circle_motion_prediction(position, orientation, goal, res):
    
    polygon = dgc_circle_motion_prediction(position, orientation, goal, res)

    fig, ax = plt.subplots()
    polygon = np.concatenate((polygon,  polygon[:1]), axis=0)
    ax.plot(polygon[:,0], polygon[:,1], '-')
    ax.set_aspect('equal')
    ax.scatter(position[0], position[1], marker='o', color='b')
    ax.scatter(goal[0], goal[1], marker='o', color='r')

    T, Y = dgc_trajectory(position, orientation, goal, [0, 30], lingain, anggain, tol, motion_direction)
    ax.plot(Y[:,0], Y[:,1], '-')

    plt.show()

    return None

def test_dgc_boundedcone_motion_prediction(position, orientation, goal, motion_direction, res):
        
    polygon = dgc_boundedcone_motion_prediction(position, orientation, goal, motion_direction, res)

    fig, ax = plt.subplots()
    polygon = np.concatenate((polygon,  polygon[:1]), axis=0)
    ax.plot(polygon[:,0], polygon[:,1], '-')
    ax.set_aspect('equal')
    ax.scatter(position[0], position[1], marker='o', color='b')
    ax.scatter(goal[0], goal[1], marker='o', color='r')

    T, Y = dgc_trajectory(position, orientation, goal, [0, 30], lingain, anggain, tol, motion_direction)
    ax.plot(Y[:,0], Y[:,1], '-')

    plt.show()

    return None

def test_dgc_icecreamcone_motion_prediction(position, orientation, goal, motion_direction, res):
            
    polygon = dgc_icecreamcone_motion_prediction(position, orientation, goal, motion_direction, res)

    fig, ax = plt.subplots()
    polygon = np.concatenate((polygon,  polygon[:1]), axis=0)
    ax.plot(polygon[:,0], polygon[:,1], '-')
    ax.set_aspect('equal')
    ax.scatter(position[0], position[1], marker='o', color='b')
    ax.scatter(goal[0], goal[1], marker='o', color='r')

    T, Y = dgc_trajectory(position, orientation, goal, [0, 30], lingain, anggain, tol, motion_direction)
    ax.plot(Y[:,0], Y[:,1], '-')

    plt.show()

    return None

def test_dgc_truncatedicecreamcone_motion_prediction(position, orientation, goal, motion_direction, res):
                    
    polygon = dgc_truncatedicecreamcone_motion_prediction(position, orientation, goal, motion_direction, res)

    fig, ax = plt.subplots()
    polygon = np.concatenate((polygon,  polygon[:1]), axis=0)
    ax.plot(polygon[:,0], polygon[:,1], '-')
    ax.set_aspect('equal')
    ax.scatter(position[0], position[1], marker='o', color='b')
    ax.scatter(goal[0], goal[1], marker='o', color='r')

    T, Y = dgc_trajectory(position, orientation, goal, [0, 30], lingain, anggain, tol, motion_direction)
    ax.plot(Y[:,0], Y[:,1], '-')

    plt.show()

    return None


if __name__ == '__main__':
    
    position = [0 , 2]
    orientation = -np.pi/4
    goal = [0, 0]


    epsgain = np.sqrt(2)/2
    refgain = 1.0
    tol = 1e-3
    motion_direction = 'bidirectional'
    duration = 5
    res = 60

    lingain = 2.0
    anggain = 1.0



#######################################################################################
############################ TEST GRADIENT DISTANCE CONTROL ##########################
#######################################################################################
    # test_dgc_controller(position, orientation, goal, lingain, anggain, tol, motion_direction)
    # test_dgc_vector_field(position, orientation, goal, lingain, anggain, tol, motion_direction)
    # test_dgc_trajectory_solver(position, orientation, goal, duration, lingain, anggain, tol, motion_direction)
    # test_dgc_discrete_update(position, orientation, goal, duration, lingain, anggain, tol, motion_direction)
    # test_dgc_circle_motion_prediction(position, orientation, goal, res)
    # test_dgc_boundedcone_motion_prediction(position, orientation, goal, motion_direction, res)
    # test_dgc_icecreamcone_motion_prediction(position, orientation, goal, motion_direction, res)
    # test_dgc_truncatedicecreamcone_motion_prediction(position, orientation, goal, motion_direction, res)