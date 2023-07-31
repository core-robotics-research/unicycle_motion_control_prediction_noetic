import numpy as np
from path_tools import path_distance
from matplotlib import pyplot as plt


def test_path_distance(path, position):
    distance2path, path_coordinates = path_distance(path, position)


    plt.figure()
    plt.plot(path[:,0], path[:,1], 'k-')
    for k in range(position.shape[0]):
        plt.scatter(position[k][0], position[k][1], c='b', s=100)
        plt.scatter(path_coordinates[k][0], path_coordinates[k][1], c='r', s=100)

        # draw a circle with radius distance2path
        distance_circle = position[k,:] + distance2path[k]*np.array([np.cos(np.linspace(0, 2*np.pi, 100)), np.sin(np.linspace(0, 2*np.pi, 100))]).T
        # governor_position[k,:] + safety_level*
        plt.plot(distance_circle[:,0], distance_circle[:,1], 'b--')

    plt.axis('equal')
    plt.show(block=False)
    plt.pause(3)

if __name__ == '__main__':

    path = np.random.rand(3,2)
    position = np.random.rand(1,2)
    
    test_path_distance(path, position)
