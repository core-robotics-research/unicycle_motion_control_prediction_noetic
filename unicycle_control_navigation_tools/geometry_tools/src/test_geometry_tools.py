import numpy as np
import matplotlib.pyplot as plt

from geometry_tools import circle_polygon, convexhull_polygon


def test_convexhull_polygon(input_points):
    
    polygon = convexhull_polygon(input_points)


    for p in polygon:
        # pass
        print(p[0], p[1])

    fig, ax = plt.subplots()
    # add the first point to the end of the polygon
    polygon = np.concatenate((polygon,  polygon[0:1]), axis=0)
    ax.plot(polygon[:,0], polygon[:,1], '-')
    ax.set_aspect('equal')
    # plt.show()
    



    return None


def test_circle_polygon(radius, resolution):
    
    polygon = circle_polygon(radius, resolution)
    

    fig, ax = plt.subplots()
    polygon = np.concatenate((polygon,  polygon[:2]), axis=0)
    ax.plot(polygon[:,0], polygon[:,1], '-')
    ax.set_aspect('equal')
    plt.show()

    return None



if __name__ == '__main__':
    
    radius = 2.0
    resolution = 60
    input_points = [[1, 1], [1, 1], [1, 1]]

    # test_circle_polygon(radius, resolution)
    # test_convexhull_polygon(input_points)