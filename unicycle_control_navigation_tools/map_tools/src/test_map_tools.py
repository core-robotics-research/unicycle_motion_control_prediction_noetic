import numpy as np
import matplotlib.pyplot as plt
from map_tools import OccupancyGridMap2D

def test_occgrid_map(width, height, resolution, origin):
    ogm = OccupancyGridMap2D(width, height, resolution, origin)

    # print("ogm.occgrid_map: ", ogm.occupancy_matrix())

    return ogm
    

def test_distance_field(width, height, resolution, origin, obstacle_list):
    
    ogm = OccupancyGridMap2D(width, height, resolution, origin)
    ogm.set_occupancy(obstacle_list, 1.0)

    D, Idx = ogm.distance_field()

    # print(D)

    plt.figure()
    plt.imshow(D, cmap='gray', origin='lower')
    plt.show()


def test_distance_to_collision(width, height, resolution, origin, obstacle_list, position, frame='world'):
    
    ogm = OccupancyGridMap2D(width, height, resolution, origin)
    ogm.set_occupancy(obstacle_list, 1.0)
    map = ogm.occupancy_matrix()

    # test distance_to_collision
    distance, coordinates = ogm.distance_to_collision(position, frame)
    print("distance: ", distance)
    print("coordinates: ", coordinates)
    if frame == 'grid':
        coordinates = ogm.grid2world(coordinates)
        position = ogm.grid2world(position)

    plt.figure()
    map_size = np.shape(map)[0] * resolution
    plt.imshow(map, extent=[0, map_size, 0, map_size], cmap='binary', origin="lower")
    plt.scatter(position[0][0], position[0][1], c='b', s=100)
    plt.scatter(coordinates[0][0], coordinates[0][1], c='r', s=100)


    plt.show(block=False)
    plt.pause(3)



def test_distance_to_collision_polygon(width, height, resolution, origin, obstacle_list, polygon, frame='world'):
        
        ogm = OccupancyGridMap2D(width, height, resolution, origin)
        ogm.set_occupancy(obstacle_list, 1.0)
        map = ogm.occupancy_matrix()
    
        # test distance_to_collision
        distance, XPolygon, XMap = ogm.distance_to_collision_polygon(polygon, frame)
        print("distance: ", distance)
        # print("XPolygon: ", XPolygon)
        # print("XMap: ", XMap)

        if frame == 'grid':
            XPolygon = ogm.grid2world(XPolygon)
            polygon = ogm.grid2world(polygon)
    
        polygon = np.reshape(np.array(polygon), (-1, 2))

        plt.figure()
        map_size = np.shape(map)[0] * resolution
        plt.imshow(map, extent=[0, map_size, 0, map_size], cmap='binary', origin="lower")
        
        # add last point to polygon
        polygon = np.vstack((polygon, polygon[0]))
        x = polygon[:, 0]
        y = polygon[:, 1]
        plt.plot(x, y, c='b', linewidth=5)  
        plt.scatter(XPolygon[0][0], XPolygon[0][1], c='r', s=100)
        plt.scatter(XMap[0][0], XMap[0][1], c='g', s=100)
    
        plt.show(block=False)
        plt.pause(10)



if __name__ == '__main__':
    

    width=12
    height=12
    resolution=0.05
    origin=[0.0, 0.0]
    obstacle_list = [[3.0, 0.0], [5, 5]]
    # add many random obstacles scattered in the map
    obstacle_list = np.random.rand(100, 2) * 10

    # obstacle_list = [[5.0, 5.0]]

    position = [[3, 8], [2, 4], [4, -5], [4, 17]]
    frame = 'world'

    # ogm_map = test_occgrid_map(width, height, resolution, origin)
    # test_distance_field(width, height, resolution, origin, obstacle_list)

    # fix the seed
    np.random.seed(0)

    polygon = [[5, 1], [8, 4.5], [8, 1]]
    obstacle_list = np.random.rand(3, 2) * 10
    test_distance_to_collision_polygon(width, height, resolution, origin, obstacle_list, polygon, frame)

    position = [[1, 1]]
    # test_distance_to_collision(width, height, resolution, origin, obstacle_list, position, frame)




    # ogm = OccupancyGridMap2D(width, height, resolution, origin)
    # ogm.set_occupancy(obstacle_list, 1.0)
    # # print(ogm.occupancy_matrix())

    # ogm_map2 = OccupancyGridMap2D(width, height, resolution, origin)
    # # ogm_map2.set_occupancy()

    # frame = "grid"
    # xy = np.array(np.zeros(ogm_map2.shape()))


    # plt.figure("ogm_map2_b4")
    # plt.imshow(ogm_map2.occupancy_matrix(), cmap='gray', origin='lower')
    # plt.show(block=False)
    # plt.pause(1)

    # # print(ogm_map2.occupancy_matrix())
    # map2_matrix = ogm_map2.occupancy_matrix()
    # occval = map2_matrix.reshape(-1,1)

    # new_map = ogm.occupancy_matrix()
    # occval = new_map.flatten()
    # xy = np.indices(new_map.shape).reshape(2, -1).T

    # print(new_map)
    # print("xy: ", xy)
    # print("occval: ", occval)

    
    # ogm_map2.set_occupancy(xy, occval, frame='grid')

    # plt.figure("ogm_map2")
    # plt.imshow(ogm_map2.occupancy_matrix(), cmap='gray', origin='lower')
    # plt.show(block=False)

    # plt.figure("new map")
    # plt.imshow(new_map, cmap='gray', origin='lower')
    # plt.show(block=False)

    # plt.figure("ogm_map")
    # plt.imshow(ogm.occupancy_matrix(), cmap='gray', origin='lower')
    # plt.show(block=False)

    # plt.pause(10)

    # print(ogm.occupancy_matrix())

    # test = np.int8(ogm.occupancy_matrix()*100).flatten().tolist()
    # print(test)

    # test = np.array(test)
    # # divide each element of test by 100
    # test2 = test / 100
    # print(test2)

    # ogm_map2.set_occupancy(xy, test2, frame='grid')
    # print(ogm_map2.occupancy_matrix())