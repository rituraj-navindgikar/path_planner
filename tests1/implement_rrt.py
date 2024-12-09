#!/usr/bin/env python3

# to implement RRT we need, step size, starting coordinates, goal coordinates and map
# in return we get number of waypoints, the waypoints and path distance

import imageio.v2 as imageio
import numpy as np
import matplotlib.pyplot as plt
from algorithms.rrt import RRTAlgorithm

def pgm_to_npy(map):
    image = imageio.imread(map)

    # Invert black and white: max_value - image
    max_value = np.max(image)
    inverted_image = max_value - image

    # Convert inverted image to NumPy array
    image_array = np.array(inverted_image, dtype=np.uint8)

    # Save NumPy array as .npy file
    npy_path = 'image.npy'
    np.save(npy_path, image_array)

pgm_map = 'image.pgm'
pgm_to_npy(pgm_map)

grid = np.load('image.npy')
start = np.array([132.0, 132.0])
goal = np.array([400.0, 300])

numIterations = 2000
stepSize = 5

goalRegion = plt.Circle((
    goal[0], goal[1]), 
    stepSize,
    color='b',
    fill= False
    )

rrt = RRTAlgorithm(start, goal, 1000, grid, stepSize)

for i in range(rrt.iterations):
    #reset nearest values
    rrt.resetNearestValues()
    print("Iteration", i)

    #algorithm
    point = rrt.sampleAPoint()
    rrt.findNearest(rrt.randomTree, point)
    new = rrt.steerToPoint(rrt.nearestNode, point)
    bool = rrt.isInObstacle(rrt.nearestNode, new)
    if bool == False:
        rrt.addChild(new[0], new[1])
        plt.pause(0.10)
        plt.plot([rrt.nearestNode.locationX, new[0]], [rrt.nearestNode.locationY, new[1]], 'go', linestyle="-")

        # if goal found, append to path
        if rrt.goalFound(new):
            rrt.addChild(goal[0], goal[1])
            print("Goal Found!")
            break

rrt.retraceRRTPath(rrt.goal)
rrt.wayPoints.insert(0,start)
print("Number of way points: ", rrt.numWayPoints)
print("Path Distance (m): ", rrt.path_distance)
print("Waypoints: ", rrt.wayPoints)




