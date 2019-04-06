import math
import random
import matplotlib.pyplot as plt
import numpy as np

"""
lidar_pts = [(theta, distance, weighting)]
color_sensor_pts = []
"""

SQUARE_SIDE_LENGTH = 100
THRESHOLD = 0.07
ACCEPTABLE_PERCENTAGE = 0.9
ITERATIONS = 100000

def readLidarImage():
    with open("lidar_dataset/image1.txt") as f:
        content = f.readlines();

    lidar_points = []
    for line in content:
        line_data = line.split()
        quality = int(line_data[1])
        theta = float(line_data[2])
        distance = float(line_data[3])

        if (quality > 0):
            lidar_points.append((theta, distance, quality))

    return lidar_points


def ransac():
    pass

if __name__ == "__main__":
    lidar_pts = readLidarImage()
    
    fig = plt.figure()
    ax = plt.subplot(111, projection='polar')
    line = ax.scatter([np.radians(point[0]) for point in lidar_pts], [point[1] for point in lidar_pts], s=5,
                           cmap=plt.cm.Greys_r, lw=0)
    ax.grid(True)

    plt.show()

    ransac(lidar_points=lidar_pts, color_sensor_pts=None, landmark_pts=None, impact_pts=None)
    # landmark_pts = [(x, y, "camera", "pillar" or "internal wall" or "spacetels", color, how sure are that we saw it there)]
    # color_sensors_pts = [("colorsensr", color, weighting)]
    # impac_pts = ["impact_sensors"]