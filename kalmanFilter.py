from matplotlib import pyplot as mp
import numpy as np

def gaussian(reading, mean, sd):
    return np.exp(-np.power(reading - mean, 2.) / (2 * np.power(sd, 2.)))

movement_variance = 1
lidar_variance = 1
imu_variance = 1


posX = 0
posY = 0
posTheta = 0
velocity = 1
dt = 0.1


# general idea is to generate a gaussian for each sensor and update/predict for
# each gausssian available
