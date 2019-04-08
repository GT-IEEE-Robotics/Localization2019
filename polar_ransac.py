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


#takes in 2 cartesian points of the form (x,y) and outputs a list of all 4 verticies
def drawSquare(pt1, pt2):
    """
    https://www.quora.com/Given-two-diagonally-opposite-points-of-a-square-how-can-I-find-out-the-other-two-points-in-terms-of-the-coordinates-of-the-known-points
    """
    A = pt1
    C = pt2
    B = ((A[0] + C[0] + A[1] - C[1]) / 2, (C[0] - A[0] + A[1] + C[1]) / 2)
    D = ((A[0] + C[0] + C[1] - A[1]) / 2, (A[0] - C[0] + A[1] + C[1]) / 2)
    # print([A, B, C, D])
    # plt.plot((A[0], B[0], C[0], D[0], A[0]), (A[1], B[1], C[1], D[1], A[1]), color="blue")
    # plt.show()
    return [A, B, C, D]


def cartesianConvert(polarPt):
    return (polarPt[1] * math.cos(math.radians(polarPt[0])), polarPt[1] * math.sin(math.radians(polarPt[0])))

def getDist(pt1, pt2):
    return math.sqrt(math.pow(pt2[0] - pt1[0], 2) + math.pow(pt2[1] - pt1[1], 2))

def pPtOnLine(cPt1, cPt2, check, error_dist):
    


def ransac(lidar_points=None, color_sensor_pts=None, landmark_pts=None, impact_pts=None, error=0.2, sideLength=SQUARE_SIDE_LENGTH, percentOfPoints=.9, numIterations=1000):
    # landmark_pts = [(x, y, "camera", "pillar" or "internal wall" or "spacetels", color, how sure are that we saw it there)]
    # color_sensors_pts = [("colorsensr", color, weighting)]
    # impac_pts = ["impact_sensors"]

    iteration = 0
    random.seed(15)
    while iteration < numIterations:
        iteration += 1

        pPt1 = lidar_pts[random.randrange(len(lidar_pts))]
        pPt2 = lidar_pts[random.randrange(len(lidar_pts))]

        cPt1 = cartesianConvert(pPt1)
        cPt2 = cartesianConvert(pPt2)
        # print(pPt1)
        # print(pPt2)
        # print(cPt1)
        # print(cPt2)

        #don't bother checking points that are right next to each other and wont form a big enough square
        if sideLength * (1 - error) * math.sqrt(2) < getDist(cPt1, cPt2) < sideLength * (1 + error) * math.sqrt(2):
            verts = drawSquare(cPt1, cPt2)
            # print(verts)

            #finding angle of all the verticies
            thetaA = pPt1[0]
            thetaB = math.degrees(math.atan(verts[1][1] / verts[1][0]))
            thetaC = pPt2[0]
            thetaD = math.degrees(math.atan(verts[3][1] / verts[3][0]))

            side = getDist(cPt1, cPt2)
            error_dist = side * error
            score = 0
            for point in lidar_pts:
                if thetaA <= point[0] <= thetaB:
                    if pPtOnLine(verts[0], verts[1], point, error_dist):
                        score += point[2]
                elif thetaB <= point[0] <= thetaC:
                    if pPtOnLine(verts[1], verts[2], point, error):
                        score += point[2]
                elif thetaC <= point[0] <= thetaD:
                    if pPtOnLine(verts[2], verts[3], point, error):
                        score += point[2]
                else:
                    if pPtOnLine(verts[1], verts[3], point, error):
                        score += point[2]
            return




if __name__ == "__main__":
    lidar_pts = readLidarImage()
    
    fig = plt.figure()
    ax = plt.subplot(111, projection='polar')
    line = ax.scatter([np.radians(point[0]) for point in lidar_pts], [point[1] for point in lidar_pts], s=5,
                           cmap=plt.cm.Greys_r, lw=0)
    ax.grid(True)

    plt.show()

    ransac(lidar_pts)