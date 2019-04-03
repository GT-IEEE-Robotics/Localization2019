#input polarCord as a list of tuples with form (radius, theta)
#outputs a list of tuples with the form of (x, y)
import math
import random
import matplotlib.pyplot as plt

SIDE_LENGTH = 3000
ERROR = 0.025

def cartesianConvert(polarCord):
    out = []
    for point in polarCord:
        out.append((point[0] * math.cos(point[1]), point[0] * math.sin(point[1])))
    return out

def getDist(pt1, pt2):
    out = math.sqrt(math.pow(pt2[0] - pt1[0], 2) + math.pow(pt2[1] - pt1[1], 2))
    # print(out)
    return out

#take in 2 points as tuples (x,y)
#outputs a 2 tuples for lines that have the form y = mx + b [(m1, b1), (m2, b2)]
#these are the two lines that go from point 1 to form half a square 
# def drawSquare(pt1, pt2):
#     if pt1[0] > pt2[0]:     #draw the same square regardless of order of points
#         tmp = pt1
#         pt1 = pt2
#         pt2 = tmp
#     if pt2[0] - pt1[0] == 0:  #need to deal with vertical lines
#         m1 = (pt2[1] - pt1[1]) / 0.00000000001
#     else:
#         m1 = (pt2[1] - pt1[1]) / (pt2[0] - pt1[0])
#     if m1 == 0:
#         m2 = -1 / 0.00000000001
#     else:
#         m2 = -1 / m1
#     b1 = pt1[1] - m1 * pt1[0]
#     b2 = pt1[1] - m2 * pt1[0]
#     b3 = pt2[1] - m1 * pt2[0] 
#     print([(m1, b1), (m2, b2)])
#     return [(m1, b1), (m2, b2)]

def drawSquare(pt1, pt2):       #https://www.quora.com/Given-two-diagonally-opposite-points-of-a-square-how-can-I-find-out-the-other-two-points-in-terms-of-the-coordinates-of-the-known-points
    A = pt1
    C = pt2
    B = ((A[0] + C[0] + A[1] - C[1]) / 2, (C[0] - A[0] + A[1] + C[1]) / 2)
    D = ((A[0] + C[0] + C[1] - A[1]) / 2, (A[0] - C[0] + A[1] + C[1]) / 2)
    # print([A, B, C, D])
    # plt.plot((A[0], B[0], C[0], D[0], A[0]), (A[1], B[1], C[1], D[1], A[1]), color="blue")
    # plt.show()
    return [A, B, C, D]


def ptOnLine(pt1, pt2, ptCheck, error):      #returns 1 if on line, otherwise 0
    out = 0
    side = getDist(pt1, pt2)
    if pt1[0] > pt2[0]:       #making pt1 always to the left of pt2
        tmp = pt1
        pt1 = pt2
        pt2 = tmp
    if pt1[0] - (pt2[0] - pt1[0]) * error <= ptCheck[0] <= pt2[0] + (pt2[0] - pt1[0]) * error:    #check if x val is within range
        if pt2[0] - pt1[0] != 0:
            m = (pt2[1] - pt1[1]) / (pt2[0] - pt1[0])
            b = pt1[1] - m * pt1[0]
            yOnLine = m * ptCheck[0] + b
            if m != 0:
                error_dist = side * error
                denom = math.cos(math.atan(1 / m))
                if denom == 0:
                    out = 1
                else:
                    if yOnLine - error_dist / denom <= ptCheck[1] <= yOnLine + error_dist / denom:
                        out = 1
            else:
                if yOnLine - side * error <= ptCheck[1] <= yOnLine + side * error:
                    out = 1
        else:
            out = 1
    # print(out)
    return out




#cartesianCord is a list of tuples of (x, y)
#sideLength is the known length of the square be localized
#error is the acceptable percent error of the drawn square to find if points lie in
#percentOfPoints is the percentage of points that need to be included in the square to return a successful value
#outputs a tuple where the first value is 0 if failed or 1 if success
def ransacSquare(cartesianCord, sideLength, error, percentOfPoints, numIterations):
    # for i in range(len(cartesianCord)):
    #     x = cartesianCord[i][0]
    #     y = cartesianCord[i][1]
    #     plt.plot(x,y, marker='o', markersize=3, color="red")

    iteration = 0
    success = 0
    while not success and iteration < numIterations:
        pt1 = cartesianCord[random.randrange(len(cartesianCord))]
        # print(pt1)
        pt2 = cartesianCord[random.randrange(len(cartesianCord))]
        # print(pt2)
        if sideLength * (1 - error) * math.sqrt(2) < getDist(pt1, pt2) < sideLength * (1 + error) * math.sqrt(2):   #don't bother checking points that are right next to each other
            verts = drawSquare(pt1, pt2)
            for i in range(len(cartesianCord)):
                x = cartesianCord[i][0]
                y = cartesianCord[i][1]
                plt.plot(x,y, marker='o', markersize=3, color="red")
            plt.plot((verts[0][0], verts[1][0], verts[2][0], verts[3][0], verts[0][0]), (verts[0][1], verts[1][1], verts[2][1], verts[3][1], verts[0][1]), color="blue")
            plt.show()
            #now i need to count the number of points that are on the new square
            count = 0
            for point in cartesianCord:
                if ptOnLine(verts[0], verts[1], point, error) or ptOnLine(verts[1], verts[2], point, error) or ptOnLine(verts[2], verts[3], point, error) or ptOnLine(verts[0], verts[3], point, error):
                    count = count + 1
            if count >= percentOfPoints * len(cartesianCord):
                return verts
        iteration = iteration + 1
    return 0

def main():
    field_coords = [(n, 50) for n in range(-50, 51)]+[(50, n) for n in range(50, -51, -1)]+[(n, -50) for n in range(50, -51, -1)]+[(-50, n) for n in range(-50, 51)]
    lidar_coords = [(f[0]+random.randint(28, 32), f[1]+random.randint(28, 32)) for f in field_coords]
    # plot()
    # cart = []
    # for i in range(360):
    #     x = random.randrange(100)
    #     y = random.randrange(100)
    #     cart.append((x,y))
    #     plt.plot(x,y, marker='o', markersize=3, color="red")
    # plt.show()

    print(ransacSquare(lidar_coords, 100, .04, .97, 1000))
        



# getDist((1,0), (1,1))
# drawSquare((0,0), (1,2))
# ptOnLine((0,0), (1,0), (.5,-.025), .05)
main()

