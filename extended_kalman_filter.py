"""

Extended kalman filter (EKF) localization sample

author: Atsushi Sakai (@Atsushi_twi)

"""

import numpy as np
import math
import random
import matplotlib.pyplot as plt
from ransac import ransacSquare

# Estimation parameter of EKF
Q = np.diag([0.1, 0.1, np.deg2rad(1.0), 1.0])**2  # predict state covariance
R = np.diag([1.0, 1.0])**2  # Observation x,y position covariance

#  Simulation parameter
Qsim = np.diag([1.0, np.deg2rad(30.0)])**2
Rsim = np.diag([0.5, 0.5])**2

DT = 0.1  # time tick [s]
SIM_TIME = 5.0  # simulation time [s]

show_animation = True

"""
Our state vector should be defined as follows:
[x-pos, y-pos, orientation, velocity]
we will use ransac for objects
orientation and velocity information in I2C format from IMU in C code
"""

def calc_input(): # this function reads sensor input
    velocity = 2.0  # [m/s]
    angVel = 0.1  # [rad/s]4
    input = np.array([[velocity, angVel]]).T
    return input

def motion_model(state, input):

    F = np.array([[1.0, 0, 0, 0],
                  [0, 1.0, 0, 0],
                  [0, 0, 1.0, 0],
                  [0, 0, 0, 1.0]])

    B = np.array([[DT * math.cos(state[2, 0]), 0],
                  [DT * math.sin(state[2, 0]), 0],
                  [0.0, DT],
                  [1.0, 0.0]])

    state = F@state + B@input # array multiplication

    return state

def observation_model(x):
    #  Observation Model
    H = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    z = H@x

    return z

def observation(state, xd, input):

    state = motion_model(state, input)

    # add noise to gps x-y
    zx = state[0, 0] + np.random.randn() * Rsim[0, 0]
    zy = state[1, 0] + np.random.randn() * Rsim[1, 1]
    z = np.array([[zx, zy]]).T

    # add noise to input
    ud1 = input[0, 0] + np.random.randn() * Qsim[0, 0]
    ud2 = input[1, 0] + np.random.randn() * Qsim[1, 1]
    ud = np.array([[ud1, ud2]]).T

    xd = motion_model(xd, ud)

    return state, z, xd, ud

def jacobF(state, input):
    """
    Jacobian of Motion Model

    motion model
    x_{t+1} = x_t+v*dt*cos(yaw)
    y_{t+1} = y_t+v*dt*sin(yaw)
    yaw_{t+1} = yaw_t+omega*dt
    v_{t+1} = v{t}
    so
    dx/dyaw = -v*dt*sin(yaw)
    dx/dv = dt*cos(yaw)
    dy/dyaw = v*dt*cos(yaw)
    dy/dv = dt*sin(yaw)
    """
    yaw = state[2, 0]
    velocity = input[0, 0]
    jF = np.array([
        [1.0, 0.0, -DT * velocity * math.sin(yaw), DT * math.cos(yaw)],
        [0.0, 1.0, DT * velocity * math.cos(yaw), DT * math.sin(yaw)],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]])

    return jF


def jacobH():
    # Jacobian of Observation Model
    jH = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    return jH


def ekf_estimation(state, covariance, mean, input):

    #  Predict
    state_pred = motion_model(state, input)
    jF = jacobF(state_pred, input)
    covariance_pred = jF@covariance@jF.T + Q

    #  Update
    jH = jacobH()
    meanPred = observation_model(state_pred)
    residual = mean - meanPred
    S = jH@covariance_pred@jH.T + R
    KalmanGain = covariance_pred@jH.T@np.linalg.inv(S)
    state = state_pred + KalmanGain@residual
    covariance = (np.eye(len(state)) - KalmanGain@jH)@covariance_pred

    return state, covariance


def plot_covariance_ellipse(xEst, PEst):  # pragma: no cover
    Pxy = PEst[0:2, 0:2]
    eigval, eigvec = np.linalg.eig(Pxy)

    if eigval[0] >= eigval[1]:
        bigind = 0
        smallind = 1
    else:
        bigind = 1
        smallind = 0

    t = np.arange(0, 2 * math.pi + 0.1, 0.1)
    a = math.sqrt(eigval[bigind])
    b = math.sqrt(eigval[smallind])
    x = [a * math.cos(it) for it in t]
    y = [b * math.sin(it) for it in t]
    angle = math.atan2(eigvec[bigind, 1], eigvec[bigind, 0])
    R = np.array([[math.cos(angle), math.sin(angle)],
                  [-math.sin(angle), math.cos(angle)]])
    fx = R@(np.array([x, y]))
    px = np.array(fx[0, :] + xEst[0, 0]).flatten()
    py = np.array(fx[1, :] + xEst[1, 0]).flatten()
    plt.plot(px, py, "--r")


def main():
    print(__file__ + " start!!")

    time = 0.0
    # State Vector [x y yaw v list]'
    xEst = np.zeros((4, 1)) # state vector
    xTrue = np.zeros((4, 1))
    PEst = np.eye(4)
    xDR = np.zeros((4, 1))  # Dead reckoning

    field_coords = [(n, 50) for n in range(-50, 51)]+[(50, n) for n in range(50, -51, -1)]+[(n, -50) for n in range(50, -51, -1)]+[(-50, n) for n in range(-50, 51)]
    lidar_coords = [(f[0]+random.randint(28, 32), f[1]+random.randint(28, 32)) for f in field_coords]
    field = ransacSquare(lidar_coords, 100, .04, .97, 1000)
    # with open('example.csv') as csvData:
    #     readCSV = csv.reader(csvfile, delimiter=',')
    #     # read input and take data

    # history
    hxEst = xEst
    hxTrue = xTrue
    hxDR = xTrue
    hz = np.zeros((2, 1))

    while SIM_TIME >= time:
        time += DT
        u = calc_input()

        xTrue, z, xDR, ud = observation(xTrue, xDR, u)

        xEst, PEst = ekf_estimation(xEst, PEst, z, ud)

        # store data history
        hxEst = np.hstack((hxEst, xEst))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))
        hz = np.hstack((hz, z))

        if show_animation:
            plt.cla()
            plt.plot(hz[0, :], hz[1, :], ".g")
            plt.plot(field)
            plt.plot(hxTrue[0, :].flatten(),
                     hxTrue[1, :].flatten(), "-b")
            plt.plot(hxDR[0, :].flatten(),
                     hxDR[1, :].flatten(), "-k")
            plt.plot(hxEst[0, :].flatten(),
                     hxEst[1, :].flatten(), "-r")
            plot_covariance_ellipse(xEst, PEst) # red dotted circle
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)


if __name__ == '__main__':
    main()
