import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from math import tan, sin, cos, sqrt, atan2
from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.stats import plot_covariance


def readData(FileName):
	data = pd.read_csv('%s.csv'%FileName)
	
	return data

def normalize_angle(x):
    x = x % (2 * np.pi)    # force in range [0, 2 pi)
    if x > np.pi:          # move to [-pi, pi)
        x -= 2 * np.pi
    return x


def residual_x(a, b):
    y = a - b
    y[2] = normalize_angle(y[2])
    return y

def residual_h(a, b):
    y = a - b
    y[2] = normalize_angle(y[2])
    return y


def state_trans(x, dt, u):
    if(abs(u[1]) > 0.000001):
        x[2] = normalize_angle(x[2] + u[1])
    else:
        x[0] -= sin(x[2]) * u[0]
        x[1] += cos(x[2]) * u[0];

    return x;

def measurement(x):
    return x


def state_mean(sigmas, Wm):
    x = np.zeros(3)

    sum_sin = np.sum(np.dot(np.sin(sigmas[:, 2]), Wm))
    sum_cos = np.sum(np.dot(np.cos(sigmas[:, 2]), Wm))
    x[0] = np.sum(np.dot(sigmas[:, 0], Wm))
    x[1] = np.sum(np.dot(sigmas[:, 1], Wm))
    x[2] = atan2(sum_sin, sum_cos)
    return x

def z_mean(sigmas, Wm):
    x = np.zeros(3)

    sum_sin = np.sum(np.dot(np.sin(sigmas[:, 2]), Wm))
    sum_cos = np.sum(np.dot(np.cos(sigmas[:, 2]), Wm))
    x[0] = np.sum(np.dot(sigmas[:, 0], Wm))
    x[1] = np.sum(np.dot(sigmas[:, 1], Wm))
    x[2] = atan2(sum_sin, sum_cos)
    return x


def run_sim(position, controlIn, truth):
    points = MerweScaledSigmaPoints(n=3, alpha=.00001, beta=2, kappa=0, subtract=residual_x)
 
    dt = 0.1
    ukf = UKF(dim_x=3, dim_z=3, fx=state_trans, hx=measurement,
              dt=dt, points=points, x_mean_fn=state_mean, 
              z_mean_fn=z_mean, residual_x=residual_x, 
              residual_z=residual_h)

    ukf.x = np.array([0, 0, .0])
    ukf.P = np.diag([.1, .1, .05])
    ukf.R = np.diag([0.1**2, 0.1**2, np.radians(1)])
    ukf.Q = np.eye(3)*0.0001


    for x in range(np.size(position, 0)):
        
        u = controlIn[x]
        z = position[x]
        ukf.predict(u=u)
        ukf.update(z)

        if x % 10 == 0:
            plt.plot(ukf.x[0], ukf.x[1], 'ro', alpha=0.3)
            plt.plot(position[x][0], position[x][1], 'go', alpha=0.3)
            plt.plot(truth[x][0], truth[x][1], 'ko', alpha=0.3)

        #plot_covariance((ukf.x[0], ukf.x[1]), ukf.P[0:2, 0:2], std=.1, facecolor='g', alpha=0.3)

    #axes = plt.gca()
    #axes.set_xlim([-.5,.5])
    #axes.set_ylim([-.5,.5])
    plt.show()




truth = readData("truth")
measurements = readData("position")

position = measurements.filter(regex='^[^u]')
controlIn = measurements.filter(regex='^u')

run_sim(position.to_numpy(), controlIn.to_numpy(), truth.to_numpy())


#plt.plot(truth.x, truth.y, 'bo')
#plt.plot(measurements.x, measurements.y, 'ro')
##plt.plot([truth.x[0], truth.x[2]], [truth.y[0], truth.y[2]], color='k', linestyle='-', linewidth=2)
#plt.show()
