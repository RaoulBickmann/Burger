import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from math import sin, cos, sqrt, atan2
from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.stats import plot_covariance


TIRE_RAD = 0.02
AXLE_LEN = 0.065 * 2


def pointOnLine(l1, l2, p):
    return false

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

#def residual_h(a, b):
#    y = a - b
#    y[2] = normalize_angle(y[2])
#    return y


#prediction aus control input und vorigem zustand
def state_trans(x, dt, u):

    x[0] -= sin(x[2]) * u[0] * dt
    x[1] += cos(x[2]) * u[0] * dt

    x[2] = normalize_angle(x[2] + u[1] * dt)

    return x;

def measurement_trans(x, measurement):
    #dist = min(z[0], z[1])
    #angle = (z[0] - z[1]) /(AXLE_LEN)

    #x[0] -= sin(x[2]) * dist
    #x[1] += cos(x[2]) * dist

    #x[2] = normalize_angle(x[2] + angle)
    
    y = np.zeros(3)
    
    y[0] = measurement[0]
    y[1] = measurement[1]
    y[2] = 0

    angle = normalize_angle(atan2(x[4] - x[1], x[3] - x[0]))
    if(abs(angle < 5)):
        y[2] = 1 - sqrt((x[3] - x[0])**2 + (x[4] - x[1])**2)
    if(y[2] < 0):
        y[2] = 0

    return y


def state_mean(sigmas, Wm):
    x = np.zeros(5)


    x[0] = np.sum(np.dot(sigmas[:, 0], Wm))
    x[1] = np.sum(np.dot(sigmas[:, 1], Wm))

    sum_sin = np.sum(np.dot(np.sin(sigmas[:, 2]), Wm))
    sum_cos = np.sum(np.dot(np.cos(sigmas[:, 2]), Wm))
    x[2] = atan2(sum_sin, sum_cos)

    x[3] = np.sum(np.dot(sigmas[:, 3], Wm))
    x[4] = np.sum(np.dot(sigmas[:, 4], Wm))
    return x

#def z_mean(sigmas, Wm):
#    x = np.zeros(3)

#    sum_sin = np.sum(np.dot(np.sin(sigmas[:, 2]), Wm))
#    sum_cos = np.sum(np.dot(np.cos(sigmas[:, 2]), Wm))
#    x[0] = np.sum(np.dot(sigmas[:, 0], Wm))
#    x[1] = np.sum(np.dot(sigmas[:, 1], Wm))
#    x[2] = atan2(sum_sin, sum_cos) 
#    return x


def run_sim(measurements, controlIn, truth):
    points = MerweScaledSigmaPoints(n=5, alpha=.00001, beta=2, kappa=0, subtract=residual_x)
 
    dt = 0.1
    ukf = UKF(dim_x=5, dim_z=3, fx=state_trans, hx=measurement_trans,
              dt=dt, points=points, x_mean_fn=state_mean, residual_x=residual_x)

    ukf.x = np.array([0, 0, 0, 0, 0])
    ukf.P = np.diag([.1, .1, .1, 1, 1])
    ukf.R = np.diag([0.01**2, 0.01**2, 0])
    ukf.Q = np.eye(5)*0.01


    for x in range(np.size(measurements, 0)):
        
        u = controlIn[x]
        z = measurements[x]
        ukf.predict(u=u)
        ukf.update(z, measurement=z)

        plt.plot(0, 0.4, 'go', alpha=0.3)

        if x % 10 == 0:
            plt.plot(ukf.x[0], ukf.x[1], 'ro', alpha=0.3)
            #plt.plot(ukf.x[3], ukf.x[4], 'bo', alpha=0.3)
            #plt.plot(position[x][0], position[x][1], 'go', alpha=0.3)
            plt.plot(truth[x][0], truth[x][1], 'ko', alpha=0.3)
            plot_covariance((ukf.x[0], ukf.x[1]), ukf.P[0:2, 0:2], std=.1, facecolor='g', alpha=0.3)


    #axes = plt.gca()
    #axes.set_xlim([-.5,.5])
    #axes.set_ylim([-.5,.5])
    plt.show()




truth = readData("truth")
data = readData("data")

measurements = data.filter(regex='^z')
controlIn = data.filter(regex='^u')

run_sim(measurements.to_numpy(), controlIn.to_numpy(), truth.to_numpy())

#print(measurement_trans(np.array([0., 0., 0., 0., 2])))

#plt.plot(truth.x, truth.y, 'bo')
#plt.plot(measurements.x, measurements.y, 'ro')
##plt.plot([truth.x[0], truth.x[2]], [truth.y[0], truth.y[2]], color='k', linestyle='-', linewidth=2)
#plt.show()
