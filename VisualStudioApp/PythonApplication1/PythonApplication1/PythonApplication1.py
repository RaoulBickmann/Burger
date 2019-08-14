import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from math import sin, cos, sqrt, atan2, radians
from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.stats import plot_covariance


TIRE_RAD = 0.02
AXLE_LEN = 0.065 * 2
SENSOR_OFFSET = 0.05

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
    x = +x

    l = u[0] * TIRE_RAD * dt
    r = u[1] * TIRE_RAD * dt

    dist = min(l, r)

    a = (l - r)/AXLE_LEN

    if(abs(a) > 0.00001):
        x[2] = normalize_angle(x[2] + a)
    else:
        x[0] += cos(x[2]) * dist
        x[1] += sin(x[2]) * dist

    return x;

#measurement vector from previous state and control
def measurement_trans(x, u, dt):
    
    x = +x

    y = np.zeros(3)
    
    y[0] = u[0] * dt
    y[1] = u[1] * dt
    y[2] = 1.0

    #entfernungsvector des hindernisses
    direction = np.array([x[3] - x[0], x[4] - x[1]])

    #diff zwischen blickrichtung des roboters und richtung in der sich das hindernis befindet
    angle = normalize_angle(atan2(direction[1], direction[0]) - x[2]) 
    
    #wenn diff klein genug, misst sensor die entfernung 
    if(abs(angle) < radians(1)):
        y[2] = sqrt(direction[0]**2 + direction[1]**2)
        y[2] -= SENSOR_OFFSET
    #beschränkt entfernung auf 1
    if(y[2] >= 1):
        y[2] = 1.0

    return y

#Berechnung des mean der Zustände z=[xRobo, yRobo, Orientierung, xHindernis, yHindernis]
def state_mean(sigmas, Wm):
    x = np.zeros(5)


    x[0] = np.sum(np.dot(sigmas[:, 0], Wm))
    x[1] = np.sum(np.dot(sigmas[:, 1], Wm))

    #Mean Winkel
    sum_sin = np.sum(np.dot(np.sin(sigmas[:, 2]), Wm))
    sum_cos = np.sum(np.dot(np.cos(sigmas[:, 2]), Wm))
    x[2] = atan2(sum_sin, sum_cos)

    x[3] = np.sum(np.dot(sigmas[:, 3], Wm))                      
    x[4] = np.sum(np.dot(sigmas[:, 4], Wm))                      
    return x

#Berechnung des mean der Messwerte z=[linkeOdometrie, rechteOdometrie, distanzInCm]
def z_mean(sigmas, Wm):
    z = np.zeros(3)

    z[0] = np.sum(np.dot(sigmas[:, 0], Wm))
    z[1] = np.sum(np.dot(sigmas[:, 1], Wm))
    
    #Filterung der Messwerte bei denen distanz = 1
    dmean = 0.
    dcounter = 0.
    for i in range(np.size(sigmas, 0)):
        if(sigmas[i, 2] != 1):
            print(sigmas[i, 2], Wm[i])
            dmean += sigmas[i, 2] * Wm[i]
            dcounter += 1

    if(dcounter > 0):
        z[2] = dmean / dcounter
    #z[2] = np.sum(np.dot(sigmas[:, 2], Wm))
    return z


def run_sim(measurements, controlIn, truth):
    points = MerweScaledSigmaPoints(n=5, alpha=.00001, beta=2, kappa=0, subtract=residual_x)
 
    dt = 0.1
    ukf = UKF(dim_x=5, dim_z=3, fx=state_trans, hx=measurement_trans,
              dt=dt, points=points, x_mean_fn=state_mean, z_mean_fn=z_mean, residual_x=residual_x)

    ukf.x = np.array([0, 0, 0, 0, 0])
    ukf.P = np.diag([.01, .01, .01, 10000000, 10000000])
    ukf.R = np.diag([0.1**2, 0.1**2, 0**2])
    ukf.Q = np.diag([0.1**2, 0.1**2, 0.01**2, 0.01**2, 0.01**2])


    for x in range(np.size(measurements, 0)):
        
        u = controlIn[x]
        z = measurements[x]
        ukf.predict(u=u)
        
        #R hoch stzen bei sensor output 1
        if(z[2] >= 1):
            badR = np.diag([0.1**2, 0.1**2, 10000000**2])
            ukf.update(z, badR, u = u, dt = dt)
        else:
            ukf.update(z, u = u, dt = dt)

        plt.plot(0.45, 0, 'go', alpha=0.3)


        if x % 10 == 0:
            #plt.plot(ukf.x[0], ukf.x[1], 'ro', alpha=0.3)
            plt.plot(ukf.x[3], ukf.x[4], 'bo', alpha=0.3)
            plt.plot(ukf.x[0], ukf.x[1], 'go', alpha=0.3)
            plt.plot(truth[x][0], truth[x][1], 'ko', alpha=0.3)
            #plot_covariance((ukf.x[0], ukf.x[1]), ukf.P[0:2, 0:2], std=.1, facecolor='g', alpha=0.3)
            print(ukf.x[3], ukf.x[4])

    plt.show()


def test_measurement_trans(state, expected_outcome):
    z = measurement_trans(state, u = np.array([0,0]), dt=0.1)
    if(np.allclose(z, expected_outcome)):
        print("passed")
    else:
        print("failed with: ")
        print(z[2], expected_outcome[2])

truth = readData("truth")
data = readData("data")

measurements = data.filter(regex='^z')
controlIn = data.filter(regex='^u')

run_sim(measurements.to_numpy(), controlIn.to_numpy(), truth.to_numpy())

#testing measurments trans
#test_measurement_trans(np.array([0., 0., 0., 0.45, 0.]), np.array([0., 0., 0.4]))
#test_measurement_trans(np.array([0., 0., 0., 0.2, 0.]), np.array([0., 0., 0.15]))
#test_measurement_trans(np.array([0., 0., 0., 0.8, 0.]), np.array([0., 0., 0.75]))
#test_measurement_trans(np.array([0., 0., np.pi, -0.45, 0.]), np.array([0., 0., 0.4]))
#test_measurement_trans(np.array([0., 0., np.pi/2, 0., 0.3]), np.array([0., 0., 0.25]))
#test_measurement_trans(np.array([0.2, 1., 0, 0.5, 1]), np.array([0., 0., 0.25]))
#test_measurement_trans(np.array([0., 0., np.pi, 0.5, 0.]), np.array([0., 0., 1.]))
#test_measurement_trans(np.array([0., 0., -np.pi/2, 0., 0.5]), np.array([0., 0., 1.]))
#test_measurement_trans(np.array([0., 0., 0., 0.5, 0.]), np.array([0., 0., 0.45]))
#test_measurement_trans(np.array([0., 0., 0., 1.5, 0.]), np.array([0., 0., 1.]))



#print(state_trans(np.array([0., 0., 0., 0.4, 0.]), 0.1, np.array([0,0])))

#plt.plot(truth.x, truth.y, 'bo')
#plt.plot(measurements.x, measurements.y, 'ro')
##plt.plot([truth.x[0], truth.x[2]], [truth.y[0], truth.y[2]], color='k', linestyle='-', linewidth=2)
#plt.show()
