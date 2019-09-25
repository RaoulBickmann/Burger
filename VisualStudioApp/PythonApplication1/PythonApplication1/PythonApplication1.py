import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from math import sin, cos, sqrt, atan2, radians
from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.stats import plot_covariance
import posdef as ps

TIRE_RAD = 0.02
AXLE_LEN = 0.065 * 2
#SENSOR_OFFSET = 0.05



def readData(FileName):
	data = pd.read_csv('%s.csv'%FileName)
	
	return data

def normalize_angle(x):
    x = x % (2 * np.pi)     # force in range [0, 2 pi)
    if x > np.pi:          # move to [-pi, pi)
        x -= 2 * np.pi
    return x


def residual_x(a, b):
    y = a - b
    y[2] = normalize_angle(y[2])
    return y

def residual_h(a, b):
    y = a - b
    y[1] = normalize_angle(y[1])
    return y


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

#state vector from previous state and control, returns measurement
def measurement_trans(x, u, dt):
    
    x = +x

    y = np.zeros(2)
    
    #y[0] = u[0] * dt
    #y[1] = u[1] * dt

    direction = np.array([0 - x[0], 0 - x[1]])
    distance = sqrt(direction[0]**2 + direction[1]**2)

    y[0] = distance
    y[1] = normalize_angle(atan2(direction[1], direction[0]) - x[2])

    return y

#Berechnung des mean der Zustände z=[xRobo, yRobo, Orientierung, xHindernis, yHindernis]
def state_mean(sigmas, Wm):
    x = np.zeros(3)

    x[0] = np.sum(np.dot(sigmas[:, 0], Wm))
    x[1] = np.sum(np.dot(sigmas[:, 1], Wm))

    #Mean Winkel
    sum_sin = np.sum(np.dot(np.sin(sigmas[:, 2]), Wm))
    sum_cos = np.sum(np.dot(np.cos(sigmas[:, 2]), Wm))
    x[2] = atan2(sum_sin, sum_cos)

    #x[3] = np.sum(np.dot(sigmas[:, 3], Wm))                      
    #x[4] = np.sum(np.dot(sigmas[:, 4], Wm))                      
    return x

#Berechnung des mean der Messwerte z = [linkeOdometrie, rechteOdometrie, distanzInCm, angleInRadians]
def z_mean(sigmas, Wm):
    z = np.zeros(2)

    z[0] = np.sum(np.dot(sigmas[:, 0], Wm))
    #z[1] = np.sum(np.dot(sigmas[:, 1], Wm))
    #z[2] = np.sum(np.dot(sigmas[:, 2], Wm))

    #Mean Winkel
    sum_sin = np.sum(np.dot(np.sin(sigmas[:, 1]), Wm))
    sum_cos = np.sum(np.dot(np.cos(sigmas[:, 1]), Wm))
    z[1] = atan2(sum_sin, sum_cos)
    return z


def run_sim(measurements, controlIn, truth):
    points = MerweScaledSigmaPoints(n=3, alpha=.00001, beta=2, kappa=0, subtract=residual_x)
 
    dt = 0.1            #0.1s timestep
    ukf = UKF(dim_x=3, dim_z=2, fx=state_trans, hx=measurement_trans,
              dt=dt, points=points, x_mean_fn=state_mean, z_mean_fn=z_mean, residual_x=residual_x, residual_z=residual_h)

    ukf.x = np.array([0, 0, 0])
    ukf.P = np.diag([.01, .01, .01])
    #ukf.R = np.diag([0.1**2, 0.1**2, 0**2])
    ukf.R = np.diag([0.01**2, 0.01**2])           #measurement noise = [linkeOdometrie, rechteOdometrie, distanzInCm, angleInRadians]
    ukf.Q = np.diag([0.001**2, 0.001**2, 0.2**2])  #process noise = [xRobo, yRobo, Orientierung, xHindernis, yHindernis]

    for x in range(np.size(measurements, 0)):
        
        u = controlIn[x]
        z = measurements[x][2:3]

        ukf.predict(u=u)
        
        #R hoch stzen bei sensor output 1
        #if(z[2] >= 1):
        #    badR = np.diag([0.1**2, 0.1**2, 10000000**2])
        #    ukf.update(z, badR, u = u, dt = dt)
        #else:
        ukf.update(z, u = u, dt = dt)

        try:
            _ = np.linalg.cholesky(ukf.P)
        except np.linalg.LinAlgError:
            ukf.P = ps.nearestPD(ukf.P)

        angle = - np.pi/2
        rotation = np.array([[cos(angle), -sin(angle)], [sin(angle), cos(angle)]])

        if x % 1 == 0:

            #roboX = ukf.x[0] * rotation[0][0] + ukf.x[1] * rotation[0][1]
            #roboY = ukf.x[0] * rotation[1][0] + ukf.x[1] * rotation[1][1]

            #obsX = ukf.x[3] * rotation[0][0] + ukf.x[4] * rotation[0][1]
            #obsY = ukf.x[3] * rotation[1][0] + ukf.x[4] * rotation[1][1]

            #plt.plot(roboX, roboY, 'go', alpha=0.3)
            #plt.plot(obsX,  obsY, 'bo', alpha=0.3)


            plt.plot(ukf.x[0], ukf.x[1], 'go', alpha=0.3)
            #plt.plot(ukf.x[3], ukf.x[4], 'bo', alpha=0.3)
            #print(ukf.x[2])

            #echte position roboter
            plt.plot(truth[x][0], truth[x][1], 'ko', alpha=0.3)


            #plot_covariance((ukf.x[0], ukf.x[1]), ukf.P[0:2, 0:2], std=.1, facecolor='g', alpha=0.3)

    #echte position hindernis
    plt.plot(0 , 0.25, 'ro', alpha=0.3)
    plt.show()


def test_measurement_trans(state, expected_outcome):
    z = measurement_trans(state, u = np.array([0,0]), dt=0.1)
    if(np.allclose(z, expected_outcome)):
        print("passed")
    else:
        print("failed with: ")
        print(z, expected_outcome)

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

#test_measurement_trans(np.array([0., 0.,  0., 0.3, 0.]), np.array([0., 0., 0.3, 0]))
#test_measurement_trans(np.array([0., 0.,  1.57, 0.3, 0.]), np.array([0., 0., 0.3, 1.57]))

#print(state_trans(np.array([0., 0., 0., 0.4, 0.]), 0.1, np.array([0,0])))

#plt.plot(truth.x, truth.y, 'bo')
#plt.plot(measurements.x, measurements.y, 'ro')
##plt.plot([truth.x[0], truth.x[2]], [truth.y[0], truth.y[2]], color='k', linestyle='-', linewidth=2)
#plt.show()
