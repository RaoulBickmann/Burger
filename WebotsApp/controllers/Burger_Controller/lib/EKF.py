import numpy as np
import sympy as sp
from filterpy.kalman import ExtendedKalmanFilter as EKF

# Matrices
R = 0
Q = 0
G = 0
K = 0
H = 0


#EKF

def slam_Prediction(prev_mean, prev_cova, control_in, observations, map_size):
    map_size_new = map_size
    
    F = np.matrix



# def slam_Correction():
