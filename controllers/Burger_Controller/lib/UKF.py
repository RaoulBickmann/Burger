from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.kalman import UnscentedKalmanFilter as UKF


# x = [x, y, angle] Zustandsvektor
# dt = zeit zwischen steps
# u = [leftSpeed, rightSpeed]           Control Input

R = np.array()
Q = 


def state_transition(x, dt, u):

    if(u[0] == u[1])   #straight
        
    

    
    
def measurement(x):
    




def run():
    
    
    points = MerweScaledSigmaePoints(n = 5, alpha=.00001, beta=2, kappa=0, )