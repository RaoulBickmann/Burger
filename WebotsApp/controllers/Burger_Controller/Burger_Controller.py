"""Burger_Controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, LED, DistanceSensor
from controller import Robot, DistanceSensor, Motor, PositionSensor
import numpy as np
from lib import drive as d
from controller import Keyboard
# import struct, math
from math import atan2

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  led = robot.getLED('ledname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

emitter = robot.getEmitter("emitter")

sensorFLL = robot.getDistanceSensor("FLL")
sensorFL = robot.getDistanceSensor("FL")
sensorFR = robot.getDistanceSensor("FR")
sensorFRR = robot.getDistanceSensor("FRR")

lidar = robot.getLidar("lidar")

sensorFLL.enable(timestep)
sensorFL.enable(timestep)
sensorFR.enable(timestep)
sensorFRR.enable(timestep)

leftWheel = robot.getMotor("left_motor")
rightWheel = robot.getMotor("right_motor")

leftOdo = robot.getPositionSensor("left_odo")
rightOdo = robot.getPositionSensor("right_odo")

leftLastPos = 0;
rightLastPos = 0;

leftOdo.enable(timestep)
rightOdo.enable(timestep)


lidar.enable(timestep)
lidar.enablePointCloud()



controlIn = np.zeros((2, ))

freq = 0

TIRE_RAD = 0.02
AXLE_LEN = 0.065 * 2
MAX_RANGE = lidar.getMaxRange()

total = 0

keyboard = robot.getKeyboard()
keyboard.enable(timestep)

simStep = 10


def sendData(data):
    message = str(data)
    emitter.send(message.encode('utf-8')) 
    
    
def normalize_angle(x):
    x = x % (2 * np.pi)    # force in range [0, 2 pi)
    if x > np.pi:          # move to [-pi, pi)
        x -= 2 * np.pi
    return x

mode = 0

# 0 stop
# 1 forward
# 2 left
# 3 right

def steering(key):
    u = np.zeros((2,))
    if (key == 87):   #w
        u[0] = 1
        u[1] = 1
        rightWheel.setVelocity(u[0]);
        leftWheel.setVelocity(u[1]);
        return 1, u
    elif (key == 83):   #s
        u[0] = 0
        u[1] = 0
        rightWheel.setVelocity(u[0]);
        leftWheel.setVelocity(u[1]);
        return 0, u
    elif (key == 65):   #a
        u[0] = 1
        u[1] = -1
        rightWheel.setVelocity(u[0]);
        leftWheel.setVelocity(u[1]);
        #u[1] = -(2 * TIRE_RAD)/AXLE_LEN
        # u[1] = math.asin(2 * TIRE_RAD/AXLE_LEN) #2 weil sich das andere rad gegen dreht
        return 2, u
    elif (key == 68):   #d
        u[0] = -1
        u[1] = 1
        rightWheel.setVelocity(u[0]);
        leftWheel.setVelocity(u[1]);
        #u[1] = (2 * TIRE_RAD)/AXLE_LEN
        # u[1] = -math.asin(2 * TIRE_RAD/AXLE_LEN) #2 weil sich das andere rad gegen dreht
        return 3, u
    else:
        return mode, controlIn

leftWheel.setPosition(float('inf'))
rightWheel.setPosition(float('inf'))
    
rightWheel.setVelocity(0);
leftWheel.setVelocity(0);

    
while robot.step(timestep) != -1:
    
    # print(
        # lidar.getRangeImage()
        #sensorFLL.getValue(), 
        # sensorFL.getValue(), 
        # sensorFR.getValue(), 
        #sensorFRR.getValue()
        # )
        
    key = keyboard.getKey()
    if(key != -1):
        mode, controlIn = steering(key)    
    
    if freq == 10:
        newLeft = leftOdo.getValue()
        newRight = rightOdo.getValue()
        
    
        leftDist = newLeft - leftLastPos
        rightDist = newRight - rightLastPos
        
        leftLastPos = newLeft
        rightLastPos = newRight
               
        rangeImage = lidar.getRangeImage()
        counter = 0
        features = [[0.5,0]]
        cumulative = 0

        # print("step")
        # print(rangeImage)
        points = []
        for i in range(len(rangeImage)):
            distance = rangeImage[i]
            if(distance < 0.5):
                angle = normalize_angle(2*np.pi/128 * i)
                points.append([distance, angle])
                  
        
        points = np.array(points)
        meanDist = np.sum(points[:, 0])/len(points)

        #Mean Winkel
        sum_sin = np.sum(np.sin(points[:, 1]))
        sum_cos = np.sum(np.cos(points[:, 1]))
        
        meanAngle = - atan2(sum_sin, sum_cos)
                                  
        #einzelne werte
        # print(features)               
                              
        data  = np.array([leftDist, rightDist, 
        meanDist,
        meanAngle
        # sensorFL.getValue(), 
        #sensorFR.getValue()
        ])
        
        data = np.concatenate([data, controlIn])
        # data = np.concatenate([data, rangeImage])
        data = ', '.join(str(x) for x in data)
        sendData(data)        

        
        freq = 0
    freq += 1
    pass









