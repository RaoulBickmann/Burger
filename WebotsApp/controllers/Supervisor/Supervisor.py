"""Supervisor controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, LED, DistanceSensor
from controller import Supervisor
import numpy as np
import pandas as pd


# create the Robot instance.
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

stopped = False

burger = robot.getFromDef('Burger')
# Supervisor
translationField = robot.getFromDef('ROBOT_POSITION')
rotationField = burger.getField('rotation')

groundtruth = np.zeros((3, 1)).transpose()
burgerData = np.zeros((5, 1)).transpose()

freq = 0

keyboard = robot.getKeyboard()
keyboard.enable(timestep)

receiver = robot.getReceiver("receiver")
receiver.enable(timestep)

lastDist = 0
np.set_printoptions(suppress=True)

while robot.step(timestep) != -1:

    if freq == 5:
        translation = translationField.getPosition()
        orientation = translationField.getOrientation()
        
        zOri = np.array([orientation[6], orientation[8]])
        angle = np.arctan2(zOri[0], zOri[1])
       
        b =  np.array([translation[2], translation[0], angle])
        groundtruth = np.vstack((groundtruth, b))
        freq = 0
    freq += 1
    
    key=keyboard.getKey()
    if (key== 67):   #c
        dfTruth = pd.DataFrame(groundtruth, columns=['x', 'y', 'a'])
        dfData = pd.DataFrame(burgerData, 
            columns=['zl', 'zr', 'zFL', 'uv', 'ua'])
        dfTruth.to_csv("truth.csv", index = False)
        dfData.to_csv("data.csv", index = False)
        print("done")
   
    if receiver.getQueueLength() > 0:
        message = receiver.getData().decode('utf-8')
        receiver.nextPacket()
        data = np.fromstring(message, sep=",")
        burgerData = np.vstack((burgerData, data))
pass

