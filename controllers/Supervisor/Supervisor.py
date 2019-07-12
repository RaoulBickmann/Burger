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

position = np.array([[0, 0, 0]])

freq = 0

keyboard = robot.getKeyboard()
keyboard.enable(timestep)

receiver = robot.getReceiver("receiver")
receiver.enable(timestep)

lastDist = 0


while robot.step(timestep) != -1:

    
    if freq == 62:
        translation = translationField.getPosition()
        m = rotationField.getSFRotation()
        # rotation = np.arctan2(m[0], m[6])
        print(translation[2] - lastDist)
        lastDist = translation[2]
        b =  np.array([round(translation[0], 2), round(translation[2], 2), round(m[1] * m[3], 2)])
        position = np.vstack((position, b))
        freq = 0
    freq += 1
    
    key=keyboard.getKey()
    if (key== 65):   #a
        df = pd.DataFrame(position, columns=['x', 'y', 'a'])
        df.to_csv("test.csv")
   
   
   
    if receiver.getQueueLength() > 0:
        message = receiver.getData().decode('utf-8')
        receiver.nextPacket()
        print('I should ' + message + '!')   
pass

