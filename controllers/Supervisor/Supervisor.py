"""Supervisor controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, LED, DistanceSensor
from controller import Supervisor
import numpy as np
import matplotlib.pyplot as plt
from multiprocessing import Process
# plt.ion()

# create the Robot instance.
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

stopped = False

burger = robot.getFromDef('Burger')
# Supervisor
translationField = burger.getField('translation')

position = np.array([[0, 0, 0]])

freq = 0

keyboard = robot.getKeyboard()
keyboard.enable(timestep)

def plot_graph(*args):
    plt.plot([1,2,3,4])
    plt.draw()
    plt.show()

while robot.step(timestep) != -1:

    
    if freq == 100:
        translation = translationField.getSFVec3f()
        b =  np.array([translation[0], translation[1], translation[2]]);
        position = np.append(position, b)
        freq = 0
    freq += 1
    
    key=keyboard.getKey()
    if (key== 65):   #a
        p = Process(target=plot_graph)
        p.start()
        # p.start()
        # plt.plot([1,2,3,4])
        # plt.ion()
        # plt.show()
        # plt.draw()
        # plt.pause(0.001)
        print(position)
   
    pass

