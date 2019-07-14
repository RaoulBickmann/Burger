"""Burger_Controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, LED, DistanceSensor
from controller import Robot, DistanceSensor, Motor, PositionSensor
import numpy as np
from lib import drive as d
from lib import EKF as ekf
from controller import Keyboard
import struct, math


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


position = np.array([0 , 0, 0])

freq = 0

TIRE_RAD = 0.02
AXLE_LEN = 0.065 * 2
    

def sendData(data):
    message = str(data)
    emitter.send(message.encode('utf-8'))   

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    
    # print(sensorFLL.getValue(), 
        # sensorFL.getValue(), 
        # sensorFR.getValue(), 
        # sensorFRR.getValue())
        
    # print(leftOdo.getValue(), rightOdo.getValue())
    
    leftWheel.setPosition(float('inf'))
    rightWheel.setPosition(float('inf'))
    

    leftWheel.setVelocity(1);
    rightWheel.setVelocity(1);
    
    
            
    # leftWheel.setPosition(float(3.14))
    # rightWheel.setPosition(float(3.14))
            
    
    if freq == 62:
        leftDist = round((leftOdo.getValue() - leftLastPos) * TIRE_RAD, 4)
        rightDist = round((rightOdo.getValue() - rightLastPos) * TIRE_RAD, 4)
        leftLastPos = leftOdo.getValue()
        rightLastPos = rightOdo.getValue()
        
        diff = leftDist - rightDist
        circumf = AXLE_LEN * math.pi
        b = diff/circumf
        print(b)
        
        
        print(leftDist, rightDist)
        b =  np.array([0, leftDist, 0])
        position = np.add(position, b)
        freq = 0
    freq += 1

    # Process sensor data here.
    # d.driveStraight(robot)

    # Enter here functions to send actuator commands, like:
    #  led.set(1)
    pass

# Enter here exit cleanup code.








