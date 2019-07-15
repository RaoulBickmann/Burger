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

total = 0

a = False
keyboard = robot.getKeyboard()
keyboard.enable(timestep)


def sendData(data):
    message = str(data)
    emitter.send(message.encode('utf-8')) 
    

# def convertAngle(total, newAngle):
    # if(total >= 0 && total + newAngle >= ):
        
    # elif(total < 0)
    
def normalize_angle(x):
    x = x % (2 * np.pi)    # force in range [0, 2 pi)
    if x > np.pi:          # move to [-pi, pi)
        x -= 2 * np.pi
    return x


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
    
    key=keyboard.getKey()
    if (key== 65):   #a
       a = True
   
    if(a):
        rightWheel.setVelocity(1);
    else:
        rightWheel.setVelocity(0);
        leftWheel.setVelocity(0);

    
            
    # leftWheel.setPosition(float(3.14))
    # rightWheel.setPosition(float(3.14))
            
    
    if freq == 2:
        leftDist = (leftOdo.getValue() - leftLastPos) * TIRE_RAD
        rightDist = (rightOdo.getValue() - rightLastPos) * TIRE_RAD
        leftLastPos = leftOdo.getValue()
        rightLastPos = rightOdo.getValue()
        
        diff = leftDist - rightDist
        
        
        angle = diff /(AXLE_LEN)
        total = total + angle
        # print(diff, total, total/0.4084* 100)
        print(normalize_angle(total))
        b =  np.array([rightDist, leftDist, angle])
        position = np.add(position, b)
        # print(position[2])
        freq = 0
    freq += 1

    # Process sensor data here.
    # d.driveStraight(robot)

    # Enter here functions to send actuator commands, like:
    #  led.set(1)
    pass

# Enter here exit cleanup code.








