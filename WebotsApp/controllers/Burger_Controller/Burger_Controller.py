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


controlIn = np.zeros((2, ))

freq = 0

TIRE_RAD = 0.02
AXLE_LEN = 0.065 * 2

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
        rightWheel.setVelocity(1);
        leftWheel.setVelocity(1);
        u[0] = 1 * TIRE_RAD
        u[1] = 0
        return 1, u
    elif (key == 83):   #s
        rightWheel.setVelocity(0);
        leftWheel.setVelocity(0);
        u[0] = 0
        u[1] = 0
        return 0, u
    elif (key == 65):   #a
        rightWheel.setVelocity(1);
        leftWheel.setVelocity(-1);
        u[0] = 0
        u[1] = -(2 * TIRE_RAD)/AXLE_LEN
        # u[1] = math.asin(2 * TIRE_RAD/AXLE_LEN) #2 weil sich das andere rad gegen dreht
        return 2, u
    elif (key == 68):   #d
        rightWheel.setVelocity(-1);
        leftWheel.setVelocity(1);
        u[0] = 0
        u[1] = (2 * TIRE_RAD)/AXLE_LEN
        # u[1] = -math.asin(2 * TIRE_RAD/AXLE_LEN) #2 weil sich das andere rad gegen dreht
        return 3, u
    else:
        return mode, controlIn

leftWheel.setPosition(float('inf'))
rightWheel.setPosition(float('inf'))
    
rightWheel.setVelocity(0);
leftWheel.setVelocity(0);

    
while robot.step(timestep) != -1:
    
    print(
        #sensorFLL.getValue(), 
        sensorFL.getValue(), 
        sensorFR.getValue(), 
        #sensorFRR.getValue()
        )
        
    key = keyboard.getKey()
    if(key != -1):
        mode, controlIn = steering(key)    
    
    if freq == 5:
        newLeft = leftOdo.getValue()
        newRight = rightOdo.getValue()
        
    
        leftDist = newLeft - leftLastPos
        rightDist = newRight - rightLastPos
        
        leftLastPos = newLeft
        rightLastPos = newRight
               
               
        data  = np.array([leftDist, rightDist, 
        sensorFL.getValue(), 
        #sensorFR.getValue()
        ])
        
        data = np.concatenate([data, controlIn])
        data = ', '.join(str(x) for x in data)
        sendData(data)        
        
        freq = 0
    freq += 1

    # Process sensor data here.
    # d.driveStraight(robot)

    # Enter here functions to send actuator commands, like:
    #  led.set(1)
    pass

# Enter here exit cleanup code.







