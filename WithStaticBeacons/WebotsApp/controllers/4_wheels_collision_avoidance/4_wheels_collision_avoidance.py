"""4_wheels_collision_avoidance controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, LED, DistanceSensor
from controller import Robot, DistanceSensor, Motor

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  led = robot.getLED('ledname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)


left = robot.getDistanceSensor("ds_left")
right = robot.getDistanceSensor("ds_right")

left.enable(timestep)
right.enable(timestep)

# initialize motors
wheels = []
wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
for name in wheelsNames:
    wheels.append(robot.getMotor(name))


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # read sensors outputs
    leftVal = left.getValue();
    rightVal= right.getValue();
    
    # Process sensor data here.
    # detect obstacles
    left_obstacle = rightVal > 70.0
    right_obstacle = leftVal > 70.0

    leftSpeed = 1.5  # [rad/s]
    rightSpeed = 1.5  # [rad/s]
    

    # modify speeds according to obstacles
    if left_obstacle:
        # turn right
        leftSpeed  += 0.5
        rightSpeed -= 0.5
    elif right_obstacle:
        # turn left
        leftSpeed  -= 0.5
        rightSpeed += 0.5 
    # write actuators inputs   
        
    for wheel in wheels:
        wheel.setPosition(float('inf'))
    
            
    wheels[0].setVelocity(leftSpeed);
    wheels[1].setVelocity(leftSpeed);
    wheels[2].setVelocity(rightSpeed);
    wheels[3].setVelocity(rightSpeed);
    
     
    pass

# Enter here exit cleanup code.














