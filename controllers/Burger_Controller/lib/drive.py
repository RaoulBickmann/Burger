from controller import Robot, Motor



def driveStraight(robot, speed = 1.5):
    

    leftWheel = robot.getMotor("left_motor")
    rightWheel = robot.getMotor("right_motor")

    leftWheel.setPosition(float('inf'))
    rightWheel.setPosition(float('inf'))

    
    leftWheel.setVelocity(speed);
    rightWheel.setVelocity(speed);