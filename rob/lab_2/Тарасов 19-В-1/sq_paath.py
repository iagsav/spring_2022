"""Sample Webots controller for the square path benchmark."""

from controller import Robot
robot = Robot()

leftWheel = robot.getMotor('left wheel')
rightWheel = robot.getMotor('right wheel')

def rotateRight(x):
    leftWheel.setPosition(1000)
    rightWheel.setPosition(-1000)
    robot.step(x)

def goStraight(x):
    leftWheel.setPosition(1000)
    rightWheel.setPosition(1000)
    robot.step(x)

def goSquere():
    goStraight(3900)  #
    rotateRight(464)  #
    goStraight(3950)  #
    rotateRight(465)  #
    goStraight(3980)
    rotateRight(464)
    goStraight(3900)

goSquere()


leftWheel.setVelocity(0)
rightWheel.setVelocity(0)
