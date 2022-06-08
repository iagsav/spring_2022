"""my_controller_square controller."""
"""Sample Webots controller for the square path benchmark."""

from controller import Robot
import math

# Variables
wheelDiameter = 0.195
wheelRadius = wheelDiameter/2
wheelDistance = 0.33
MAX_SPEED = 5.24
prevValueSensor = 0

# delta = [[0.060, 0.055, 0.060, 0.04], 
#         [ 0.065, 0.055, 0.060, 0.00]]


delta = [[0.065, 0.055, 0.060], 
        [ 0.065, 0.045, 0.055]]

quarterInRads = (wheelDistance*math.pi/4)/wheelRadius

# Get distance from prev to current rads, returns meters
def getDistance(sensor, prevValue, radius=wheelRadius):
    return radius*sensor.getValue() - prevValue

# Get pointer to the robot.
robot = Robot()


# Get pointer to each wheel of our robot.
leftWheel = robot.getMotor('left wheel')
rightWheel = robot.getMotor('right wheel')

# Get sensors
leftWheelSensor = robot.getPositionSensor('left wheel sensor')
rightWheelSensor = robot.getPositionSensor('right wheel sensor')

# Enable sensors
leftWheelSensor.enable(16)
rightWheelSensor.enable(16)

leftWheel.setVelocity(MAX_SPEED)
rightWheel.setVelocity(MAX_SPEED)

# Repeat the following 4 times (once for each side).
for side in range(0, 4):
    # First set both wheels to go forward, so the robot goes straight.
    leftWheel.setPosition(1000)
    rightWheel.setPosition(1000)
    robot.step(16)

    # While robot not reached corner
    while (getDistance(rightWheelSensor, prevValueSensor) < 2.0):
        # If corner is near then set slow speed
        if (getDistance(rightWheelSensor, prevValueSensor) > 1.95):
            leftWheel.setVelocity(0.6*MAX_SPEED)
            rightWheel.setVelocity(0.6*MAX_SPEED)
        robot.step(160)
    
    # robot spin
    if (side == 3):
        # Robot shouldn't spin at the last corner
        break
    else:
        leftWheel.setPosition(leftWheelSensor.getValue() + quarterInRads + delta[0][side])
        rightWheel.setPosition(rightWheelSensor.getValue() - quarterInRads - delta[1][side])
    
    robot.step(1000)
    
    # Update prev rads
    prevValueSensor = rightWheelSensor.getValue() * wheelRadius
    
    leftWheel.setVelocity(MAX_SPEED)
    rightWheel.setVelocity(MAX_SPEED)
# Stop the robot when path is completed, as the robot performance
# is only computed when the robot has stopped.
leftWheel.setVelocity(0)
rightWheel.setVelocity(0)
