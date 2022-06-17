"""Braitenberg-based obstacle-avoiding robot controller."""

from controller import Robot

# Get reference to the robot.
robot = Robot()

direction = 0

# Get simulation step length.
timeStep = int(robot.getBasicTimeStep())

# Constants of the Thymio II motors and distance sensors.
maxMotorVelocity = 9.53
distanceSensorCalibrationConstant = 360

# Get left and right wheel motors.
leftMotor = robot.getMotor("motor.left")
rightMotor = robot.getMotor("motor.right")

# Get frontal distance sensors.
outerLeftSensor = robot.getDistanceSensor("prox.horizontal.0")
centralLeftSensor = robot.getDistanceSensor("prox.horizontal.1")
centralSensor = robot.getDistanceSensor("prox.horizontal.2")
centralRightSensor = robot.getDistanceSensor("prox.horizontal.3")
outerRightSensor = robot.getDistanceSensor("prox.horizontal.4")

# Enable distance sensors.
outerLeftSensor.enable(timeStep)
centralLeftSensor.enable(timeStep)
centralSensor.enable(timeStep)
centralRightSensor.enable(timeStep)
outerRightSensor.enable(timeStep)

# Disable motor PID control mode.
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# Set ideal motor velocity.
initialVelocity = 0.7 * maxMotorVelocity

# Set the initial velocity of the left and right wheel motors.
leftMotor.setVelocity(initialVelocity)
rightMotor.setVelocity(initialVelocity)

while robot.step(timeStep) != -1:
    # Read values from four distance sensors and calibrate.
    outerLeftSensorValue = outerLeftSensor.getValue() / distanceSensorCalibrationConstant
    centralLeftSensorValue = centralLeftSensor.getValue() / distanceSensorCalibrationConstant
    centralSensorValue = centralSensor.getValue() / distanceSensorCalibrationConstant
    centralRightSensorValue = centralRightSensor.getValue() / distanceSensorCalibrationConstant
    outerRightSensorValue = outerRightSensor.getValue() / distanceSensorCalibrationConstant

    leftV = initialVelocity - (centralRightSensorValue + outerRightSensorValue) / 1.0
    rightV = initialVelocity - (centralLeftSensorValue + outerLeftSensorValue) / 1.0 - centralSensorValue

    turning = rightV - leftV
    print('--------------------------------------')
    print(turning)
    if turning == 0 and direction != 0 :
        if direction > 0:
            rightV = rightV - min(direction, 1)
        else:
            leftV = leftV + max(direction, -1)
    turning = rightV - leftV

    direction = direction + turning
    print(direction)
    print(turning)
    # Set wheel velocities based on sensor values, prefer right turns if the central sensor is triggered.
    leftMotor.setVelocity(leftV)
    rightMotor.setVelocity(rightV)


