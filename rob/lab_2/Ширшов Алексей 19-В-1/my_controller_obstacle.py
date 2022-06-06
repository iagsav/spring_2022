"""my_controller_obstacle controller."""
"""Braitenberg-based obstacle-avoiding robot controller."""

from controller import Robot, Compass

# Get reference to the robot.
robot = Robot()

# Get simulation step length.
timeStep = int(robot.getBasicTimeStep())

# Constants of the Thymio II motors and distance sensors.
MAX_SPEED = 9.53
distanceSensorCalibrationConstant = 360
prev_y = 0

k_p = 0.001
k_d = 0.001
k_i = 0.003

sum_left = [0] * 5
sum_right = [0] * 5

max_ui = 6
min_ui = -6

# PID Controller for distance sensor
def PIDctl(y, sensors):
    global sum_left
    global sum_right

    # Get sensors values
    left = (sensors[0].getValue() + sensors[1].getValue()) /2
    right = (sensors[3].getValue() + sensors[4].getValue()) /2
    if (left > right):
        left += sensors[2].getValue()
    else:
        right += sensors[2].getValue()

    # Update values for integral
    sum_left.pop(0)
    sum_right.pop(0)
    sum_left.append(left)
    sum_right.append(right)

    # Integral
    ui_left = sum(sum_left) * k_i
    ui_right = sum(sum_right) * k_i

    if (ui_left > max_ui):
        ui_left = max_ui
    elif (ui_left < min_ui):
        ui_left = min_ui

    if (ui_right > max_ui):
        ui_right = max_ui
    elif (ui_right < min_ui):
        ui_right = min_ui

    # Proportional
    up_left = left * k_p
    up_right = right * k_p

    # Differential
    ud_left = (left - sum_left[3]) * k_d
    ud_right = (right - sum_right[3]) * k_d

    # Debug
    #print("%3.3f %3.3f %3.3f | %3.3f %3.3f %3.3f" % (up_left, ud_left, ui_left, up_right, ud_right, ui_right))

    # return(up_left, up_right)
    return(up_left + ud_left + ui_left, up_right + ud_right + ui_right)


# Get left and right wheel motors.
leftMotor = robot.getDevice("motor.left")
rightMotor = robot.getDevice("motor.right")

# Get compass
compass = robot.getDevice("compass")
compass.enable(timeStep)

# Get frontal distance sensors.
outerLeftSensor = robot.getDevice("prox.horizontal.0")
centralLeftSensor = robot.getDevice("prox.horizontal.1")
centralSensor = robot.getDevice("prox.horizontal.2")
centralRightSensor = robot.getDevice("prox.horizontal.3")
outerRightSensor = robot.getDevice("prox.horizontal.4")

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
initialVelocity = 0.5 * MAX_SPEED

# Set the initial velocity of the left and right wheel motors.
leftMotor.setVelocity(initialVelocity)
rightMotor.setVelocity(initialVelocity)

while robot.step(timeStep) != -1:

    outerLeftSensorValue = outerLeftSensor.getValue() / distanceSensorCalibrationConstant
    centralLeftSensorValue = centralLeftSensor.getValue() / distanceSensorCalibrationConstant
    centralSensorValue = centralSensor.getValue() / distanceSensorCalibrationConstant
    centralRightSensorValue = centralRightSensor.getValue() / distanceSensorCalibrationConstant
    outerRightSensorValue = outerRightSensor.getValue() / distanceSensorCalibrationConstant
    sensors = [outerLeftSensor, centralLeftSensor, centralSensor, centralRightSensor, outerRightSensor]

    # Get errors
    robot.step(40)
    (error_left, error_right) = PIDctl(compass.getValues()[1], sensors)

    # If there is no obstacles, then need straight by compass
    if ((sum(sum_left) + sum(sum_right)) < 0.1):
        y = compass.getValues()[1]
        # if robot turned right
        if (y > 0.01):
            leftMotor.setVelocity(MAX_SPEED - abs(y)*5)
            rightMotor.setVelocity(MAX_SPEED)
        else:
            leftMotor.setVelocity(MAX_SPEED)
            rightMotor.setVelocity(MAX_SPEED - abs(y)*5)
    else:
        # Set wheel velocities based on sensor values.
        leftMotor.setVelocity(initialVelocity - error_right)
        rightMotor.setVelocity(initialVelocity - error_left)


    # If there is "wall"
    if (error_left > 7 and error_right > 7):
        # Then robot gets back
        leftMotor.setPosition(0)
        rightMotor.setPosition(0)
        leftMotor.setVelocity(0.1*MAX_SPEED)
        rightMotor.setVelocity(0.8*MAX_SPEED)
        robot.step(1000)

    leftMotor.setPosition(float('inf'))
    rightMotor.setPosition(float('inf'))
