"""Sample Webots controller for the wall following benchmark."""

from controller import Robot
from math import sqrt

def clamp(num, min_value, max_value):
   return max(min(num, max_value), min_value)

def getDistance(sensor):
    return clamp(((1000 - abs(sensor.getValue())) / 100), 0.0, 5.0)

def PIDctl(prev):
    up = kp*prev[4]
    ud = (prev[4] - prev[0]) * kd
    ui = sum(prev)

    if (ui > max_ui):
        ui = max_ui
    elif (ui < min_ui):
        ui = min_ui
    ui *= ki

    # Debug

    print(">> ", prev[4])
    print(">> %3.3f | %3.3f | %3.3f = %3.3f" % (up, ud, ui, up + ud + ui))

    return up + ud + ui

# Maximum speed for the velocity value of the wheels.
# Don't change this value.
MAX_SPEED = 5.24

prev_left = [0] * 5
prev_left_center = [0] * 5
prev_center = [0] * 5

kp = 1
kd = 0.5
ki = 0.2
min_ui = -0.2
max_ui = 0.2


robot = Robot()

# Get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Get pointer to the robot wheels motors.
leftWheel = robot.getDevice('left wheel')
rightWheel = robot.getDevice('right wheel')

# We will use the velocity parameter of the wheels, so we need to
# set the target position to infinity.
leftWheel.setPosition(float('inf'))
rightWheel.setPosition(float('inf'))

# Get and enable the distance sensors.
front_left_sensor = robot.getDevice("so0")
back_left_sensor = robot.getDevice("so15")
front_sensor_1 = robot.getDevice("so1")
front_sensor_2 = robot.getDevice("so2")
front_center_sensor = robot.getDevice("so3")
front_sensor_4 = robot.getDevice("so4")
front_sensor_5 = robot.getDevice("so5")

front_left_sensor.enable(timestep)
back_left_sensor.enable(timestep)
front_sensor_1.enable(timestep)
front_sensor_2.enable(timestep)
front_center_sensor.enable(timestep)
front_sensor_4.enable(timestep)
front_sensor_5.enable(timestep)

# Move forward until we are 50 cm away from the wall.
leftWheel.setVelocity(MAX_SPEED)
rightWheel.setVelocity(MAX_SPEED)
while robot.step(timestep) != -1:
    if getDistance(front_center_sensor) < 0.25:
        break

# Rotate clockwise until the wall is to our left.
leftWheel.setVelocity(MAX_SPEED)
rightWheel.setVelocity(-MAX_SPEED)
while robot.step(timestep) != -1:
    # Rotate until there is a wall to our left, and nothing in front of us.
    if getDistance(back_left_sensor) < 0.26:
        break

leftWheel.setVelocity(MAX_SPEED)
rightWheel.setVelocity(MAX_SPEED)
# Main loop.
while robot.step(timestep) != -1:
	# Read values from sensors
	front_left_sensor_value = getDistance(front_left_sensor) # 0
	back_left_sensor_value = getDistance(back_left_sensor)
	front_sensor_1_value = getDistance(front_sensor_1)
	front_sensor_2_value = getDistance(front_sensor_2)
	front_center_sensor_value = getDistance(front_center_sensor) # 3
	front_sensor_4_value = getDistance(front_sensor_4)
	front_sensor_5_value = getDistance(front_sensor_5)
	
	print("front: ", front_left_sensor_value, ", back: ", back_left_sensor_value, ", center: ", front_center_sensor_value)
	print("sensor 1: ", front_sensor_1_value, ", sensor 2: ", front_sensor_2_value)
	
	left_wheel_speed = 0.8
	right_wheel_speed = 1
	
	# Если впереди что-то
	if min(front_center_sensor_value, front_sensor_2_value, front_sensor_1_value) < 0.6:
		# if (1 - min(front_center_sensor_value, front_sensor_2_value, front_sensor_1_value) / 0.6) < (min(front_center_sensor_value, front_sensor_2_value, front_sensor_1_value) / 0.6):
			# left_wheel_speed = min(front_center_sensor_value, front_sensor_2_value, front_sensor_1_value) / 0.6
		# else:
			# left_wheel_speed = 1
		# if min(front_center_sensor_value, front_sensor_2_value, front_sensor_1_value) <= 0.01:
			# right_wheel_speed = 0.01
		# else:
			# right_wheel_speed = (1 - min(front_center_sensor_value, front_sensor_2_value, front_sensor_1_value) / 0.6)
		left_wheel_speed = 1
		right_wheel_speed = 0.35
		print("going idk")
	
	# Клонимся влево
	elif front_left_sensor_value < 0.6 and back_left_sensor_value > front_left_sensor_value:
		left_wheel_speed = 1
		if front_left_sensor_value <= 0.01:
			right_wheel_speed = 0.1
		else:
			right_wheel_speed = 1 - front_left_sensor_value / 0.6 
		print("need to go right")
	
	# Надо повернуть влево
	elif back_left_sensor_value < 0.6 and front_left_sensor_value > 1:
		if back_left_sensor_value <= 0.01:
			left_wheel_speed = 0.1
		else:
			left_wheel_speed = 1 - back_left_sensor_value / 0.6
		right_wheel_speed = 1
		print("need to go left")
		
	# Если уходим от стены
	elif min(front_left_sensor_value, back_left_sensor_value, front_sensor_1_value) > 0.6:
		left_wheel_speed = 	1 - (min(front_left_sensor_value, back_left_sensor_value, front_sensor_1_value) / 5)
		right_wheel_speed = 1
		print("going right")
	
		
	if 0.5 < front_center_sensor_value < 4.5:
        #if (0.0 < getDistance(front_left_sensor) < 0.20 or 0.30 < getDistance(front_left_sensor) < 1):
		if (0.0 < front_left_sensor_value < 1):
			expectedDistanceToWall = 0.5
			realDistance = front_left_sensor_value
			error = (expectedDistanceToWall - realDistance)

			print("Left - ", realDistance)

			prev_left.pop(0)
			prev_left.append(error)
			pid = PIDctl(prev_left)

			if (pid < 0):
				left_wheel_speed = (1 - abs(pid))
				right_wheel_speed = 1
			else:
				left_wheel_speed = 1
				right_wheel_speed = (1 - abs(pid))
				
	# if front_center_sensor_value > 4.5:
		# left_wheel_speed = 1
		# right_wheel_speed = 0.2

	print("left speed: ", left_wheel_speed, "right speed: ", right_wheel_speed)
	leftWheel.setVelocity(clamp(left_wheel_speed * MAX_SPEED, 0.0, 4.192)) # MAX_SPEED * 0.8 = 4.192
	rightWheel.setVelocity(clamp(right_wheel_speed * MAX_SPEED, 0.0, MAX_SPEED))

# Stop the robot when we are done.
leftWheel.setVelocity(0)
rightWheel.setVelocity(0)
