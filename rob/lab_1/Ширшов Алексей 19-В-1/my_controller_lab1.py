from controller import Robot
MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
# get motors
left_motor = robot.getDevice('left wheel')
right_motor = robot.getDevice('right wheel')
# set the distance the wheels will pass
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
# set speed
left_motor.setVelocity(0.8 * MAX_SPEED)
right_motor.setVelocity(1 * MAX_SPEED)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    pass