"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, DistanceSensor, Motor

TIME_STEP = 64

# create the Robot instance.
robot = Robot()
ps = []
psNames = [
    'ps-0', 'ps-1', 'ps-2', 'ps-3'
    'ps-4', 'ps-5', 'ps-6', 'ps-7'
    ]
    
    for i in range(8)
        ps.append(robot.getDevice(psNames[i])
        ps[i].enable(TIME_STEP)
        
    leftmotor = robot.getDevice('left-motor')
    rightmotor = robot.getDevice('roght-motor')
    leftmotor.setPosition(float('inf'))
    rightmotor.setPosition(float('inf'))
    leftmotor.setVelocity(0.0)
    rightmotor.setVelocity(0.0)
    
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    psValues = []
    for i in range(8):
        psvalues.append(ps[i].getValue())
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    
    #detect obstacles

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
