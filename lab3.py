"""lab3 controller."""
# Copyright University of Colorado Boulder 2022
# CSCI 3302 "Introduction to Robotics" Lab 3 Base Code.

from controller import Robot, Motor
import math

# TODO: Fill out with correct values from Robot Spec Sheet (or inspect PROTO definition for the robot)
MAX_SPEED = 6.67 # [rad/s] #Max Rotational Velocity
MAX_SPEED_MS = 0.22 # [m/s] #Max Translational velocity
AXLE_LENGTH = 0.16 # [m] 



MOTOR_LEFT = 0 # Left wheel index
MOTOR_RIGHT = 1 # Right wheel index

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# The Turtlebot robot has two motors
part_names = ("left wheel motor", "right wheel motor")


# Set wheels to velocity control by setting target position to 'inf'
# You should not use target_pos for storing waypoints. Leave it unmodified and 
# use your own variable to store waypoints leading up to the goal
target_pos = ('inf', 'inf') 
robot_parts = []

for i in range(len(part_names)):
        robot_parts.append(robot.getDevice(part_names[i]))
        robot_parts[i].setPosition(float(target_pos[i]))

# Odometry
pose_x     = -8
pose_y     = -5
pose_theta = 0

# Rotational Motor Velocity [rad/s]
vL = 0
vR = 0

# TODO
# Create you state and goals (waypoints) variable here
# You have to MANUALLY figure out the waypoints, one sample is provided for you in the instructions

#r = MAX_SPEED_MS / MAX_SPEED
#phiL = ((MAX_SPEED_MS - pose_theta*(AXLE_LENGTH)/2)/2)
#phiR = ((MAX_SPEED_MS + pose_theta*(AXLE_LENGTH)/2)/2)
    
#print(phiL)
#print("This is %f"%phiR)

p1 = 1.5
p2 = 1.5
p3 = 1.5





while robot.step(timestep) != -1:

    # STEP 2.1: Calculate error with respect to current and goal position
    xg = -7.866 #goal position of the x-cor yellow ball
    yg = -4.09 #goal position of the y-cor yellow ball
    theta_g = 0
    rho = math.sqrt((pose_x-xg)**2 +(pose_y-yg)**2) #distance
    alpha = (math.atan(yg-pose_y/xg-pose_x)) - pose_theta #bearing
    heading = theta_g - pose_theta #heading
    pass   
    
    # STEP 2.2: Feedback Controller
    xDot=p1*rho
    thetaDot = p2*alpha + p3*heading
    r = MAX_SPEED_MS / MAX_SPEED
    pass
    
    
    
    
    
    # STEP 1: Inverse Kinematics Equations (vL and vR as a function dX and dTheta)
    # Note that vL and vR in code is phi_l and phi_r on the slides/lecture
    phiL = ((xDot - thetaDot*(AXLE_LENGTH)/2)/r)
    phiR = ((xDot + thetaDot*(AXLE_LENGTH)/2)/r)
    pass
    
    # STEP 2.3: Proportional velocities
    
    vL = phiL # Left wheel velocity in rad/s
    vR = phiR # Right wheel velocity in rad/s
    
    if(phiL > MAX_SPEED):
       vL = MAX_SPEED
       vR = MAX_SPEED / 2 
    if(phiR > MAX_SPEED):
        vR = MAX_SPEED
        vL = MAX_SPEED / 2
    if(phiL > MAX_SPEED and phiR > MAX_SPEED):
        vR = MAX_SPEED
        vL = MAX_SPEED
    pass

    # STEP 2.4: Clamp wheel speeds
    pass
    
    if(rho < (10)**(-1)):
        vL = 0
        vR = 0
    


    
    # TODO\
    distL = vL/MAX_SPEED * MAX_SPEED_MS * timestep/1000.0

    distR = vR/MAX_SPEED * MAX_SPEED_MS * timestep/1000.0

    pose_x += (distL+distR) / 2.0 * math.cos(pose_theta)

    pose_y += (distL+distR) / 2.0 * math.sin(pose_theta)

    pose_theta += (distR-distL)/AXLE_LENGTH
    # Use Your Lab 2 Odometry code after these 2 comments. We will supply you with our code next week 
    # after the Lab 2 deadline but you free to use your own code if you are sure about its correctness
    
    # NOTE that the odometry should ONLY be a function of 
    # (vL, vR, MAX_SPEED, MAX_SPEED_MS, timestep, AXLE_LENGTH, pose_x, pose_y, pose_theta)
    # Odometry code. Don't change speeds (vL and vR) after this line
    
    
    

    ########## End Odometry Code ##################
    
    ########## Do not change ######################
    # Bound pose_theta between [-pi, 2pi+pi/2]
    # Important to not allow big fluctuations between timesteps (e.g., going from -pi to pi)
    if pose_theta > 6.28+3.14/2: pose_theta -= 6.28
    if pose_theta < -3.14: pose_theta += 6.28
    ###############################################

    # TODO
    # Set robot motors to the desired velocities
    robot_parts[MOTOR_LEFT].setVelocity(vL)
    robot_parts[MOTOR_RIGHT].setVelocity(vR)
    
    #max_velocity = robot_parts[MOTOR_LEFT].getMaxVelocity()
    #print(thetaDot)
    #print(rho)
    #print(pose_theta)
    print(rho)
    #print(xg)
    #print(alpha)
    #print(heading)

    