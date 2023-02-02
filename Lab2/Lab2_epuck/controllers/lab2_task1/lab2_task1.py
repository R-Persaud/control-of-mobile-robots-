"""lab2_task1 controller."""
# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Motor
from controller import DistanceSensor

import numpy as np
import math


def forwardKinematics(P, VL, VR, T):

    # Main loop:
    # perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        # Read the sensors:
        # Enter here functions to read sensor data, like:
        
        # print simulation time in sec (timesteps are in msec)
        print("Time: " +str(robot.getTime()))
        
        # Process sensor data here.   
        print("Left position sensor: " +str(leftposition_sensor.getValue()))
        print("Right position sensor: " +str(rightposition_sensor.getValue()))
    
        # Process sensor data here.
        print("IMU: "+str(imu.getRollPitchYaw()[2]))
    
        # Enter here functions to send actuator commands, like:
        #leftMotor.setPosition(P[2])
        #rightMotor.setPosition(P[2])
        
        if (VL > 6.28 or VR > 6.28):
            print("Maximum velocity exceeded")
            break
        
        if (leg == 0):
            #CIRLCE 1
            print("Circle 1")
            Circle(P, VL, VR)
            break
        elif (leg ==1):
            print("Distance D1")
            #move D1
            move_straight(P,VL,D1)
            break
        elif (leg ==2):
            #move Circle 2
            Circle2(P,VL,VR)
            break
        elif (leg ==3):
            #move D2
            print("Distance D2")
            move_straight(P,VL, D2)
            print("Total Navigation Time:", robot.getTime())
            break
        
        
    return 0 
    

def Circle (Position, VelL, VelR):

    while robot.step(timestep) != -1:
        #CIRLCE 1
        leftMotor.setVelocity(VelL)
        rightMotor.setVelocity(VelR)
        angle = imu.getRollPitchYaw()[2]
      
        if ( angle <= Position[2]): 
            print("here")
            rightMotor.setVelocity(0)
            leftMotor.setVelocity(0)
            break  
    return

def Circle2 (Position, VelL, VelR):

    while robot.step(timestep) != -1:
        #CIRLCE 2
        leftMotor.setVelocity(VelL)
        rightMotor.setVelocity(VelR)
        angle = imu.getRollPitchYaw()[2]
      
        if (angle >= Position[2]): 
            print("here")
            rightMotor.setVelocity(0)
            leftMotor.setVelocity(0)
            break  
    return



def move_straight(Position,Vel, Distance):
    start_position = leftposition_sensor.getValue()
    start_time = robot.getTime()

    leftMotor.setVelocity(Vel)
    rightMotor.setVelocity(Vel)
    
    while robot.step(timestep) != -1:
         if WheelRadius*abs(leftposition_sensor.getValue() - start_position) >= Distance-0.01:
            print("stopping")
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break
    return

# create the Robot instance.
robot = Robot()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
    
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#motor = robot.getDevice('e-puck')
ds = robot.getDevice('front distance sensor')
ds.enable(timestep)
    
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)
    
#getting the position sensors
    
leftposition_sensor = robot.getDevice('left wheel sensor')
rightposition_sensor = robot.getDevice('right wheel sensor')
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)
    
imu = robot.getDevice('inertial unit')
imu.enable(timestep)
   
#constants 
D = 2.28
Dmid = 2.28/2
WheelDiameter = 1.6
WheelRadius = 1.6/2

#intial position
C = (0,0,0)
   
#given ANGULAR VELOCITY and TIME
phi_L= (5, 4, 5, 4) #left motor angular velocity 
phi_R = (3.143, 4, 3.143, 4) # right motor angular velocity 
T = (1.929, 3, 3.857, 3)
   
   #kinematics 
#lINEAR VELOCITIES 
VL = [0,0,0,0] #left wheel Linear velocities
VR = [0,0,0,0] #Right wheel linear velocities

#setting linear velocities:   
for i in range(0,4):
    VL[i] = phi_L[i]*(WheelRadius) # v = Phi*r
    VR[i] = phi_R[i]*(WheelRadius) 


#calculating R1    
R1 = Dmid*((VL[0]+ VR[0])/(VL[0]-VR[0]))
R1 = abs(R1)
print("R1: ", R1)

    
#Computing P1 x, y, theta
V1avg = (VL[0] + VR[0])/2 #average velocity 
s1 = V1avg * T[0] #distance from P0 to P1 <- problem
omega1 = (VL[0]-VR[0])/D
theta1 = omega1* T[0]
#theta1 = s1/R1
x1 = R1*(1- math.cos(theta1))
y1 = R1*(math.sin(theta1))
    
P1 = (x1, y1, theta1) 
print("Position 1: ", P1)

#Computing P2 x, y, theta
theta2 = 0 #theta1 # no change since it is a straight line 
V2avg = (VL[1] + VR[1])/2
D1 = V2avg*T[1] #distance from P1 to P2
print("D1: ", D1)
x2 = D1 
y2 = 0 #y isn't changing since it moves parallel to the x-axis

P2 = (x2, y2, theta2)
print("Position 2: ", P2)
 

#calculating R2
R2 = Dmid*((VL[2]+ VR[2])/(VL[2]-VR[2]))
R2 = abs(R2)   
print("r2: ", R2)
 
#Computing P3 x, y, theta
V3avg = (VL[2]+VR[2])/2 
s3 = V3avg * T[2] #distance from P2 to P3
omega2 = (VL[2] - VR[2])/D
theta3 = omega2* T[2]
#theta3 = s3/R2 #problem
x3 = R2*(1-math.cos(theta3))
y3 = R2*(math.sin(theta3))

P3 = (x3,y3,theta3)
print("Position 3: ", P3)

#Computing P4 x, y, theta 
theta4 = theta3
V4avg = (VL[3]+ VR[3])/2 
D2 = V4avg*T[3]
print("D2:", D2)
x4 = R2
y4 = D2

P4 = (x4, y4, theta4)
print("Position 4: ", P4)


leg = 0 
#call with inital position C: phi_l = 5, phi_r = 3.143. T = 1.929
forwardKinematics(C, phi_L[0], phi_R[0], T[0])
leg += 1
#call with P1: phi_l = 4, phi_r = 4, T = 3
forwardKinematics(P1, phi_L[1], phi_R[1], T[1])
leg +=1
#call with P2: phi_l = 5, phi_r = 3.143, T = 3.957
forwardKinematics(P3, phi_L[2], phi_R[2],T[2])
leg+=1 
#call with P1: phi_l = 4, phi_r = 4, T = 3
forwardKinematics(P4, phi_L[3], phi_R[3], T[3])
leg+=1



