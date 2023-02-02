"""lab2_task1 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Motor
from controller import DistanceSensor

import numpy as np
import math

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

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


#lists 
VL = [0,0,0,0]
VR = [0,0,0,0]
T = [0,0,0,0]

#constants 
D = 2.28
Dmid = 2.28/2
WheelDiameter = 1.6
WheelRadius = 1.6/2


#intial position
C = (0,0,0)
#final position = P4
F = (5, -5, -math.pi)
print ("Position 4 (x,y, theta): ", F)

Velocity = 3 # avg linear velocity - constant

#Linear Velocities:
#moving d2
VL[3] = Velocity
VR[3] = Velocity

#circle2
totalV = 2*Velocity #6
VR[2] = 2 #right wheel less to turn right
VL[2] = totalV - 2 #4

#moving d1
VL[1] = Velocity
VR[1] = Velocity

#cirlce 1
VL[0] = VL[2]
VR[0] = VR[2]

#calulating P3 - 
omega2 = (VL[2]-VR[2])/D
R2 = Dmid*((VR[2]+VL[2])/(VL[2]-VR[2]))
R2 = abs(R2)
R1 = R2
theta2 = F[2]


P3 = [0,0,0]
P3[0] = R1*(1-math.cos(theta2))
P3[1] = R1*(math.cos(theta2))
P3[2] = theta2

print ("Position 3 (x,y, theta): ", P3)
#D2 = difference in Ys of f AND P3  

D2 = math.sqrt((F[0]-P3[0])**2 + (F[1]-P3[1])**2) #F[1]-P3[1] 
#D1 = total x distance minus the 2 circles' radii
D1 = F[0]+R1 

#calculating P2
P2 = [0,0,0]
P2[0] = D1 + R1 #x value = half a cirle + disntace to circle 2
P2[1] = 0 #No y translation - moving horizontally
P2[2] = 0 #no rotation
print ("Position 2 (x,y, theta): ", P2)
 
#P1
omega1 = omega2
P1 = [0,0,0]
P1[0] = R1
P1[1] = R1
theta1 = math.pi
P1[2] = theta1
print ("Position 1 (x,y, theta): ", P1)

print("D1:", D1)
print("D2:", D2)
print("R1:", R1)
print("R2:", R2)


#angular velocities 
phiL = [0,0,0,0]
phiR = [0,0,0,0]
error = False
for i in range(0,4):
    phiL[i] = VL[i]/WheelRadius
    phiR[i] = VR[i]/WheelRadius
    if(phiL[i] >= 6.28 or phiR[i] >= 6.28):
        error = True
       
    #print("phiL, phiR :", phiL[i], phiR[i])
print("Left wheel angular velocities:", phiL)
print("Right wheel angular velocities:", phiR)

#Time
T[0] = theta1/omega1
T[1] = D1/Velocity
T[2] = theta2/ omega2
T[3] = D2/Velocity


def inverseKinematic(C, F, V):
    # Main loop:
    
   
    # perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        #error handling
        if (error == True): 
            print("Motion cannot be completed due to max velocity constraint")
            break
        
        # print simulation time in sec (timesteps are in msec)
        print("Time: " +str(robot.getTime()))
        
        # Process sensor data here.   
        print("Left position sensor: " +str(leftposition_sensor.getValue()))
        print("Right position sensor: " +str(rightposition_sensor.getValue()))
    
        # Process sensor data here.
        print("IMU: "+str(imu.getRollPitchYaw()[2]))
    
        # Enter here functions to send actuator commands, like:
        #  motor.setPosition(10.0)
        leg1 = Circle(C, phiL[0], phiR[0])
        if leg1 == True:
            leg2 = move_straight(phiL[1],D1)  
        if leg2== True: 
            leg3 = Circle2(F,phiL[2],phiR[2])
        if leg3==True:
            leg4 = move_straight(phiL[3],D2)
            print("Motion complete")
            break
        pass
    
def Circle (Position, VelL, VelR):
     
    print("Circle 1")
    while robot.step(timestep) != -1:
        #CIRLCE 1
        leftMotor.setVelocity(VelL)
        rightMotor.setVelocity(VelR)
        angle = imu.getRollPitchYaw()[2]
      
        if ( angle <= Position[2]): 
            print("End of Circle 1")
            rightMotor.setVelocity(0)
            leftMotor.setVelocity(0)
            break  
    return True
    
def Circle2 (Position, VelL, VelR):
    print("Circle 2")
    start = imu.getRollPitchYaw()[2]
    while robot.step(timestep) != -1:
        #CIRLCE 2
        leftMotor.setVelocity(VelL)
        rightMotor.setVelocity(VelR)
        
        angle = imu.getRollPitchYaw()[2]
        #print("IMU: "+str(imu.getRollPitchYaw()[2]))
        
        if (angle >= abs(F[2])-0.021108): 
            print("End of Circle 2")
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break  
    return True
    
def move_straight(Vel, Distance):
    start_position = leftposition_sensor.getValue()
    start_time = robot.getTime()

    leftMotor.setVelocity(Vel)
    rightMotor.setVelocity(Vel)
    
    while robot.step(timestep) != -1:
         if WheelRadius*abs(leftposition_sensor.getValue() - start_position) >= Distance-0.01:
            print("Distance moved: ", Distance)
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break
    return True
    
inverseKinematic(C, F, Velocity)