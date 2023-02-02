"""lab1_task2 controller."""


from controller import Robot
import numpy as np

def circlesMotion(R1, R2, V):

    if R1 ==0 or R2 ==0:
        print("Division by 0 error")
        return
    R1 = R1/ 39.37
    R2 = R2/39.37
    # create the Robot instance.
    robot = Robot()
    
    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())

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
    
   

    w_radius = 0.04064/2 # 1.6"/ 2 (in meters) - radius of wheel
    D = 0.057912 #2.28" = distance between wheels
    D_mid = D/2
    V  = V/39.37 #average velocity in m/s 
    angle = 6.28 #2pi = 360

    #global angular velocities
    omega1 = V/R1
    omega2 = V/R2

    #global turning time 
    T1 = angle/ omega1
    T2 = angle/ omega2

    #local  angular velocities:
    #circle 1:
    phi_l1 = (omega1*(R1-D_mid))/w_radius
    phi_r1 = (omega1*(R1+D_mid))/w_radius
    #circle 2:    
    phi_l2 = (omega2*(R2+D_mid))/w_radius
    phi_r2 = (omega2*(R2-D_mid))/w_radius
    
    #error handling
    if phi_l1 >6.28 or phi_l2 > 6.28 or phi_r2 > 6.28 or phi_r1 > 6.28:
        print ("Cannot be completed: max velocity reached")
        return
    
    
    t_initial = robot.getTime()
    
    #turn robot around
    t_180 = 3.14/omega1
    #Rotiation ranges
    T1s = t_initial 
    T1e = t_initial + T1
    
    T2s = T1e
    T2e = T2s + T2 
   


    # Main loop:
    # perform simulation steps until Webots is stopping the controller

    while robot.step(timestep) != -1:

        
        now = robot.getTime()

        leftMotor.setVelocity(-phi_l1)
        rightMotor.setVelocity(-phi_r1)

        if T1e + T1/2 < now < T2e + T2/4:
            
            leftMotor.setVelocity(phi_l2)
            rightMotor.setVelocity(phi_r2)
            
        elif now > T2e + T2/4:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)    
            print("Total Navigation time: " , now)
            break

circlesMotion(5, 10, 4)