
from controller import Robot
# import numpy as it may be used in future labs
import numpy as np



def rectangleMotion(W,H,V):
      
    robot = Robot()

    timestep = int(robot.getBasicTimeStep())
    

    leftMotor = robot.getDevice('left wheel motor')
    rightMotor = robot.getDevice('right wheel motor')
   
    leftMotor.setPosition(float('inf'))
    rightMotor.setPosition(float('inf'))

    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)

    leftposition_sensor = robot.getDevice('left wheel sensor')
    rightposition_sensor = robot.getDevice('right wheel sensor')
    leftposition_sensor.enable(timestep)
    rightposition_sensor.enable(timestep)
    #if greater than max velocity
    if V>5: 
        print("Cannot be completed")
        return 1
    #constants 
    RADIUS = 0.04064/2 # 1.6"/ 2 (in meters) - radius of wheel
    D = 0.057912 #2.28" = distance between wheels
    W = W/39.37
    H = H/39.37
    V  = V/39.37 #average velocity in m/s 
    v_angular = (V/2)/RADIUS # phi = v/r - angular velocity of each wheel
    omega = (2*V)/D # global angular rotation of robot for turning = v_l -(-v_r)/d  [negative vr to turn right]
    angle = 1.5708 # pi/2 =  angle of rotation

    #Time to travel distances (move straight)
    Tw2 = (W/2)/V
    TW = W/V
    TH = H/V

    #Time to turn
    T_rotation = angle/ omega  #time for rotation of robot
   
    #start time of simulation
    t_initial = robot.getTime()

    #time range for turn 1
    T1s = t_initial + Tw2 # starts turn1 after travelling for Tw2
    T1e = T1s + T_rotation #stops turning after T_rotation 

    #time range for turn 2: 
    T2s = T1e + TH #second turn starts after travelling distance TH
    T2e = T2s + T_rotation #second turn ends after rotation time 

    #time range for turn 3: 
    T3s = T2e + TW #third turn starts after travelling distance TW
    T3e = T3s + T_rotation #second turn ends after rotation time 
   
    #time range for turn 3: 
    T3s = T2e + TW #third turn starts after travelling distance TW
    T3e = T3s + T_rotation #second turn ends after rotation time 

    #time range for turn 4: 
    T4s = T3e + TH #fourth turn starts after travelling distance TH
    T4e = T4s + T_rotation #fourth turn ends after rotation time 
    
    while robot.step(timestep) != -1:
        #move W/2
        now = robot.getTime()
        print(now, ": Moving straight")  
        leftMotor.setVelocity(v_angular)
        rightMotor.setVelocity(v_angular)
        
        #turn 1
        if T1s < now < T1e:
            leftMotor.setVelocity(v_angular)
            rightMotor.setVelocity(-v_angular)
            print(now, ": Turn 1")

        
        #turn 2 
        elif T2s < now < T2e: 
           leftMotor.setVelocity(v_angular)
           rightMotor.setVelocity(-v_angular)
           print(now,": Turn 2")
       
  
       #turn 3
        elif T3s < now < T3e:
           leftMotor.setVelocity(v_angular)
           rightMotor.setVelocity(-v_angular)
           print(now,": Turn 3") 

        #turn 4
        elif T4s < now < T4e:
           leftMotor.setVelocity(v_angular)
           rightMotor.setVelocity(-v_angular)
           print(now, ": Turn 4 ") 
             
        elif now > T4e + Tw2:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)  
            print ("Total navigation time:", now)
            break
rectangleMotion(10,15,10)
