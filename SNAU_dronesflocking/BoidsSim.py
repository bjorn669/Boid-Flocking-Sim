# -*- coding: utf-8 -*-
"""
Created on Fri Dec  1 19:13:16 2023

@author: strei
"""


import pygame as pg 
import random as rd 
import perlin_noise as pn 
from pygame import Vector2
import numpy as np
 

""" !!! INSTRUCTIONS !!!

- Press M to launch the sim 

- Press L before M at the beginning to set random intitial velocity vectors
  (L can be pressed at any time to reset the velocities and scatter the boids) 
  
- Press R to start recording the positions of each boids 

- Hit the SPACE BAR to stop the simulation and stop the recording 


"""


 
#%% This is the boid class on which the whole program is based

class Boid: 
    def __init__(self, ID=0, xPos=450, yPos=450, size = (20,20), angle=0):
        
        # pose params
        self.ID = ID
        self.xPos = xPos
        self.yPos = yPos
        self.boidAngle = angle
        
        # display params
        self.boidSize = size
        
        # displacement vectors init 
        self.boidPos = Vector2(xPos,yPos)
        self.boidPosPrev = self.boidPos.copy()
        self.boidVel = Vector2(0,0)
        # uncomment next line if you want to lanch the boids at the beginning of sim
        #self.boidVel = Vector2(round(rd.uniform(-20,20)),round(rd.uniform(-20,20)))   
        
        # noise displacement offset params
        self.xoff = rd.uniform(100,10000)
        self.yoff = rd.uniform(100,10000)
        self.noise_Xincrement = round(np.random.uniform(low=0.001, high=0.01),3)
        self.noise_Yincrement = round(np.random.uniform(low=0.001, high=0.01),3)
        
    def loadImg(self,imgFile):
        
        self.boid_img = pg.image.load(imgFile).convert_alpha()
        self.boid_img = pg.transform.scale(self.boid_img, self.boidSize)
        self.boid_rect = self.boid_img.get_rect(center = self.boidPos)
    
    def initCopy(self):
        self.boid_img_copy = self.boid_img.copy()
        
#%% Functions bank 


def rot_image(img,angle,imgRect):
    
    rotated_img = pg.transform.rotate(img, angle)
    imgRect = rotated_img.get_rect(center=imgRect.center)
    
    return rotated_img, imgRect
    
def CreateBoids(number,preset,presetArr = []):
    MyBoids = []
    for objnum in range(1,number+1):
        MyBoids.append("boid"+str(objnum))
    
    for objnum in range(len(MyBoids)):
        if preset == True:
            if len(presetArr)>0:
                MyBoids[objnum] = Boid(ID = objnum, xPos= presetArr[objnum][0], yPos= presetArr[objnum][1])
            else: 
                raise Exception('you forgot to pass the array of predefined positions !')
        else: 
            MyBoids[objnum] = Boid(ID = objnum, xPos= rd.uniform(1,WIDTH), yPos= rd.uniform(1,HEIGHT))
        MyBoids[objnum].loadImg("myboid.png")
        MyBoids[objnum].initCopy()
        
    return MyBoids

def edges(boid):
    if boid.boidPos.x > WIDTH: 
        boid.boidPos.x = 0
        #boid.boidPos.y = rd.uniform(0,HEIGHT)
    elif boid.boidPos.y > HEIGHT:
        boid.boidPos.y = 0
        #boid.boidPos.x = rd.uniform(0,WIDTH)
    elif boid.boidPos.x < 0:
        boid.boidPos.x = WIDTH
        #boid.boidPos.y = rd.uniform(0,HEIGHT)
    elif boid.boidPos.y < 0: 
        boid.boidPos.y = HEIGHT
        #boid.boidPos.x = rd.uniform(0,WIDTH)
    else: 
        pass

def euclidianDist(boid1,boid2):
    x1 = boid1.boidPos.x
    y1 = boid1.boidPos.y 
    x2 = boid2.boidPos.x 
    y2 = boid2.boidPos.y 
    
    dist = round(np.sqrt((x2-x1)**2 + (y2-y1)**2))

    return dist

def StopRecording(boidlist, stopradius, recordcond, stopcond):
    counter = 0
    if recordcond == True:
        for boidnum in range(len(boidlist)):
            dist2waypoint = round(np.sqrt((waypoint.x - boidlist[boidnum].boidPos.x)**2 + (waypoint.y - boidlist[boidnum].boidPos.y)**2))
            if dist2waypoint < stopradius:
                counter += 1
        if counter == len(boidlist):
            stopcond = True
    return stopcond 

#______________________________Behaviour main rules_______________________________

def Alignment(current_boidnum,boidlist,maxforce,vectMag):
    perceptionRadius = 100
    steering =  Vector2(0,0)
    total = 0
    
    for boidnum in range(len(boidlist)):
        if boidnum != current_boidnum:
            dist = euclidianDist(boidlist[current_boidnum], boidlist[boidnum])
            if dist < perceptionRadius: 
                steering += boidlist[boidnum].boidVel
                total += 1
    if total > 0:
        steering = steering / total 
        # resetting speed vector magnitude 
        steering.x += rd.uniform(-1,1) * 0.00001
        steering.y += rd.uniform(-1,1) * 0.00001
        steering.scale_to_length(vectMag)
        steering -= boidlist[current_boidnum].boidVel
        # limiting vector magnitude to max steering force 
        if steering.x > maxforce: 
            steering.x = maxforce
        if steering.x < -maxforce: 
            steering.x = -maxforce
        if steering.y > maxforce: 
            steering.y = maxforce
        if steering.y < -maxforce: 
            steering.y = -maxforce
            
    return steering 

def Cohesion(current_boidnum,boidlist,maxforce,vectMag):
    perceptionRadius = 100
    steering =  Vector2(0,0)
    total = 0
    
    for boidnum in range(len(boidlist)):
        if boidnum != current_boidnum:
            dist = euclidianDist(boidlist[current_boidnum], boidlist[boidnum])
            if dist < perceptionRadius: 
                steering += boidlist[boidnum].boidPos    # CHANGE FOR POS HERE 
                total += 1
    if total > 0:
        steering = steering / total 
        steering -= boidlist[current_boidnum].boidPos
        steering.x += rd.uniform(-1,1) * 0.00001
        steering.y += rd.uniform(-1,1) * 0.00001
        steering.scale_to_length(vectMag)
        steering -= boidlist[current_boidnum].boidVel
        
        # limiting vector magnitude to max steering force 
        if steering.x > maxforce: 
            steering.x = maxforce
        if steering.x < -maxforce: 
            steering.x = -maxforce
        if steering.y > maxforce: 
            steering.y = maxforce
        if steering.y < -maxforce: 
            steering.y = -maxforce
            
    return steering 

def Separation(current_boidnum,boidlist,maxforce,vectMag):
    perceptionRadius = 70
    steering =  Vector2(0,0)
    total = 0
    
    for boidnum in range(len(boidlist)):
        if boidnum != current_boidnum:
            dist = euclidianDist(boidlist[current_boidnum], boidlist[boidnum])
            if dist < perceptionRadius: 
                
                difference = Vector2(boidlist[boidnum].boidPos.x - boidlist[current_boidnum].boidPos.x , boidlist[boidnum].boidPos.y - boidlist[current_boidnum].boidPos.y)  
                # we want the difference to be inversely proportional to the distance 
                if dist != 0:
                    difference = difference * (1/(dist))    # squared or not ??
                else: 
                    difference = difference * (1/(0.00001))
                steering -= difference    
                total += 1
    if total > 0:
        steering = steering / total
        steering.x += rd.uniform(-1,1) * 0.00001
        steering.y += rd.uniform(-1,1) * 0.00001
        steering.scale_to_length(vectMag)
        steering -= boidlist[current_boidnum].boidVel
        
        # limiting vector magnitude to max steering force 
        if steering.x > maxforce: 
            steering.x = maxforce
        if steering.x < -maxforce: 
            steering.x = -maxforce
        if steering.y > maxforce: 
            steering.y = maxforce
        if steering.y < -maxforce: 
            steering.y = -maxforce
            
    return steering 

#______________________________Additional rules_______________________________

def Steer2Waypoint(current_boidnum,boidlist,waypoint,maxforce,vectMag):
    perceptionRadius = 1200
    steering =  Vector2(0,0)
    
    dist = round(np.sqrt((waypoint.x - boidlist[current_boidnum].boidPos.x)**2 + (waypoint.y - boidlist[current_boidnum].boidPos.y)**2))
    
    if dist < perceptionRadius: 
        steering += waypoint    # CHANGE FOR WAYPOINT HERE 
        
        steering -= boidlist[current_boidnum].boidPos
        steering.x += rd.uniform(-1,1) * 0.00001
        steering.y += rd.uniform(-1,1) * 0.00001
        steering.scale_to_length(vectMag)
        steering -= boidlist[current_boidnum].boidVel
        
        # limiting vector magnitude to max steering force 
        if steering.x > maxforce: 
            steering.x = maxforce
        if steering.x < -maxforce: 
            steering.x = -maxforce
        if steering.y > maxforce: 
            steering.y = maxforce
        if steering.y < -maxforce: 
            steering.y = -maxforce
            
    return steering 

def AvoidObstacle(current_boidnum,boidlist,obstacle,obstacleWidth,maxforce,vectMag):
    perceptionRadius = obstacleWidth + 120
    steering = Vector2(0,0)
    
    dist = dist = round(np.sqrt((obstacle.x - boidlist[current_boidnum].boidPos.x)**2 + (obstacle.y - boidlist[current_boidnum].boidPos.y)**2))
    if dist < perceptionRadius: 
        difference = Vector2(obstacle.x - boidlist[current_boidnum].boidPos.x - obstacleWidth , obstacle.y - boidlist[current_boidnum].boidPos.y - obstacleWidth)  
        # we want the difference to be inversely proportional to the distance 
        if dist != 0:
            difference = difference * (1/(dist**2))    # squared or not ??
        else: 
            difference = difference * (1/(0.00001**2))
        steering -= difference    
        steering.x += rd.uniform(-1,1) * 0.00001
        steering.y += rd.uniform(-1,1) * 0.00001
        steering.scale_to_length(vectMag)
        steering -= boidlist[current_boidnum].boidVel
        
        # limiting vector magnitude to max steering force 
        if steering.x > maxforce: 
            steering.x = maxforce
        if steering.x < -maxforce: 
            steering.x = -maxforce
        if steering.y > maxforce: 
            steering.y = maxforce
        if steering.y < -maxforce: 
            steering.y = -maxforce
          
    return steering 


# main process 
def main(FPS=30,num_boids = 1,natural_behaviour = True,ScreenMod = 'window',PredetPos = []):
    
    BG = (0,0,0)
    pg.init()
    global WIDTH
    global HEIGHT
    global simspeed
    global waypoint
    global obstacle
    global obstacleWidth
    global POSITIONX
    global POSITIONY
    
    if ScreenMod == 'window':
        WIDTH = 900
        HEIGHT = 900 
        win = pg.display.set_mode((WIDTH,HEIGHT))
    if ScreenMod == 'fullscreen':
        WIDTH = 1920
        HEIGHT = 1080 
        win = pg.display.set_mode((WIDTH,HEIGHT),pg.FULLSCREEN)
        
    pg.display.set_caption('2D Flocking Sim')
    pygame_icon = pg.image.load("myboid.png").convert_alpha()
    pg.display.set_icon(pygame_icon)

    # creating the required number of boids
    
    # we start by checking if the user wants predefined initial positions 
    if len(PredetPos) > 0:
        if len(PredetPos) != num_boids :
            raise Exception(f'wrong size for PredetPos list of tuples, should be of size {num_boids}')
        else: 
            MyBoids = CreateBoids(num_boids,preset=True,presetArr= PredetPos)
    else:        
        # standard way of creating the boids with a random initial position 
        MyBoids = CreateBoids(num_boids,preset=False)
     
        
    
    # Natural movement : Perlin Noise initial parameters
    liveness = 1
    randSeed = rd.uniform(1,600)
    noise = pn.PerlinNoise(octaves = liveness, seed = randSeed)

    accelFactor = 90
    simspeed = 1
    
    # Rules coefficient 
    alignmentFactor = 1
    cohesionFactor = 1
    separationFactor = 1
    waypointFactor = 3
    obstacleFactor = 5
    
    waypoint = Vector2(0,0)
    obstacle = Vector2(0,0)
    obstacleWidth = round(rd.uniform(20,60))
    
    POSITIONX = []
    POSITIONY = []
    
    for objnum in range(len(MyBoids)):
        win.blit(MyBoids[objnum].boid_img_copy, MyBoids[objnum].boidPos)
        POSITIONX.append([])
        POSITIONY.append([])
        
    print(POSITIONX)
    
    
    framerate = pg.time.Clock()
    running = True
    record = False
    stoprecord = False
    launch = False
    
    # Sim loop 
    while running:
        
        win.fill(BG)
        for event in pg.event.get():
            if event.type == pg.QUIT: 
                running = False
            if event.type == pg.KEYDOWN:
                if event.key == pg.K_SPACE:
                    running = False
                if event.key == pg.K_r:
                    record = True 
                    print('...RECORDING...',record)
                    
                # pressing L either launches or resets the speed of all the boids 
                if event.key == pg.K_l:
                    print('...RESETTING BOIDS VELOCITIES AND DIRECTIONS...')
                    for objnum in range(len(MyBoids)):
                        MyBoids[objnum].boidVel = Vector2(round(rd.uniform(-20,20)),round(rd.uniform(-20,20)))
                # pressing M HARDLAUNCHES the sim 
                if event.key == pg.K_m:
                    print('...LAUNCHING SIM...')
                    launch = True 
                
            if event.type == pg.MOUSEBUTTONDOWN:
                mousepress = pg.mouse.get_pressed()
                if mousepress[0]:
                    # Setting the waypoint to click position  
                    waypoint.x = pg.mouse.get_pos()[0]
                    waypoint.y = pg.mouse.get_pos()[1]
                    print(f"--- waypoint set at {waypoint} ---")
                elif mousepress[2]:
                    obstacle.x = pg.mouse.get_pos()[0]
                    obstacle.y = pg.mouse.get_pos()[1]
                    obstacleWidth = round(rd.uniform(20,100)) # setting a random new size of obstacle at each click 
                    print(f"--- obstacle set at {obstacle} ---")
                    
            
        for objnum in range(len(MyBoids)):
            MyBoids[objnum].boid_rect.centerx = MyBoids[objnum].boidPos.x
            MyBoids[objnum].boid_rect.centery = MyBoids[objnum].boidPos.y
            
            # rotating the boid image to correct direction 
            v1rot = MyBoids[objnum].boidVel.copy()
            v2rot = Vector2(0,-1)
            MyBoids[objnum].boidAngle = v1rot.angle_to(v2rot)
            
            MyBoids[objnum].boid_img, MyBoids[objnum].boid_rect = rot_image(MyBoids[objnum].boid_img_copy,MyBoids[objnum].boidAngle,MyBoids[objnum].boid_rect)
            
            # Updating the speed vector with perlin noise values for natural movement
            if natural_behaviour == True:
                MyBoids[objnum].boidVel.x = noise(MyBoids[objnum].xoff)*accelFactor
                MyBoids[objnum].boidVel.y = noise(MyBoids[objnum].yoff)*accelFactor 
            
            # borders 
            edges(MyBoids[objnum])
            
            if launch == True: 
                # Alignment rule  
                alignment = Alignment(objnum, MyBoids, maxforce=0.3, vectMag=15)
                # Cohesion rule
                cohesion = Cohesion(objnum, MyBoids, maxforce=0.3, vectMag=15)
                # Separation rule 
                separation = Separation(objnum, MyBoids, maxforce=0.8, vectMag=15)
                
                #Additional rules
                
                # Steer to waypoint objective 
                if waypoint != Vector2(0,0):
                    cohesion2waypoint = Steer2Waypoint(objnum, MyBoids, waypoint, maxforce=0.3, vectMag=15)
                else: 
                    cohesion2waypoint = Vector2(0,0)
                # Avoid the near by obstacle
                if obstacle != Vector2(0,0):
                    obstacle_avoidance = AvoidObstacle(objnum, MyBoids, obstacle, obstacleWidth, maxforce=0.4, vectMag=15)
                else: 
                    obstacle_avoidance = Vector2(0,0)
                
                #print(obstacle_avoidance)
                
                # Refreshing the speed vector with the contribution of all the rules  
                # !!!! comment here to deactivate the rule !!!!
                
                MyBoids[objnum].boidVel += alignment * alignmentFactor
                MyBoids[objnum].boidVel += cohesion * cohesionFactor  
                MyBoids[objnum].boidVel += separation * separationFactor
                MyBoids[objnum].boidVel += cohesion2waypoint * waypointFactor
                MyBoids[objnum].boidVel += obstacle_avoidance * obstacleFactor
                
                MyBoids[objnum].boidVel = MyBoids[objnum].boidVel * simspeed 
                # Updating the position vector with the speed vector 
                MyBoids[objnum].boidPos += MyBoids[objnum].boidVel
                
            # Updating the record lists 
            if record == True:
                POSITIONX[objnum].append(MyBoids[objnum].boidPos.x)
                POSITIONY[objnum].append(MyBoids[objnum].boidPos.y)
            # Stopping the recording when the boids reached the perimeter of the waypoint  
            
            if StopRecording(MyBoids,50,record,stoprecord):
                record = False
                running = False
                print('...END OF RECORDING... ',record)
            
            # Storing previous position (optional)
            MyBoids[objnum].boidPosPrev = MyBoids[objnum].boidPos.copy()
            #print("boid pos : ",Boid1.boidPos.xy)
            #print("boid vel : ",Boid1.boidVel.xy)
            
            if natural_behaviour == True: 
                MyBoids[objnum].xoff += MyBoids[objnum].noise_Xincrement
                MyBoids[objnum].yoff += MyBoids[objnum].noise_Yincrement
            
            # displaying the waypoint 
            if waypoint != [0,0]:
                pg.draw.circle(win, (0,0,255), (waypoint.x,waypoint.y), 10)
            # displaying the obstacle 
            if obstacle != [0,0]:
                pg.draw.circle(win, (0,255,0), (obstacle.x,obstacle.y), obstacleWidth, width = 3)
            # displaying the boid 
            win.blit(MyBoids[objnum].boid_img,MyBoids[objnum].boid_rect) #drawing each boid 
            
        
        pg.display.flip()
        framerate.tick(FPS)
        
        
    pg.quit()     
    return 0 
    

main(num_boids = 60, natural_behaviour = False, ScreenMod='fullscreen', PredetPos = [])    #(50,500),(50,400) my preset 



#%% Plotting trajectory record 

import matplotlib.pyplot as plt 

grey = '#BDC3C7'
handles = []
labels = []

fig = plt.figure(1,figsize=(10,10))
axes = fig.add_subplot(111)
boidtot = 0
if len(POSITIONX[0]) > 0:
    
    for objnum in range (len(POSITIONX)):
        color = np.random.rand(1)
        xdata = POSITIONX[objnum]
        ydata = POSITIONY[objnum]
        plot_boid = axes.scatter(xdata,ydata,
                    facecolor = plt.cm.hsv(color),
                    linewidths = 1,
                    marker ="^", 
                    edgecolor ="black", 
                    s = 50,
                    alpha= 0.7)
        
        boidname = 'drone n°'+str(objnum)
        boidtot = objnum +1
        handles.append(plot_boid)
        labels.append(boidname)


plot_waypoint = axes.scatter(waypoint.x,waypoint.y,
                c ="blue",
                linewidths = 1,
                marker =".", 
                edgecolor ="black", 
                s = 600,
                alpha= 1)

labels.append('waypoint')
handles.append(plot_waypoint)

plot_obstacle = plt.Circle((obstacle.x, obstacle.y), obstacleWidth, fill=False, edgecolor = 'green', linewidth = 4, alpha = 1)
axes.add_artist(plot_obstacle,)

labels.append('obstacle')
handles.append(plot_obstacle)

axes.set_xlim(0,WIDTH)
axes.set_ylim(0,HEIGHT)
axes.set_aspect('equal')
axes.invert_yaxis()
axes.set_facecolor(grey)
plt.xlabel('x', font = "Cambria", fontweight = 'bold', size = 16)
plt.ylabel('y', font = "Cambria", fontweight = 'bold', size = 16)
plt.grid(True,color='k', linestyle='--', linewidth=1)

plt.yticks(font = 'Cambria',fontsize = 8, fontweight = 'heavy')
plt.xticks(font = 'Cambria',fontsize = 10, fontweight = 'heavy')

axes.legend(handles, labels,loc="upper right")
plt.title(f"2D Path Tracking -- trajectory of {boidtot} tello drones",font= 'Cambria', fontweight = 'bold', size = 16)

plt.show()