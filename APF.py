import matplotlib.pyplot as plt
import numpy as np
import random
import math
import copy


def Plot():
    global Start
    global Goal
    global Obstacles
    global ObstacleSize
    global Mapsize
    global Path
    figure, axes = plt.subplots()
    L = len(Obstacles)
    for i in range(L):
        Drawing_colored_circle = plt.Circle(( Obstacles[i][0] , Obstacles[i][1] ),ObstacleSize[i] ,color='r')
        axes.set_aspect( 1 )
        axes.add_artist( Drawing_colored_circle )

    
    plt.title( 'Map' )
    axes.set(xlim=(0, Mapsize[0]), ylim=(0, Mapsize[1]))
    path = np.array(Path)
    print(path)
    axes.plot(path[:, 0],path[:, 1])
    axes.plot(Start[0], Start[1],'o')
    axes.plot(Goal[0], Goal[1],'s',color='g')
    
    
    plt.show()

def DistancetoGoal():
    global Goal
    return DistancetoPoint(Goal)

def DistancetoPoint(point):
    global Current_pos
    dx= point[0]-Current_pos[0]
    dy= point[1]-Current_pos[1]
    return Mag([dx,dy])

def getAttractiveForce():
    global Current_pos
    global Goal
    global Katt
    global MaxAttractiveforce
    Fatt=[0,0]
    Fatt[0] = Katt*(Goal[0]-Current_pos[0])

    Fatt[1] = Katt*(Goal[1]-Current_pos[1])
    m=Mag(Fatt)
    if (m>MaxAttractiveforce):
        Fatt[0]=Fatt[0]*MaxAttractiveforce/m
        Fatt[1]=Fatt[1]*MaxAttractiveforce/m


    return Fatt

def bra7tak(Xobs,Yobs,qstar,Dobs):

    if Dobs < qstar :
        frep_x = -Krep * (1/Dobs - 1/qstar) * -(Current_pos[0] - Xobs)/((Dobs)**3)
        frep_y = -Krep * (1 / Dobs - 1 / qstar) * -(Current_pos[1] - Yobs) / ((Dobs)**3)
    else :
        frep_x = 0
        frep_y = 0

    return [frep_x,frep_y]

def getRepulsiveForce():
    global Obstacles
    global ObstacleSize
    global Rsafe
    FrTotal=[0,0]
    for i in range(len(Obstacles)):
        Obstacle = Obstacles[i]
        qstar = Rsafe
        xobs=Obstacle[0]
        yobs=Obstacle[1]
        dobs=DistancetoPoint(Obstacle)-ObstacleSize[i]
        Frep=   bra7tak(xobs,yobs,qstar,dobs)
        FrTotal[0]+=Frep[0]
        FrTotal[1]+=Frep[1]

    return FrTotal

def Mag(V):
    return math.sqrt(V[0]**2+V[1]**2)

def ObstacleMaker():
    return null 

Katt= 10
Krep=5
MaxAttractiveforce=20
NumberOfPoints=10
Start=[0,0]
Goal = [6,5]
mapclearness=0.1 # define how clearly the map is scanned
Obstacles=[ [1,1],[40,35],[65,70],[50,60]]# pos of each obstacle x y 
ObstacleSize=[0.1,6,7,3]# size of each var
Rsafe=100
Mapsize=[10,10] # x y
Path =[]
Current_pos=copy.deepcopy(Start)
Ftotal=[0,0]
Distance=DistancetoGoal()

while(Distance>mapclearness*10):
    Fa=getAttractiveForce()
    Fr=getRepulsiveForce()
    Ftotal[0] = Fa[0]+Fr[0]
    Ftotal[1] = Fa[1]+Fr[1]
    m=Mag(Ftotal)
    Ftotal[0]=Ftotal[0]*mapclearness/m
    Ftotal[1]=Ftotal[1]*mapclearness/m
    Path.append(copy.deepcopy(Current_pos))
    Current_pos[0]+=Ftotal[0]
    Current_pos[1]+=Ftotal[1]
    Distance=DistancetoGoal()
    print(Distance)   
Plot()


