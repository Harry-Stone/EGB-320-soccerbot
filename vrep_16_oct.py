import numpy as np
import matplotlib.pyplot as plt
from soccerbot_lib import *

sceneParameters = SceneParameters()
robotParameters = RobotParameters()
robotParameters.driveType = 'differential'
robotParameters.maxBallDetectionDistance = 10
robotParameters.maxGoalDetectionDistance = 10

fovsize = 1 # 1 radian
fovsamples = 60 #samples in FOV 
field = range(1,fovsamples)
grad = 1/30
obswid = (20+20)/100 #cm
obsMem = [0,0] #number that decrements and is set to something when an obstacle is in the exremity
obsMemFrames = 15 # number of frmaes untill an obstacle is forgotten
goal = 'blue'
max_fd_vel = 0.2
max_rvel = 0.5
k = 0.8

def zeroes (num):
    out = [0] * num
    return out

def calcRepel(obstacles,field):
    obsMem[0]-=1
    obsMem[1]-=1
    if (obsMem[0]>0 or obsMem[1]>0) and obstacles == None:
        obstacles = []
    if obsMem[0]>0:
        obstacles.append([0.33,-.39])
    if obsMem[1]>0:
        obstacles.append([0.33,.39])
    if obstacles != None:
        for obstacle in obstacles:
            if obstacle[0]<0.5:
                if obstacle[1]>0.4:
                    obsMem[1]=obsMemFrames
                elif obstacle[1]<-0.4:
                    obsMem[0]=obsMemFrames
            
            for i in range(0,fovsamples):
                obstacledist=obstacle[0]
                obstacleangle=obstacle[1]
                obs_falloff = math.atan(0.5*obswid/obstacledist)
                samplerad = (((-1*fovsamples/2)+i)*(fovsize/fovsamples))
                if field[i] != -1:
                    field[i]+=(abs(((-1*fovsamples/2)+i)*fovsize/fovsamples-obstacleangle)*0.3)-0.2#(grad/obstacledist)
                if (samplerad > obstacleangle-obs_falloff and samplerad < obstacleangle+obs_falloff):
                    field[i]=-1
    return field

    
def calcfield(obstacles, objectiveRB):
    field = zeroes(fovsamples)
    field = calcRepel(obstacles,field)
    if objectiveRB != None:       
        for i in range(0,fovsamples):
            field[i]+=1-abs(((-1*fovsamples/2)+i)*fovsize/fovsamples-objectiveRB[1])*(0.5)
    else:
        field[0]=1
    visualise(field)
    return field


def visualise(data):
    plt.clf()
    plt.plot(data)
    plt.draw()
    plt.pause(0.001)

def findmax(array):
    bestindex =-1
    bestval = -1
    midval = array[30]
    for i in range(0,len(array)):
        if array[i]>bestval:
            bestval=array[i]
            bestindex=i
    print('speed:'+str(bestindex)+' rps:'+str(indextorad(bestindex)))
    return[bestindex,midval]

def indextorad(index):
    return (((-1*fovsamples/2)+index)*(fovsize/fovsamples))

def setdrive(dist, rps):
    soccerBotSim.SetTargetVelocities(max_fd_vel*(1-k*abs(rps/max_rvel)),0,rps)
    print('speed:'+str(max_fd_vel*(1-k*abs(rps/max_rvel)))+' rps:'+str(rps))


if __name__ == '__main__':

    try:

        plt.ion()
        plt.show()
        soccerBotSim = VREP_SoccerBot('127.0.0.1', robotParameters, sceneParameters)
        soccerBotSim.SetCameraHeight(.06)
        soccerBotSim.StartSimulator()
        soccerBotSim.SetTargetVelocities(0, 0, 0)

        while 1 >= 0:
            ballRB, blueRB, yellowRB, obstaclesRB = soccerBotSim.GetDetectedObjects()
            if soccerBotSim.BallInDribbler() == False:
                potentialField = calcfield(obstaclesRB,ballRB)
                objectives = findmax(potentialField)
                setdrive(objectives[1],indextorad(objectives[0]))
            else:
                
                if goal == 'blue':
                    goalRB = blueRB
                else:
                    goalRB = yellowRB
                potentialField = calcfield(obstaclesRB,goalRB)
                objectives = findmax(potentialField)
                setdrive(objectives[1],indextorad(objectives[0]))
                if goalRB != None:
                    print(str(goalRB[0])+' '+str(objectives[1])+' '+str(objectives[0])+' ')
                    if soccerBotSim.BallInDribbler() and goalRB[0]<.5 and objectives[1] >0.8:
                        soccerBotSim.KickBall(1)

            soccerBotSim.UpdateObjectPositions()
            
    except KeyboardInterrupt as e:
        soccerBotSim.StopSimulator()
        
