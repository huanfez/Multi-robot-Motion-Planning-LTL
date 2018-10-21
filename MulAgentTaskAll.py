import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.pyplot import draw
import numpy as np
import scipy.linalg
from collections import Iterable
from scipy.misc import imread
import matplotlib.cbook as cbook

import NUSMVFile as nus
import LQR as lqr
import obsDetection as obsDet
import inOutAut as autOp

# Define index number
SUBTASK = 1
OBS = 2
ActPos = 3
PATH = 4
POSNUM = 5
GROUP = 6
COLOR = 7

# subLang[1]= [55 88 99], subLang[2]= [63 ,72], subLang[3]= [43 79]
tasks = [[43, 74, 81], [62], [98, 16]]
startPos = [11, 91, 90]

# Define agent:=[id, Path, subtaskSet, color]
agents = []
m = 0
#targets = [55, 88, 99, 72, 63, 43, 79]
#obstacles = [33, 44, 76, 93]
#obstacles = [10, 42, 34, 38, 40, 54, 58, 60, 61]
obstacles = [3,4,5,6,7,17,26,27,35,36,37,41,51,64,65,67,75,77,85,95,97,99,100]
gridWidth = 10
gridLength = 10
Color = ['r', 'g', 'b']
rectColor = ["red", "green", "blue" ]
plotPlan = ['','','']
rectPlan = ['','','']

Steps = 1
timeInterval = 0.05

# Flatten a list
def flatten(lis):
     for item in lis:
         if isinstance(item, Iterable) and not isinstance(item, basestring):
             for x in flatten(item):
                 yield x
         else:        
             yield item

# Coordinate transformation
def oneD2twoD(data):
    y = (data-1)/gridWidth + 1
    x = data%gridWidth
    if x == 0:
        x = 10
    return x, y

# Coordinate transformation
def twoD2oneD(dataX, dataY):
    data = dataX + (dataY - 1) * gridWidth
    return data
    
# Inter-collision avoidance
def collAvoid(agents, plotPlan):
    for agent in agents:
        if agent[PATH][agent[POSNUM]] != agent[PATH][-1]:
            # if path includes obstacles, repPath(agent)
            if agent[GROUP] != []:
                agentNeiObs_ = []
                for groupMem in agent[GROUP]:
                    neiAgent = agents[groupMem]
                    if neiAgent[PATH][neiAgent[POSNUM]] != neiAgent[PATH][-1]:
                        neiAgent_NeiPos = neiAgent[PATH][neiAgent[POSNUM] + 1]
                        agentNeiObs_ = agentNeiObs_ + [twoD2oneD(neiAgent_NeiPos[0], neiAgent_NeiPos[1])]
                tmpObs = obsDet.CollisionDtc(agentNeiObs_, agent[PATH][agent[POSNUM]])
                print agent[PATH][agent[POSNUM]], tmpObs
                agent[OBS] = agent[OBS] + tmpObs
                if tmpObs:
                    print agent, tmpObs
                    repPath(agent)
                    #p1.plot(rePlanPath[0], rePlanPath[1], agent[COLOR])
                    agent[OBS] = [e for e in agent[OBS] if e not in tmpObs]
                    #plotPlans(agent, plotPlan)
    return 0

# RePlan path_i^j for agent_i with e_gi^j
def repPath(agent):
    ## ltl specification
    ltlSpec = 'LTLSPEC ! ( F x.state = ' + str(agent[SUBTASK][0]) + ')'
    #print agent[SUBTASK][0]

    ## 1st time try to plan
    expansion = 0
    pathNum = twoD2oneD(agent[PATH][agent[POSNUM]][0], agent[PATH][agent[POSNUM]][1])
    #print pathNum
    nus.makeNusmvFile(ltlSpec, pathNum, agent[SUBTASK][0], expansion, agent[OBS])
    agent[PATH] = nus.NuSMVPlan()
    
    ## if cannot find solution, try again             
    while agent[PATH] == []:
        expansion = expansion + 1
        nus.makeNusmvFile(ltlSpec, pathNum, agent[SUBTASK][0], expansion, agent[OBS])
        agent[PATH] = nus.NuSMVPlan()
    agent[POSNUM] = 0

    #coordinateX = [coordinate[0] for coordinate in agent[PATH]]
    #coordinateY = [coordinate[1] for coordinate in agent[PATH]]
    #return coordinateX, coordinateY
    return 0

# Update position for agent_i with e_gi^j
def updatePos(agent):
    agentNexPos = agent[PATH][agent[POSNUM] + 1]
    agent[ActPos] = lqr.ctrOutput([[agent[ActPos][0]],[agent[ActPos][1]]],             \
                        [[agentNexPos[0]],[agentNexPos[1]]], 0.1)
    error = np.array(agent[ActPos]) - np.array(agentNexPos)
    if np.linalg.norm(error) < 0.01:
        agent[POSNUM] = agent[POSNUM] + 1
    #print agent[PATH][agent[POSNUM]], agent[PATH][-1]
    return 0

# Plot motion
def plotMotion(agents):
    circle1 = ['','','']
    circle2 = ['','','']
    for agent in agents:
        circle1[agent[0]] = plt.Circle(agent[ActPos], 0.2, color = agent[COLOR])
        circle2[agent[0]] = plt.Circle(agent[ActPos], 1.05, color = agent[COLOR], alpha = 200)
        p2.add_artist(circle1[agent[0]])
        p2.add_artist(circle2[agent[0]])
        p2.plot(agent[ActPos][0], agent[ActPos][1], '.', color = agent[COLOR])
    draw()
    plt.pause(timeInterval)
    for agent in agents:
        circle1[agent[0]].remove()
        circle2[agent[0]].remove()
    draw()
    return 0

# Plot Plan
def plotPlans(agent, plotPlan, rectPlan):
    plt.ion
    #for agent in agents:    
    coordinateX = [coordinate[0] for coordinate in agent[PATH]]
    coordinateY = [coordinate[1] for coordinate in agent[PATH]]
        #plotPlan[agent[0]], = p1.plot(coordinateX, coordinateY, agent[COLOR])
    (plotPlan[agent[0]]).set_xdata(coordinateX)
    (plotPlan[agent[0]]).set_ydata(coordinateY)

    rectXStrt = min(coordinateX) - 0.4
    rectYStrt = min(coordinateY) - 0.4
    rectWidth = max(coordinateX) - rectXStrt + 0.4
    rectHeight = max(coordinateY) - rectYStrt + 0.4
    rectPlan[agent[0]].set_bounds(rectXStrt, rectYStrt, rectWidth, rectHeight)
    draw()
    plt.ioff
    return 0

########## Main Process ##########
# 0. read automaton
#taskAuts = autOp.readAut('/media/hz/New Volume/GAP/GAP_manual/decomAut.txt')

# 1.1 Initialize the environment
fig1 = plt.figure(figsize=(24, 11.5))
fig1.patch.set_facecolor('white')
datafile = cbook.get_sample_data('/media/hz/New Volume/proposal/MAP/RoomPlans.png')
img = imread(datafile)

# the first figure
p1 = plt.subplot(121)
col, row = np.meshgrid(np.array(range(gridWidth+1)) + 0.5, np.array(range(gridLength+1)) + 0.5)
p1.plot(col, row, 'k')
p1.plot(row, col, 'k')
p1.axis('equal')
p1.axis('off')
plt.title('Planning Overview', fontsize=20)
#p1.imshow(img, zorder=0, extent=[0.52, 10.5, 0.5, 10.5])

for obs in obstacles:
    p1.plot((oneD2twoD(obs))[0], (oneD2twoD(obs))[1], 'x', color = 'k', markersize=50)

# the second figure
p2 = plt.subplot(122)
p2.plot(col, row, 'k--')
p2.plot(row, col, 'k--')
p2.axis('equal')
p2.axis('off')
plt.title('Motion', fontsize=20)
#p2.imshow(img, zorder=0, extent=[0.52, 10.5, 0.5, 10.5])

draw()
plt.pause(0.5)

# 1.2 Initialize all agents
for task in tasks:
    # 0. Allocate subtasks
    for target in task:
        p1.plot((oneD2twoD(target))[0], (oneD2twoD(target))[1], '*', color = Color[m], markersize = 30)
        #p2.plot((oneD2twoD(target))[0], (oneD2twoD(target))[1], '*', color = Color[m], markersize = 30)

    # 1. obstacles
    obsInd = obsDet.obstacleDtc(obstacles, oneD2twoD(startPos[m]))
    if obsInd:
        for obsi in obsInd:
            p2.plot((oneD2twoD(obsi))[0], (oneD2twoD(obsi))[1], 'x', color = 'k', markersize=50)
    obsTask = [item for item in tasks if item != task]
    obsInd = obsInd + list(flatten(obsTask))

    # 2. ltl specification & NuSMV motion planning and path
    ltlSpec = 'LTLSPEC ! ( F x.state = ' + str(tasks[m][0]) + ')'
    expansion = 0
    nus.makeNusmvFile(ltlSpec, startPos[m], tasks[m][0], expansion, obsInd)
    path = nus.NuSMVPlan()
    while path == []:
        expansion = expansion + 1
        nus.makeNusmvFile(ltlSpec, startPos[m], tasks[m][0], expansion, obsInd)
        path = nus.NuSMVPlan()
    coordinateX = [coordinate[0] for coordinate in path]
    coordinateY = [coordinate[1] for coordinate in path]
    plotPlan[m], = p1.plot(coordinateX, coordinateY, Color[m], linewidth=2.0)
    rectXStrt = min(coordinateX) - 0.4
    rectYStrt = min(coordinateY) - 0.4
    rectWidth = max(coordinateX) - rectXStrt + 0.4
    rectHeight = max(coordinateY) - rectYStrt + 0.4
    #### rectPlan[m] = p1.add_patch(patches.Rectangle((rectXStrt, rectYStrt), rectWidth, rectHeight, facecolor=rectColor[m], alpha=0.5, linestyle='dashed', linewidth=2, edgecolor=rectColor[m]))

    # 3.group members
    groupMem = []
    agents.append([m, task[:], obsInd, oneD2twoD(startPos[m]), path, 0,  groupMem, Color[m]])
    m = m + 1

draw()
plt.pause(2)
print agents
## 2. Update planned paths and positions
while Steps:
    # Update time steps
    Steps = Steps + 1

    # Check if any agents finish task allocation
    if all(agent[SUBTASK] for agent in agents) == False: # someone has finished its local tasks
        break
        # Wait for human to reallocate tasks

    # Obstacle detection
    for agent in agents:
        newObs = obsDet.obstacleDtc(obstacles, agent[PATH][agent[POSNUM]])
        newObs =  list(set(newObs) - set(agent[OBS]))
        if newObs:
            for obsi in newObs:
                p2.plot(oneD2twoD(obsi)[0], oneD2twoD(obsi)[1], 'x', color='k', markersize = 50)
            agent[OBS] = agent[OBS] + newObs
            if agent[PATH][agent[POSNUM]] != agent[PATH][-1]:
                repPath(agent)
                #### plotPlans(agent, plotPlan, rectPlan)

    # Communications among agents
    for agent in agents:
        agent[GROUP] = []
        agent_cPos = agent[PATH][agent[POSNUM]]
        for agentNei in agents:
            if agent[0] != agentNei[0]:
                agentNei_cPos = agentNei[PATH][agentNei[POSNUM]]
                agentNeiObs = [twoD2oneD(agentNei_cPos[0],agentNei_cPos[1])]
                if obsDet.obstacleDtc(agentNeiObs, agent_cPos):
                    agent[GROUP] = list(set(agent[GROUP] + [agentNei[0]]))
                    agent[OBS] = list(set(agent[OBS] + agentNei[OBS]))
                    agent[OBS] = [item for item in agent[OBS] if item not in tasks[agent[0]] ]
                    print agent[0], agent[OBS], tasks[agent[0]]
                    if agent[PATH][agent[POSNUM]] != agent[PATH][-1]:
                        repPath(agent)
                        #### plotPlans(agent, plotPlan, rectPlan)
                    #print agent[0], obsDet.obstacleDtc(agentNeiObs, agent_cPos)

    ## Inter Collision detection and avoidance
    collAvoid(agents, plotPlan)

    for agent in agents:
        if agent[PATH][agent[POSNUM]] != agent[PATH][-1]: # agent_i not finsihed current path
            updatePos(agent)
        else: # agent_i has finsihed current path
            del agent[SUBTASK][0]    # E_gi\e_gi^j
            if agent[SUBTASK] == []: # If no subtasks left
                break
            else: # subtaks are left
            # Plan path_i^j for agent_i with e_gi^j
                repPath(agent)     
                #p1.plot(rePlanPath[0], rePlanPath[1], agent[COLOR])
                #### plotPlans(agent, plotPlan, rectPlan)
    # Plot motion
    plotMotion(agents)
for agent in [agents[1]]:
    circle1 = plt.Circle(agent[ActPos], 0.2, color = agent[COLOR])
    p2.add_artist(circle1)
    circle2 = plt.Circle(agent[ActPos], 1.05, color = agent[COLOR], alpha = 200)
    p2.add_artist(circle2)

#########################################  Phase 2  #############################################
while Steps:
    # Update time steps
    Steps = Steps + 1

    # Check if any agents finish task allocation
    #if all(agent[SUBTASK] for agent in agents) == False: # someone has finished its local tasks
        #break
        # Wait for human to reallocate tasks
    if agents[0][PATH][agents[0][POSNUM]][0] == 3 and agents[0][PATH][agents[0][POSNUM]][1] == 6:
        break

    # Obstacle detection
    for agent in [agents[0], agents[2]]:
        newObs = obsDet.obstacleDtc(obstacles, agent[PATH][agent[POSNUM]])
        newObs =  list(set(newObs) - set(agent[OBS]))
        if newObs:
            for obsi in newObs:
                p2.plot(oneD2twoD(obsi)[0], oneD2twoD(obsi)[1], 'x', color='k', markersize = 50)
            agent[OBS] = agent[OBS] + newObs
            if agent[PATH][agent[POSNUM]] != agent[PATH][-1]:
                repPath(agent)
                #### plotPlans(agent, plotPlan, rectPlan)

    # Communications among agents
    for agent in [agents[0], agents[2]]:
        agent[GROUP] = []
        agent_cPos = agent[PATH][agent[POSNUM]]
        for agentNei in [agents[0], agents[2]]:
            if agent[0] != agentNei[0]:
                agentNei_cPos = agentNei[PATH][agentNei[POSNUM]]
                agentNeiObs = [twoD2oneD(agentNei_cPos[0],agentNei_cPos[1])]
                if obsDet.obstacleDtc(agentNeiObs, agent_cPos):
                    agent[GROUP] = list(set(agent[GROUP] + [agentNei[0]]))
                    agent[OBS] = list(set(agent[OBS] + agentNei[OBS]))
                    agent[OBS] = [item for item in agent[OBS] if item not in tasks[agent[0]] ]
                    print agent[0], agent[OBS], tasks[agent[0]]
                    if agent[PATH][agent[POSNUM]] != agent[PATH][-1]:
                        repPath(agent)
                        #### plotPlans(agent, plotPlan, rectPlan)
                    #print agent[0], obsDet.obstacleDtc(agentNeiObs, agent_cPos)

    ## Inter Collision detection and avoidance
    #collAvoid([agents[0], agents[2]], plotPlan)

    for agent in [agents[0], agents[2]]:
        if agent[PATH][agent[POSNUM]] != agent[PATH][-1]: # agent_i not finsihed current path
            updatePos(agent)
        else: # agent_i has finsihed current path
            del agent[SUBTASK][0]    # E_gi\e_gi^j
            if agent[SUBTASK] == []: # If no subtasks left
                break
            else: # subtaks are left
            # Plan path_i^j for agent_i with e_gi^j
                repPath(agent)     
                #p1.plot(rePlanPath[0], rePlanPath[1], agent[COLOR])
                #### plotPlans(agent, plotPlan, rectPlan)
    # Plot motion
    plotMotion([agents[0], agents[2]])


#########################################  Phase 3  #############################################
agents[0][SUBTASK] = [74]
(agents[1][SUBTASK]).append(81)
(agents[1][OBS]).remove(81)
repPath(agents[1])
circle1.remove()
circle2.remove()
draw()

while Steps:
    # Update time steps
    Steps = Steps + 1

    # Check if any agents finish task allocation
    if all(agent[SUBTASK] for agent in agents) == False: # someone has finished its local tasks
        print "FINISH"
        break
        # Wait for human to reallocate tasks

    # Obstacle detection
    for agent in agents:
        newObs = obsDet.obstacleDtc(obstacles, agent[PATH][agent[POSNUM]])
        newObs =  list(set(newObs) - set(agent[OBS]))
        if newObs:
            for obsi in newObs:
                p2.plot(oneD2twoD(obsi)[0], oneD2twoD(obsi)[1], 'x', color='k', markersize = 50)
            agent[OBS] = agent[OBS] + newObs
            if agent[PATH][agent[POSNUM]] != agent[PATH][-1]:
                repPath(agent)
                #### plotPlans(agent, plotPlan, rectPlan)

    # Communications among agents
    for agent in agents:
        agent[GROUP] = []
        agent_cPos = agent[PATH][agent[POSNUM]]
        for agentNei in agents:
            if agent[0] != agentNei[0]:
                agentNei_cPos = agentNei[PATH][agentNei[POSNUM]]
                agentNeiObs = [twoD2oneD(agentNei_cPos[0],agentNei_cPos[1])]
                if obsDet.obstacleDtc(agentNeiObs, agent_cPos):
                    agent[GROUP] = list(set(agent[GROUP] + [agentNei[0]]))
                    agent[OBS] = list(set(agent[OBS] + agentNei[OBS]))
                    agent[OBS] = [item for item in agent[OBS] if item not in tasks[agent[0]] ]
                    print agent[0], agent[OBS], tasks[agent[0]]
                    if agent[PATH][agent[POSNUM]] != agent[PATH][-1]:
                        repPath(agent)
                        #### plotPlans(agent, plotPlan, rectPlan)
                    #print agent[0], obsDet.obstacleDtc(agentNeiObs, agent_cPos)

    ## Inter Collision detection and avoidance
    collAvoid(agents, plotPlan)

    for agent in agents:
        if agent[PATH][agent[POSNUM]] != agent[PATH][-1]: # agent_i not finsihed current path
            updatePos(agent)
        else: # agent_i has finsihed current path
            del agent[SUBTASK][0]    # E_gi\e_gi^j
            if agent[SUBTASK] == []: # If no subtasks left
                print "FINISH1"
                break
            else: # subtaks are left
            # Plan path_i^j for agent_i with e_gi^j
                repPath(agent)     
                #p1.plot(rePlanPath[0], rePlanPath[1], agent[COLOR])
                #### plotPlans(agent, plotPlan, rectPlan)
    # Plot motion
    plotMotion(agents)

for agent in agents:
    circle1 = plt.Circle(agent[ActPos], 0.2, color = agent[COLOR])
    p2.add_artist(circle1)
    circle2 = plt.Circle(agent[ActPos], 1.05, color = agent[COLOR], alpha = 200)
    p2.add_artist(circle2)

print agents
plt.show()
