from __future__ import division
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import numpy as np
import random, math
import showGraph
import copy
import sys
import threading
import os

def loadData(obstacle_path,Obs):
    vertices = list()
    codes = [Path.MOVETO]
    with open(obstacle_path) as f:
        quantity = int(f.readline())
        lines = 0
        tmpObs = []
        for line in f:
            coordinates = tuple(map(int, line.strip().split(' ')))
            if len(coordinates) == 1:
                print(tmpObs)
                Obs.append(tmpObs)
                tmpObs = []
            else:
                tmpObs.append(coordinates)
        print(tmpObs)
        Obs.append(tmpObs)

# from the start point (x0,y0)
# draw a horizental line y = y0(x > x0)
# to see if it intersect with the line ((x1,y1),(x2,y2))
def isIntersec(line,point):
    x0 = point[0]
    y0 = point[1]
    x1 = line[0][0]
    y1 = line[0][1]
    x2 = line[1][0]
    y2 = line[1][1]
    if y2 == y1:
        if y0 == y1:
            if (x1 >= x0 and x1 <= xx) or (x2 >= x0 and x2 <= x1):
                return True
        else:
            return False

    x = ((y0 - y1)*(x2 - x1))/(y2-y1) + x1
    if x > x0:
        if x1 > x2:
            if x < x1 and x > x2:
                return True
        if x2 > x1:
            if x < x2 and x > x1:
                return True 
        if x1 == x2:
            if (y1 >= y0 and y0 >= y2) or (y1 <= y0 and y0 <= y2):
                return True
    return False


def segmentintersect(p1, p2, q1, q2):
        def crossprod2d(v1, v2):
            return v1[0] * v2[1] - v1[1] * v2[0]
        p, q, r, s = p1, q1, p2 - p1, q2 - q1

        if crossprod2d(r, s) == 0:
            if crossprod2d((q - p), r) == 0:
                return False
            else:
                return False

        u1 = crossprod2d((q - p), s) / crossprod2d(r, s)
        u2 = crossprod2d((p - q), r) / crossprod2d(s, r)
        if 0. < u1 < 1. and 0. < u2 < 1.:
            return True
        else:
            return False

def isIntersecOb(p1,p2,ob):
    for e in ob:
        q1 = np.array([e[0][0],e[0][1]])
        q2 = np.array([e[1][0],e[1][1]])
        if(segmentintersect(p1,p2,q1,q2)):
            return True
    return False

# for a perticular obsticle ob
# determine wether point lies in it  
def isInOb(ob,point):
    intersec = 0
    # print("in isInOb")
    # print("the ob = ")
    # print(ob)
    # print("the point = ")
    # print(point)
    lineNum = 1
    for e in ob:
        # print(lineNum)
        lineNum+=1
        if isIntersec(e,point):
            # print("Intersect")
            intersec+=1
    if intersec%2 == 0:
        return False
    else:
        return True

# for all obsticles obslines
# determine wether point lies in it  
def isIn(obslines,point):
    for e in obslines:
        if isInOb(e,point):
            return True
    return False

def isIntersecObs(obslines,point1,point2):
    for e in obslines:
        if isIntersecOb(point1,point2,e):
            return True
    return False

# obsticles points
obs = []
# obsticles lines
obslines = []
CpointSet = []
CpointBefSet = []
CpointSetRverse = []
CpointBefSetReverse = []
reachFlag = False
reachIdxReverse = -1
# mouse callback only for debug
def on_button_press(event):
    print("[%f,%f]"%(event.xdata, event.ydata))
    for e in obslines:
        print(e)
    if isIn(obslines,(event.xdata,event.ydata)):
        print("in the obstacle")
    else:
        print("not in the obstacle")
    ax.scatter(event.xdata, event.ydata,c = 'g')
    x_list = [0,event.xdata]
    y_list = [0,event.ydata]
    ax.plot(x_list, y_list, color='r', linewidth=1, alpha=0.6)
    fig.canvas.draw()

# determine whether point lies in the map range
def isInRange(point):
    if point[0] < 0 or point[0] > 600 or point[1] < 0 or point[1] > 600:
        return False
    else:
        return True

# try to add new point to RRT root at start 
def stepForward(start,goal):
    global CpointSet
    global CpointBefSet    
    global CpointSetRverse

    para_stepsize = 20
    para_threshold = 20
    para_Dirthreshold = 0.3
    randPoint = np.array([random.randint(0,600),random.randint(0,600)])
    if random.random() < para_Dirthreshold:
        randPoint = np.array([goal[0],goal[1]])

    minDis = sys.float_info.max
    nearest = np.array([0.0,0.0])
    for e in CpointSet:
        p = np.array([e[0],e[1]])
        distance = np.linalg.norm(randPoint-p)
        if distance < minDis:
            minDis = distance
            nearest = p

    randDirt = (randPoint - nearest)/(np.linalg.norm(randPoint-nearest))
    #randDirt = np.array([1.0,0.0])
    # print("nearest")
    # print(nearest)
    newPoint = para_stepsize * randDirt + nearest
    # print("newPoint")
    # print(newPoint)
    if not isIn(obslines,(newPoint[0],newPoint[1])) and isInRange(newPoint) and not isIntersecObs(obslines,nearest,newPoint):
        CpointBefSet.append((nearest[0],nearest[1]))
        CpointSet.append((newPoint[0],newPoint[1]))
        x_list = [nearest[0],newPoint[0]]
        y_list = [nearest[1],newPoint[1]]
        ax.plot(x_list, y_list, color='r', linewidth=1, alpha=0.6)
        ax.scatter(newPoint[0], newPoint[1],c = 'g')
        fig.canvas.draw()
        # gnp = np.array([goal[0],goal[1]])
        # if np.linalg.norm(newPoint - gnp) < para_threshold:
        #     print("Reached Goal")
        #     return False
        # else:
        #     return True
        if checkMet(CpointSetRverse,newPoint,para_threshold):
            print("Reached Goal")
            global reachFlag
            reachFlag = True
        return True
    else:
        return False

# Determine whether the point met the RRT pointSet
def checkMet(pointSet,point,threshold):
    print("in checkMet pointSetsize = %d"%(len(pointSet)))
    minDis = sys.float_info.max
    minIdx = 0

    for i in range(len(pointSet)):
        enp = np.array([pointSet[i][0],pointSet[i][1]])
        tmpDis = np.linalg.norm(point - enp)
        if tmpDis< minDis:
            minIdx = i
            minDis = tmpDis

    # print("now point = ")
    # print("now mindis = %f"%(minDis))
    # print(point)
    # print("now pointSet = ")
    # for e in pointSet:
    #     print(e)

    # os.system("pause")
    if minDis < 80:
        print("checkMet point = ")
        print(point)
        print("checkMet other point = ")
        print(pointSet[minIdx])
        print("indx = %d"%(minIdx))

        print("minDis = %f"%(minDis))

    if minDis < threshold:
        global reachIdxReverse
        reachIdxReverse = minIdx
        return True
    else:
        return False


# try to add new point to RRT root at end 
def stepForwardReverse(start,goal):
    global CpointSet    
    global CpointSetRverse
    global CpointBefSetReverse

    para_stepsize = 20
    para_threshold = 20
    para_Dirthreshold = 0.3
    randPoint = np.array([random.randint(0,600),random.randint(0,600)])
    if random.random() < para_Dirthreshold:
        randPoint = np.array([start[0],start[1]])

    minDis = sys.float_info.max
    nearest = np.array([0.0,0.0])
    for e in CpointSetRverse:
        p = np.array([e[0],e[1]])
        distance = np.linalg.norm(randPoint-p)
        if distance < minDis:
            minDis = distance
            nearest = p

    randDirt = (randPoint - nearest)/(np.linalg.norm(randPoint-nearest))
    #randDirt = np.array([1.0,0.0])
    # print("nearest")
    # print(nearest)
    newPoint = para_stepsize * randDirt + nearest
    # print("newPoint")
    # print(newPoint)
    if not isIn(obslines,(newPoint[0],newPoint[1])) and isInRange(newPoint) and not isIntersecObs(obslines,nearest,newPoint):
        CpointSetRverse.append((newPoint[0],newPoint[1]))
        CpointBefSetReverse.append((nearest[0],nearest[1]))

        x_list = [nearest[0],newPoint[0]]
        y_list = [nearest[1],newPoint[1]]
        ax.plot(x_list, y_list, color='r', linewidth=1, alpha=0.6)
        ax.scatter(newPoint[0], newPoint[1],c = 'y')
        fig.canvas.draw()
        gnp = np.array([start[0],start[1]])
        # if checkMet(CpointSet,newPoint,para_threshold):
        #     print("Reached Goal")
        #     global reachFlag
        #     reachFlag = True
        return True
    else:
        return False

def startRRTBiDirt(start,goal):
    # os.system("pause")
    global CpointSet    
    global CpointSetRverse
    global CpointBefSetReverse
    global CpointBefSet

    CpointSet = [start]
    CpointSetRverse = [goal]
    while True:
        while not stepForward(start,goal):
            pass
        # print("ForwardEnd")
        while not stepForwardReverse(start,goal):
            pass
        # print("BackwardEnd")
        if reachFlag:
            break
        # print("finding path")

    pointBef = CpointBefSet[-1]
    pointAfter = CpointSet[-1]
    while True:
        # print("get here")
        x_list = [pointBef[0],pointAfter[0]]
        y_list = [pointBef[1],pointAfter[1]]   
        ax.plot(x_list, y_list, color='b', linewidth=3, alpha=0.6) 
        fig.canvas.draw()
        befidx = -1;
        for i in range(len(CpointSet)):
            if CpointSet[i] == pointBef:
                befidx = i
        if befidx == 0:
            break
        else:
            pointAfter = pointBef
            pointBef = CpointBefSet[befidx-1]

    global reachIdxReverse
    print("reached goal the bef idx = %d"%(reachIdxReverse))

    pointBef = CpointBefSetReverse[reachIdxReverse-1]

    print("poinrbef = ")
    print(pointBef)

    pointAfter = CpointSetRverse[reachIdxReverse]
    while True:
        # print("get here")
        x_list = [pointBef[0],pointAfter[0]]
        y_list = [pointBef[1],pointAfter[1]]   
        ax.plot(x_list, y_list, color='b', linewidth=3, alpha=0.6) 
        fig.canvas.draw()
        befidx = -1;
        for i in range(len(CpointSetRverse)):
            if CpointSetRverse[i] == pointBef:
                befidx = i
        if befidx == 0:
            break
        else:
            pointAfter = pointBef
            pointBef = CpointBefSetReverse[befidx-1]
    print("Reached Goal")



if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('obstacle_path',
                        help="File path for obstacle set")
    parser.add_argument('start_goal_path',
                        help="File path for obstacle set")
    args = parser.parse_args()

    fig, ax = plt.subplots()
    path = showGraph.build_obstacle_course(args.obstacle_path, ax)
    start, goal = showGraph.add_start_and_goal(args.start_goal_path, ax)
    loadData(args.obstacle_path,obs)
    print("obs.size = %d"%(len(obs)))
    for o in obs:
        tmplines = []
        if len(o) == 0:
            continue
        for i in range(len(o) - 1):
            # print(o[i])
            tmplines.append((o[i],o[i+1]))
        print('\n')
        tmplines.append((o[len(o)-1],o[0]))
        obslines.append(tmplines)
    print("obslines.size = %d"%(len(obslines)))
    # for o in obslines:
    #     print('\n')
    #     for l in o:
    #         print(l)
    print("start at ")
    print(start)
    CpointSet.append(start)
    CpointSetRverse.append(goal)
    print("goal at")
    print(goal)
    # fig.canvas.mpl_connect('button_press_event', on_button_press)
    td =threading.Thread(target = startRRTBiDirt, args = (start,goal))
    td.start()
    # print("get here")
    # startRRTOneDirt(start)
    fig.canvas.draw()
    plt.show()
    print("get here")