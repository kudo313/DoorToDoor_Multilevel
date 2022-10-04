from cmath import sqrt
from operator import le
from os import name, remove
import random
from typing import List
import numpy as np
import math
import matplotlib.pyplot as plt
import random
import time
import copy
import openpyxl
from numpy.core.fromnumeric import size
import time


# const
numberOfCus  = 10
numberOfDepot = 1
minDis = 1000000000
nearestPoint = 0
D = 8*60
depot = 0 # repair something with 0 by depot
timeRateTech = 1/0.58
timeRateDrone = 1/0.83
edurance = 30
N_Technican = 5
# read data from txt
f_name = "50.20.2"
f_name_file = f_name + ".txt"
fileName =  f_name + "_" + str(int(time.time())) + "_version2" + ".xlsx"
f = open("./data/" + f_name_file,"r")
oneLine = f.readline()
oneLine = oneLine.split()
numberOfCus = int(oneLine[1])
oneLine = f.readline()
coor_matrix = np.zeros((numberOfCus+numberOfDepot,2))
for i in range(1,numberOfCus+1):
    oneLine = f.readline()
    oneLine = oneLine.split()
    coor_matrix[i][0] = oneLine[0]
    coor_matrix[i][1] = oneLine[1]

# create variable
totalPoint = numberOfCus + numberOfDepot
distanceMatrix =  np.zeros((numberOfCus+numberOfDepot,totalPoint))
timeTechMatrix =  np.zeros((numberOfCus+numberOfDepot,totalPoint))
timeDroneMatrix = np.zeros((numberOfCus+numberOfDepot,totalPoint))
vectorMatrix = np.zeros((totalPoint,2))
cosList = np.zeros((totalPoint))
cornerList = np.zeros((totalPoint))
sortedList = np.zeros((totalPoint),int)
ListMoveOfTech = []
for i in range(N_Technican):
    MoveOfTech = [0]
    ListMoveOfTech.append(MoveOfTech)
indexOfTech = np.zeros(N_Technican,int)
TimeGuessOfTech = []
TheDroneRouteOfPoint = np.zeros(totalPoint, int)
OrderOfPointInDroneRoute = np.zeros(totalPoint, int)
for i in range(totalPoint):
    TheDroneRouteOfPoint[i] = -1
    OrderOfPointInDroneRoute[i] = -1
for i in range(N_Technican):
    GuessTime = [0]
    TimeGuessOfTech.append(GuessTime)
TimeVisitPoint = np.zeros((totalPoint))

for i in range(totalPoint):
    sortedList[i]  = i

for i in range(totalPoint):
    for j in range(totalPoint):
        if (i != j):
            distanceMatrix[i][j] = math.sqrt(pow(coor_matrix[i][0] - coor_matrix[j][0],2) + pow(coor_matrix[i][1] - coor_matrix[j][1],2))
            timeTechMatrix[i][j] = timeRateTech*distanceMatrix[i][j]
            timeDroneMatrix[i][j] = timeRateDrone*distanceMatrix[i][j]
        else:
            distanceMatrix[i][j]  = 0
            timeDroneMatrix[i][j] = 0
            timeTechMatrix[i][j] = 0
cusInDroneFlyRange = []
for i in range(1, totalPoint):
    if timeDroneMatrix[depot][i] + timeDroneMatrix[i][depot] <= edurance:
        cusInDroneFlyRange.append(i)
numCusInDroneRange = len(cusInDroneFlyRange)
feasible = False
feasibleWithDroneEdurance = False
feasibleWithMaximumWorking = False

#function 
def cacul_dis(coor_x1, coor_y1, coor_x2, coor_y2):
    return math.sqrt((coor_x1 - coor_x2)*(coor_x1 - coor_x2) + (coor_y1 - coor_y2)*(coor_y1- coor_y2))
# prepare for matching
current_level = 0
dict_matched_point = {}
first_list_matched_point = {}
for i in range(1, totalPoint):
    first_list_matched_point[i] = i
dict_matched_point[0] = first_list_matched_point
dict_coor_matrix = {}
dict_coor_matrix[current_level] = copy.deepcopy(coor_matrix)
dict_total_point = {}
dict_total_point[current_level] = totalPoint
total_new_point = totalPoint
# 
#  
#
while total_new_point - 1 > N_Technican:
    #prepare variables
    total_tmp_point = total_new_point
    unmatched_list = []
    for i in range(1, total_new_point):
        unmatched_list.append(i)
    current_coor_matrix = dict_coor_matrix[current_level]

    # read parameter
    para_file = "parameter.txt"
    read_para  = open(para_file, "r")

    # read first line is N
    n = int(read_para.readline())

    # caculate area A
    min_x_posi = 1000000000000
    min_y_posi = 1000000000000
    max_x_posi = -1000000000000
    max_y_posi = -1000000000000
    for i in range(1, total_tmp_point):
        if current_coor_matrix[i][0] < min_x_posi:
            min_x_posi = current_coor_matrix[i][0]
        if current_coor_matrix[i][0] > max_x_posi:
            max_x_posi = current_coor_matrix[i][0]
        if current_coor_matrix[i][1] < min_y_posi:
            min_y_posi = current_coor_matrix[i][1]
        if current_coor_matrix[i][1] > max_y_posi:
            max_y_posi = current_coor_matrix[i][1]
    widths = max_x_posi - min_x_posi
    length = max_y_posi - min_y_posi
    area = widths*length

    # create grid
    h = math.sqrt(area*n/total_tmp_point)
    list_cell = {}
    for i in range(1, total_tmp_point):
        x_posi = int(current_coor_matrix[i][0]/h)
        y_posi = int(current_coor_matrix[i][1]/h)
        posi = (x_posi, y_posi)
        if posi in list_cell.keys():
            list_cell[posi].append(i)
        else:
            list_cell[posi] = [i]

    #prepare variables
    new_coor_matrix = [[0, 0]]
    list_matched_point = {}

    #matching
    total_new_point = 1
    while len(unmatched_list) > 1:
        found_point = unmatched_list[0]
        #find posi of cell obtain found point
        for i in list_cell.keys():
            if found_point in list_cell[i]:
                posi_of_found_point = i
                break

        # find 2 closet point
        min_distance = [1000000000000, 100000000000]
        closet_point  = [-1, -1]
        for i in range(2):
            for j in range(2):
                search_posi = (posi_of_found_point[0] + i, posi_of_found_point[1] + j )
                if search_posi in list_cell.keys():
                    for k in list_cell[search_posi]:
                        if k != found_point:
                            dis_from_found_point = cacul_dis(current_coor_matrix[k][0], current_coor_matrix[k][1], current_coor_matrix[found_point][0], current_coor_matrix[found_point][1])
                            if dis_from_found_point < min_distance[0]:
                                min_distance[0] = dis_from_found_point
                                closet_point[0] = k
                            elif dis_from_found_point < min_distance[1]:
                                min_distance[1] = dis_from_found_point
                                closet_point[1] = k
        
        #matching
        if len(closet_point) > 0:
            unmatched = True
            for i in closet_point:
                if i in unmatched_list and total_tmp_point - 2 >= N_Technican:
                    matched_point = (found_point, i)
                    list_matched_point[total_new_point] = matched_point
                    new_coor = [(current_coor_matrix[found_point][0] + current_coor_matrix[i][0])/2, (current_coor_matrix[found_point][1] + current_coor_matrix[i][1])/2]
                    new_coor_matrix.append(new_coor)
                    unmatched_list.remove(found_point)
                    unmatched_list.remove(i)
                    total_new_point += 1
                    total_tmp_point -= 1
                    unmatched = False
                    break
            if unmatched == True:
                list_matched_point[total_new_point] = found_point
                new_coor_matrix.append(current_coor_matrix[found_point])
                unmatched_list.remove(found_point)
                total_new_point += 1
        else:
            list_matched_point[total_new_point] = found_point
            new_coor_matrix.append(current_coor_matrix[found_point])
            unmatched_list.remove(found_point)
            total_new_point += 1
    if len(unmatched_list) > 0:
        found_point = unmatched_list[0]
        list_matched_point[total_new_point] = found_point
        new_coor_matrix.append(current_coor_matrix[found_point])
        unmatched_list.remove(found_point)
        total_new_point += 1
    current_level += 1        
    dict_matched_point[current_level] = list_matched_point
    dict_coor_matrix[current_level] = new_coor_matrix
    dict_total_point[current_level] = total_new_point

total_level = current_level 
dict_distance_matrix = {}
dict_time_tech_matrix = {}
dict_time_drone_matrix = {}

# create distance matrix for each level
for level in range(total_level + 1):
    total_tmp_point = dict_total_point[level]
    distanceMatrix =  np.zeros((total_tmp_point, total_tmp_point))
    timeTechMatrix =  np.zeros((total_tmp_point, total_tmp_point))
    timeDroneMatrix = np.zeros((total_tmp_point, total_tmp_point))
    coor_matrix = dict_coor_matrix[level]
    for i in range(total_tmp_point):
        for j in range(total_tmp_point):
            if (i != j):
                distanceMatrix[i][j] = math.sqrt(pow(coor_matrix[i][0] - coor_matrix[j][0],2) + pow(coor_matrix[i][1] - coor_matrix[j][1],2))
                timeTechMatrix[i][j] = timeRateTech*distanceMatrix[i][j]
                timeDroneMatrix[i][j] = timeRateDrone*distanceMatrix[i][j]
            else:
                distanceMatrix[i][j]  = 0
                timeDroneMatrix[i][j] = 0
                timeTechMatrix[i][j] = 0
    dict_distance_matrix[level] = distanceMatrix
    dict_time_tech_matrix[level] = timeTechMatrix
    dict_time_drone_matrix[level] = timeDroneMatrix

# Initialise at lowest level
lowest_total_point = dict_total_point[total_level]
lowest_total_customer = lowest_total_point - 1
tech_lowest_route = []
if lowest_total_customer == N_Technican:
    for i in range(N_Technican):
        tmp_route = [0]
        tech_lowest_route.append(tmp_route)
    for i in range(1, lowest_total_customer + 1):
        tech_lowest_route[i - 1].append(i)
    rand_point_get_by_drone = random.randrange(1, lowest_total_customer + 1)
    drone_lowest_routes = [[0]]
    drone_lowest_routes[0].append(rand_point_get_by_drone)
previous_tech_routes = copy.deepcopy(tech_lowest_route)
previous_drone_routes = copy.deepcopy(drone_lowest_routes)

#function
def fitness(ListMoveTech, RoutesDrone, Visited):
    global alpha1, alpha2, beta, feasible, N_Technican, feasibleWithDroneEdurance, feasibleWithMaximumWorking
    # create variable
    totalPoint = 1
    for i in range(len(ListMoveTech)):
        for j in range(1, len(ListMoveTech[i])):
            if ListMoveTech[i][j] != 0:
                totalPoint += 1
    TimeGuessOfTechIF = []
    N_Technican_temp = len(ListMoveTech)
    for i in range(N_Technican_temp):
        GuessTime = [0]
        TimeGuessOfTechIF.append(GuessTime)
    TimeTechLeavePoint = np.zeros(totalPoint)
    TimeTechArrivePoint = np.zeros(totalPoint)
    TimeStartOfRoutesIF = np.zeros(len(RoutesDrone))
    TimeDroneArrivePoint = copy.deepcopy(RoutesDrone)
    TimeDroneLeavePoint = copy.deepcopy(RoutesDrone)
    TimeDroneArriveDepot = np.zeros(len(RoutesDrone))
    TimeTechArriveDepot = np.zeros(len(ListMoveTech))
    AfterPointCarrySample = np.zeros(totalPoint, int)
    for i in range(1, totalPoint):
        AfterPointCarrySample[i] = -1
    TimeSampleInDepot = np.zeros(totalPoint)
    totalSampleWaitTime = 0
    totalDurationViolation = 0
    totalWorkingTimeViolation = 0
    #
    for i in range(N_Technican_temp):
        N_PointOfTechRoute = len(ListMoveTech[i])
        timeOfTech = 0
        for j in range(1,N_PointOfTechRoute):
            thisPoint = ListMoveTech[i][j]
            prePoint = ListMoveTech[i][j - 1]
            timeArrivePoint = timeOfTech + timeRateTech*distanceMatrix[prePoint][thisPoint]
            TimeGuessOfTechIF[i].append(timeArrivePoint)
            TimeTechLeavePoint[thisPoint]  =  timeArrivePoint
            TimeTechArrivePoint[thisPoint]  =  timeArrivePoint
            timeOfTech = timeArrivePoint
            if Visited[thisPoint] == 0:
                if j != N_PointOfTechRoute -1 :
                    AfterPointCarrySample[thisPoint] = ListMoveTech[i][j + 1]
                else:
                    AfterPointCarrySample[thisPoint] = 0
        TimeTechArriveDepot[i] = TimeTechLeavePoint[thisPoint] + timeTechMatrix[thisPoint][0]
    N_RoutesOfDrone = len(RoutesDrone)
    timeDrone = 0
    for i in range(N_RoutesOfDrone):
        N_PointOfDroneRoute = len(RoutesDrone[i])
        durationOfDrone = 0
        timeStart = timeDrone
        for j in range(1, N_PointOfDroneRoute):
            thisPoint = RoutesDrone[i][j]
            prePoint = RoutesDrone[i][j - 1]
            timeToPickUp = timeRateDrone*distanceMatrix[prePoint][thisPoint]
            timeToBackDepot = timeRateDrone*distanceMatrix[thisPoint][depot]
            guessDroneTime = timeDrone + timeToPickUp
            if j == N_PointOfDroneRoute - 1:
                AfterPointCarrySample[thisPoint] = depot
            else:
                AfterPointCarrySample[thisPoint] = RoutesDrone[i][j + 1]
            if guessDroneTime < TimeTechLeavePoint[thisPoint]:
                if j != 1:
                    durationOfDrone +=  TimeTechLeavePoint[thisPoint] - timeDrone
                    TimeDroneArrivePoint[i][j] = timeDrone + timeToPickUp
                    timeDrone = TimeTechLeavePoint[thisPoint]
                    TimeDroneLeavePoint[i][j] = timeDrone
                else:
                    # drone will wait at depot to arrive point (thisPoint) in the same with technican
                    durationOfDrone +=  timeToPickUp
                    timeStart = TimeTechLeavePoint[thisPoint] - timeToPickUp
                    TimeDroneArrivePoint[i][j] = timeDrone + timeToPickUp
                    timeDrone = TimeTechLeavePoint[thisPoint]
                    TimeDroneLeavePoint[i][j] = timeDrone
            else:
                durationOfDrone += timeToPickUp
                timeDrone += timeToPickUp
                TimeDroneLeavePoint[i][j] = timeDrone
                TimeDroneArrivePoint[i][j] = timeDrone
                # bonus waiting time for guess time of tech
                checkBreak = False
                for u in range(N_Technican):
                    for v in range(1, len(ListMoveTech[u])):
                        findPoint = ListMoveTech[u][v]
                        if findPoint == thisPoint:
                            if v != len(ListMoveTech[u]) - 1:
                                for k in range(v+1, len(ListMoveTech[u])):
                                    pointAfterThisPoint = ListMoveTech[u][k]
                                    TimeTechLeavePoint[pointAfterThisPoint] += timeDrone - TimeTechLeavePoint[thisPoint]
                                    TimeTechArrivePoint[pointAfterThisPoint] += timeDrone - TimeTechArrivePoint[thisPoint]
                            checkBreak = True
                            break
                    if checkBreak == True:
                        lastPointInTechRoute = ListMoveTech[u][-1]
                        TimeTechArriveDepot[u] = TimeTechLeavePoint[lastPointInTechRoute] + timeTechMatrix[lastPointInTechRoute][0]
                        break
                TimeTechLeavePoint[thisPoint] = timeDrone
        timeDrone += timeDroneMatrix[RoutesDrone[i][N_PointOfDroneRoute - 1]][0]
        TimeDroneArriveDepot[i] = timeDrone
        totalDurationViolation += max(timeDrone - timeStart - edurance, 0)
    # caculate time back to depot of sample
    for i in range(1, totalPoint):
        if AfterPointCarrySample[i] == 0 :
            if Visited[i] == 1:
                TimeSampleInDepot[i] = TimeTechLeavePoint[i] + timeDroneMatrix[i][0]
            else:
                TimeSampleInDepot[i] = TimeTechLeavePoint[i] + timeTechMatrix[i][0]
    for i in range(1, totalPoint):
        if AfterPointCarrySample[i] != 0:
            afterPoint = AfterPointCarrySample[i]
            while AfterPointCarrySample[afterPoint] != 0:
                afterPoint = AfterPointCarrySample[afterPoint] 
            TimeSampleInDepot[i] = TimeSampleInDepot[afterPoint]
    # caculate waiting time of sample
    for i in range(1, totalPoint):
        totalSampleWaitTime += TimeSampleInDepot[i] - TimeTechArrivePoint[i]
    # caculate working time violation
    for i in range(1, totalPoint):
        totalWorkingTimeViolation += max(TimeSampleInDepot[i] - D,0)
    # for i in range(N_Technican_temp):
    #     totalWorkingTimeViolation += max(TimeTechArriveDepot[i] - D, 0)
    # if N_RoutesOfDrone != 0:
    #     totalWorkingTimeViolation += max(TimeDroneArriveDepot[-1] - D, 0)
    fitness = 0
    fitness += totalSampleWaitTime + alpha1*totalDurationViolation + alpha2*totalWorkingTimeViolation
    # update alpha beta
    
    if totalDurationViolation == 0 :
        feasibleWithDroneEdurance = True
    else:
        feasibleWithDroneEdurance = False
    if totalWorkingTimeViolation == 0:
        feasibleWithMaximumWorking = True
    else:
        feasibleWithMaximumWorking = False
    if totalDurationViolation == 0 and totalWorkingTimeViolation == 0:
        feasible = True
    else:
        feasible = False
    return fitness

            
def insertPointInRelocation(ListMoveTech, RoutesDrone, Visited, insertPoint, theRouteInsert, orderInsert, newListMove):
    bestFitnessTemp = fitness(newListMove, RoutesDrone, Visited)
    rs = (RoutesDrone, Visited, bestFitnessTemp)
    # create variable
    totalPoint = 1
    for i in range(len(ListMoveTech)):
        for j in range(1, len(ListMoveTech[i])):
            if ListMoveTech[i][j] != 0:
                totalPoint += 1
    TimeGuessOfTechIF = []
    N_Technican_temp = len(ListMoveTech)
    new_N_Technican = len(newListMove)
    for i in range(N_Technican_temp):
        GuessTime = [0]
        TimeGuessOfTechIF.append(GuessTime)
    TimeTechLeavePoint = np.zeros(totalPoint)
    TimeTechArrivePoint = np.zeros(totalPoint)
    TimeStartOfRoutesIF = np.zeros(len(RoutesDrone))
    TimeDroneArrivePoint = copy.deepcopy(RoutesDrone)
    TimeDroneLeavePoint = copy.deepcopy(RoutesDrone)
    TimeDroneArriveDepot = np.zeros(len(RoutesDrone))
    TimeTechArriveDepot = np.zeros(len(ListMoveTech))
    AfterPointCarrySample = np.zeros(totalPoint, int)
    for i in range(1, totalPoint):
        AfterPointCarrySample[i] = -1
    TimeSampleInDepot = np.zeros(totalPoint)
    totalSampleWaitTime = 0
    totalDurationViolation = 0
    totalWorkingTimeViolation = 0
    # position of point in tech route
    theTechRouteOfPoint = np.zeros(totalPoint, int)
    orderOfPointInTechRoute = np.zeros(totalPoint, int) 
    for i in range(N_Technican_temp):
        for j in range(1, len(ListMoveTech[i])):
            thisPoint = ListMoveTech[i][j]
            theTechRouteOfPoint[thisPoint] = i
            orderOfPointInTechRoute[thisPoint] = j
    # position of customer in drone trip
    TheDroneRouteOfPoint = np.zeros(totalPoint, int)
    OrderOfPointInDroneRoute = np.zeros(totalPoint, int)
    for i in range(totalPoint):
        TheDroneRouteOfPoint[i] = (-1)*(i + 1)
        OrderOfPointInDroneRoute[i] = (-1)*(i + 1)
    for i in range(len(RoutesDrone)):
        for j in range(1, len(RoutesDrone[i])):
            thisPoint = RoutesDrone[i][j]
            TheDroneRouteOfPoint[thisPoint] = i
            OrderOfPointInDroneRoute[thisPoint] = j
    #
    if theRouteInsert == theTechRouteOfPoint[insertPoint]:
        if abs(orderInsert - orderOfPointInTechRoute[insertPoint]) <= 1:
            return rs
    # print(ListMoveTech)
    for i in range(N_Technican_temp):
        N_PointOfTechRoute = len(ListMoveTech[i])
        timeOfTech = 0
        for j in range(1,N_PointOfTechRoute):
            thisPoint = ListMoveTech[i][j]
            prePoint = ListMoveTech[i][j - 1]
            timeArrivePoint = timeOfTech + timeRateTech*distanceMatrix[prePoint][thisPoint]
            TimeGuessOfTechIF[i].append(timeArrivePoint)
            TimeTechLeavePoint[thisPoint]  =  timeArrivePoint
            TimeTechArrivePoint[thisPoint]  =  timeArrivePoint
            timeOfTech = timeArrivePoint
            if Visited[thisPoint] == 0:
                if j != N_PointOfTechRoute -1 :
                    AfterPointCarrySample[thisPoint] = ListMoveTech[i][j + 1]
                else:
                    AfterPointCarrySample[thisPoint] = 0
        TimeTechArriveDepot[i] = TimeTechLeavePoint[thisPoint] + timeTechMatrix[thisPoint][0]
    N_RoutesOfDrone = len(RoutesDrone)
    timeDrone = 0
    for i in range(N_RoutesOfDrone):
        N_PointOfDroneRoute = len(RoutesDrone[i])
        durationOfDrone = 0
        timeStart = timeDrone
        for j in range(1, N_PointOfDroneRoute):
            thisPoint = RoutesDrone[i][j]
            prePoint = RoutesDrone[i][j - 1]
            timeToPickUp = timeRateDrone*distanceMatrix[prePoint][thisPoint]
            timeToBackDepot = timeRateDrone*distanceMatrix[thisPoint][depot]
            guessDroneTime = timeDrone + timeToPickUp
            if j == N_PointOfDroneRoute - 1:
                AfterPointCarrySample[thisPoint] = depot
            else:
                AfterPointCarrySample[thisPoint] = RoutesDrone[i][j + 1]
            if guessDroneTime < TimeTechLeavePoint[thisPoint]:
                if j != 1:
                    durationOfDrone +=  TimeTechLeavePoint[thisPoint] - timeDrone
                    TimeDroneArrivePoint[i][j] = timeDrone + timeToPickUp
                    timeDrone = TimeTechLeavePoint[thisPoint]
                    TimeDroneLeavePoint[i][j] = timeDrone
                else:
                    # drone will wait at depot to arrive point (thisPoint) in the same with technican
                    durationOfDrone +=  timeToPickUp
                    timeStart = TimeTechLeavePoint[thisPoint] - timeToPickUp
                    TimeDroneArrivePoint[i][j] = timeDrone + timeToPickUp
                    timeDrone = TimeTechLeavePoint[thisPoint]
                    TimeDroneLeavePoint[i][j] = timeDrone
            else:
                durationOfDrone += timeToPickUp
                timeDrone += timeToPickUp
                TimeDroneLeavePoint[i][j] = timeDrone
                TimeDroneArrivePoint[i][j] = timeDrone
                # bonus waiting time for guess time of tech
                checkBreak = False
                for u in range(N_Technican_temp):
                    for v in range(1, len(ListMoveTech[u])):
                        findPoint = ListMoveTech[u][v]
                        if findPoint == thisPoint:
                            if v != len(ListMoveTech[u]) - 1:
                                for k in range(v+1, len(ListMoveTech[u])):
                                    pointAfterThisPoint = ListMoveTech[u][k]
                                    TimeTechLeavePoint[pointAfterThisPoint] += timeDrone - TimeTechLeavePoint[thisPoint]
                                    TimeTechArrivePoint[pointAfterThisPoint] += timeDrone - TimeTechArrivePoint[thisPoint]
                            checkBreak = True
                            break
                    if checkBreak == True:
                        lastPointInTechRoute = ListMoveTech[u][-1]
                        TimeTechArriveDepot[u] = TimeTechLeavePoint[lastPointInTechRoute] + timeTechMatrix[lastPointInTechRoute][0]
                        break
                TimeTechLeavePoint[thisPoint] = timeDrone
        timeDrone += timeDroneMatrix[RoutesDrone[i][N_PointOfDroneRoute - 1]][0]
        TimeDroneArriveDepot[i] = timeDrone
        totalDurationViolation += max(timeDrone - timeStart - edurance, 0)
    # # caculate time back to depot of sample
    # for i in range(1, totalPoint):
    #     if AfterPointCarrySample[i] == 0 :
    #         if Visited[i] == 1:
    #             TimeSampleInDepot[i] = TimeTechLeavePoint[i] + timeDroneMatrix[i][0]
    #         else:
    #             TimeSampleInDepot[i] = TimeTechLeavePoint[i] + timeTechMatrix[i][0]
    # for i in range(1, totalPoint):
    #     if AfterPointCarrySample[i] != 0:
    #         afterPoint = AfterPointCarrySample[i]
    #         while AfterPointCarrySample[afterPoint] != 0:
    #             afterPoint = AfterPointCarrySample[afterPoint] 
    #         TimeSampleInDepot[i] = TimeSampleInDepot[afterPoint]
    # caculate extra time and sub by relocate
    if theRouteInsert != theTechRouteOfPoint[insertPoint]:
        newPreOfInsertPoint = ListMoveTech[theRouteInsert][orderInsert - 1]
        if orderInsert >= len(ListMoveTech[theRouteInsert]):
            extraTime = 0
        else:
            newSucOfInsertPoint = ListMoveTech[theRouteInsert][orderInsert]
            extraTime = 0
            extraTime -= timeTechMatrix[newPreOfInsertPoint][newSucOfInsertPoint]
            extraTime += timeTechMatrix[newPreOfInsertPoint][insertPoint]
            extraTime += timeTechMatrix[insertPoint][newSucOfInsertPoint]

        subTime = 0
        oldRoute = theTechRouteOfPoint[insertPoint]
        oldOrder = orderOfPointInTechRoute[insertPoint]
        if oldOrder < len(ListMoveTech[oldRoute]) - 1:
            oldPreOfInsertPoint = ListMoveTech[oldRoute][oldOrder - 1]
            oldSucOfInsertPoint = ListMoveTech[oldRoute][oldOrder + 1]
            subTime += timeTechMatrix[oldPreOfInsertPoint][insertPoint]
            subTime += timeTechMatrix[insertPoint][oldSucOfInsertPoint]
            subTime -= timeTechMatrix[oldPreOfInsertPoint][oldSucOfInsertPoint]
    else:
        newPreOfInsertPoint = ListMoveTech[theRouteInsert][orderInsert - 1]
        if orderInsert >= len(ListMoveTech[theRouteInsert]) - 1:
            extraTime = 0
        else:
            newSucOfInsertPoint = ListMoveTech[theRouteInsert][orderInsert]
            extraTime = 0
            extraTime -= timeTechMatrix[newPreOfInsertPoint][newSucOfInsertPoint]
            extraTime += timeTechMatrix[newPreOfInsertPoint][insertPoint]
            extraTime += timeTechMatrix[insertPoint][newSucOfInsertPoint]

        subTime = 0
        oldRoute = theTechRouteOfPoint[insertPoint]
        oldOrder = orderOfPointInTechRoute[insertPoint]
        if oldOrder < len(ListMoveTech[oldRoute]) - 1:
            oldPreOfInsertPoint = ListMoveTech[oldRoute][oldOrder - 1]
            oldSucOfInsertPoint = ListMoveTech[oldRoute][oldOrder + 1]
            subTime += timeTechMatrix[oldPreOfInsertPoint][insertPoint]
            subTime += timeTechMatrix[insertPoint][oldSucOfInsertPoint]
            subTime -= timeTechMatrix[oldPreOfInsertPoint][oldSucOfInsertPoint]
    #
    listPointExtraAndSubTime = np.zeros(totalPoint, float)
    if extraTime != 0:
        for i in range(orderInsert, len(ListMoveTech[theRouteInsert])):
            pointWillExtraTime = ListMoveTech[theRouteInsert][i]
            listPointExtraAndSubTime[pointWillExtraTime] += extraTime
            TimeTechLeavePoint[pointWillExtraTime] += extraTime
    if subTime != 0:
        for i in range(oldOrder + 1, len(ListMoveTech[oldRoute]) ):
            pointWillSubTime = ListMoveTech[oldRoute][i]
            listPointExtraAndSubTime[pointWillSubTime] -= subTime
            TimeTechLeavePoint[pointWillSubTime] -= subTime
    if extraTime == 0 and subTime == 0:
        return rs
    for i in range(1, totalPoint):
        if listPointExtraAndSubTime[i] > 0:
            listPointExtraAndSubTime[i] = 1
        elif listPointExtraAndSubTime[i] < 0:
            listPointExtraAndSubTime[i] = -1
    # update with new ListMove and RoutesDrone
    # position of point in tech route
    theTechRouteOfPoint = np.zeros(totalPoint, int)
    orderOfPointInTechRoute = np.zeros(totalPoint, int) 
    # print(newListMove)
    for i in range(new_N_Technican):
        for j in range(1, len(newListMove[i])):
            thisPoint = newListMove[i][j]
            theTechRouteOfPoint[thisPoint] = i
            orderOfPointInTechRoute[thisPoint] = j
    # # position of customer in drone trip
    # TheDroneRouteOfPoint = np.zeros(totalPoint, int)
    # OrderOfPointInDroneRoute = np.zeros(totalPoint, int)
    # for i in range(totalPoint):
    #     TheDroneRouteOfPoint[i] = (-1)*(i + 1)
    #     OrderOfPointInDroneRoute[i] = (-1)*(i + 1)
    # for i in range(len(newRoutesDrone)):
    #     for j in range(1, len(newRoutesDrone[i])):
    #         thisPoint = newRoutesDrone[i][j]
    #         TheDroneRouteOfPoint[thisPoint] = i
    #         OrderOfPointInDroneRoute[thisPoint] = j
    prePointOfInsertPoint = ListMoveTech[theRouteInsert][orderInsert - 1]
    TimeTechLeavePoint[insertPoint] = TimeTechLeavePoint[prePointOfInsertPoint] + timeTechMatrix[prePointOfInsertPoint][insertPoint]
    currentFoundPoint = depot
    freeTime = []
    freePoint = []
    maxOrderInTechRouteOfFrontPart = np.zeros(new_N_Technican, int)
    minOrderInTechRoutOfBehindPart = np.ones(new_N_Technican, int)
    for i in range(new_N_Technican):
        minOrderInTechRoutOfBehindPart[i] = 100000
    foundFreeTime = False
    for i in range(len(RoutesDrone)):
        routeOfDrone = RoutesDrone[i]
        for j in range(1, len(routeOfDrone)):
            foundPoint = RoutesDrone[i][j]
            # variable for check insertable
            if foundFreeTime == False:
                techRouteOfCurrentPoint = theTechRouteOfPoint[currentFoundPoint]
                orderInRouteOfCurrentPoint = orderOfPointInTechRoute[currentFoundPoint]
                maxOrderInTechRouteOfFrontPart[techRouteOfCurrentPoint] = orderInRouteOfCurrentPoint
                if listPointExtraAndSubTime[currentFoundPoint] - listPointExtraAndSubTime[foundPoint] < 0:
                    # if current found point is sub time or not extra time and found point is extra time
                    # it will be create free time of drone
                    # try to insert a point to this free time
                    freeTime = [TimeTechLeavePoint[currentFoundPoint], TimeTechLeavePoint[foundPoint]]
                    freePoint = [currentFoundPoint, foundPoint]
                    foundFreeTime = True
            if foundFreeTime == True:
                techRouteOfFoundPoint = theTechRouteOfPoint[foundPoint]
                orderInRouteOfFoundPoint = orderOfPointInTechRoute[foundPoint]
                if minOrderInTechRoutOfBehindPart[techRouteOfFoundPoint] > orderInRouteOfFoundPoint:
                    minOrderInTechRoutOfBehindPart[techRouteOfFoundPoint] = orderInRouteOfFoundPoint
            currentFoundPoint = foundPoint
        # if foundFreeTime == True:
        #     break
    # for i in range(N_Technican_temp):
    #     if minOrderInTechRoutOfBehindPart[i] == 100000:
    #         minOrderInTechRoutOfBehindPart[i] = 0
    if len(freeTime) == 0:
        return rs

    for i in range(1, totalPoint):
        if Visited[i] == 0:
            if TimeTechLeavePoint[i] > freeTime[0] and TimeTechLeavePoint[i] < freeTime[1]:
                if TheDroneRouteOfPoint[freePoint[0]] == TheDroneRouteOfPoint[freePoint[1]] or freePoint[0] == 0:
                    if freePoint[0] != 0:
                        insertDroneRoute = TheDroneRouteOfPoint[freePoint[0]]
                    else:
                        insertDroneRoute = 0
                    theTechRouteOfInsertDronePoint = theTechRouteOfPoint[i]
                    orderInTechRouteOfInsertDronePoint = orderOfPointInTechRoute[i]
                    insertable = True
                    for j in range(1, len(RoutesDrone[insertDroneRoute])):
                        if theTechRouteOfInsertDronePoint == theTechRouteOfPoint[RoutesDrone[insertDroneRoute][j]]:
                            insertable = False
                            break
                    if orderInTechRouteOfInsertDronePoint < maxOrderInTechRouteOfFrontPart[theTechRouteOfInsertDronePoint]:
                        insertable = False
                    if orderInTechRouteOfInsertDronePoint > minOrderInTechRoutOfBehindPart[theTechRouteOfInsertDronePoint]:
                        insertable = False
                    if insertable ==  False:
                        continue
                    #
                    copyRoutesOfDrone = copy.deepcopy(RoutesDrone)
                    copyVisitedPoint = copy.deepcopy(Visited)
                    copyRoutesOfDrone[insertDroneRoute].insert(OrderOfPointInDroneRoute[freePoint[1]], i)
                    copyVisitedPoint[i] = 1
                    tmpFitness = fitness(newListMove, copyRoutesOfDrone, copyVisitedPoint)
                    if tmpFitness - bestFitnessTemp < (-1)*0.00001:
                        bestFitnessTemp = tmpFitness
                        rs = (copyRoutesOfDrone, copyVisitedPoint, bestFitnessTemp)
                else:
                    theTechRouteOfInsertDronePoint = theTechRouteOfPoint[i]
                    orderInTechRouteOfInsertDronePoint = orderOfPointInTechRoute[i]
                    insertable = True
                    if orderInTechRouteOfInsertDronePoint <= maxOrderInTechRouteOfFrontPart[theTechRouteOfInsertDronePoint]:
                        insertable = False
                        # print(i)
                        # print('abcccccccccccccccccccccccc')
                    if orderInTechRouteOfInsertDronePoint >= minOrderInTechRoutOfBehindPart[theTechRouteOfInsertDronePoint]:
                        insertable = False
                        # print(i)
                        # print('abdddddddddddddddddddddddd')
                    if insertable == False:
                        continue
                    # 3 case can appear
                    # create new route with only customer i 
                    # i is insert after freePoint[0]
                    # i is insert behind freePoint[1]
                    # case 1
                    newTempDroneRoute = [depot, i]
                    copyRoutesOfDrone = copy.deepcopy(RoutesDrone)
                    copyVisitedPoint = copy.deepcopy(Visited)
                    copyRoutesOfDrone.insert(TheDroneRouteOfPoint[freePoint[1]], newTempDroneRoute)
                    copyVisitedPoint[i] = 1
                    tmpFitness = fitness(newListMove, copyRoutesOfDrone, copyVisitedPoint) 
                    if tmpFitness - bestFitnessTemp < (-1)*0.00001:
                        bestFitnessTemp = tmpFitness
                        rs = (copyRoutesOfDrone, copyVisitedPoint, bestFitnessTemp)
                        # print('abdddddddddddddddddddddddd')
                    # case 2 
                    orderOfFirstFreePoint = OrderOfPointInDroneRoute[freePoint[0]]
                    insertDroneRoute0 = TheDroneRouteOfPoint[freePoint[0]]
                    insertFront = True
                    for j in range(1, len(RoutesDrone[insertDroneRoute0])):
                        pointInFrontRoute = RoutesDrone[insertDroneRoute0][j]
                        if theTechRouteOfPoint[i] == theTechRouteOfPoint[pointInFrontRoute]:
                            insertFront = False
                    if insertFront:
                        copyRoutesOfDrone = copy.deepcopy(RoutesDrone)
                        copyVisitedPoint = copy.deepcopy(Visited)
                        copyRoutesOfDrone[insertDroneRoute0].insert(orderOfFirstFreePoint + 1, i)
                        copyVisitedPoint[i] = 1
                        tmpFitness = fitness(newListMove, copyRoutesOfDrone, copyVisitedPoint)
                        if tmpFitness - bestFitnessTemp < (-1)*0.00001:
                            bestFitnessTemp = tmpFitness
                            rs = (copyRoutesOfDrone, copyVisitedPoint, bestFitnessTemp)
                    #case 3 
                    insertBehind = True
                    orderOfSecondFreePoint = OrderOfPointInDroneRoute[freePoint[1]]
                    insertDroneRoute1 = TheDroneRouteOfPoint[freePoint[1]]
                    for j in range(1, len(RoutesDrone[insertDroneRoute1])):
                        pointInBehindRoute = RoutesDrone[insertDroneRoute1][j]
                        if theTechRouteOfPoint[i] == theTechRouteOfPoint[pointInBehindRoute]:
                            insertBehind = False
                    if insertBehind:
                        copyRoutesOfDrone = copy.deepcopy(RoutesDrone)
                        copyVisitedPoint = copy.deepcopy(Visited)
                        copyRoutesOfDrone[insertDroneRoute1].insert(orderOfSecondFreePoint, i)
                        copyVisitedPoint[i] = 1
                        tmpFitness = fitness(newListMove, copyRoutesOfDrone, copyVisitedPoint)
                        if tmpFitness - bestFitnessTemp < (-1)*0.00001:
                            bestFitnessTemp = tmpFitness
                            rs = (copyRoutesOfDrone, copyVisitedPoint, bestFitnessTemp)
    return rs

def deleteEmptyRoute(allRoutes):
    routeIndex = 0
    while routeIndex != len(allRoutes):
        if len(allRoutes[routeIndex]) <= 1:
            allRoutes.remove(allRoutes[routeIndex])
        else:
            routeIndex += 1
    return allRoutes

print(dict_matched_point)
# Multilevel
for level in range(total_level - 1, -1, -1):
    # inherit from previous level
    ListMoveOfTech = []
    allRouteOfDrones = []
    totalPoint = dict_total_point[level]
    visited = np.zeros(totalPoint, int)
    list_matched_point = dict_matched_point[level + 1]
    for i in range(len(previous_tech_routes)):
        tmp_routes = [0]
        for j in range(1, len(previous_tech_routes[i])):
            previous_point  = previous_tech_routes[i][j]
            try:
                for matched_point in list_matched_point[previous_point]:
                    tmp_routes.append(matched_point)
            except:
                tmp_routes.append(list_matched_point[previous_point])
        ListMoveOfTech.append(tmp_routes)        
    allRouteOfDrones = copy.deepcopy(previous_drone_routes)
    for i in range(len(previous_drone_routes)):
        for j in range(1, len(previous_drone_routes[i])):
            point = previous_drone_routes[i][j]
            visited[point] = 1
    pass
    # change value of data level by level
    distanceMatrix = copy.deepcopy(dict_distance_matrix[level])
    timeTechMatrix = copy.deepcopy(dict_time_tech_matrix[level])
    timeDroneMatrix = copy.deepcopy(dict_time_drone_matrix[level])
    # Refine current level
    # parameter and variable for tabu search
    numberOfCus = totalPoint - 1
    a1 = numberOfCus*N_Technican/5
    a2 = numberOfCus*N_Technican/10
    a3 = 3/(len(cusInDroneFlyRange)*D/edurance)
    a4 = 5/(len(cusInDroneFlyRange)*D/edurance)
    alpha1 = 1
    alpha2 = 1
    beta = 0.5
    depot = 0 # repair something with 0 by depot
    timeRateTech = 1/0.58
    timeRateDrone = 1/0.83
    ITmax = 50 # if the current best solution is not improved after IT max just break the loop 
    r = 1 #parameter define probability distribution of neighborhoods
    deltaR = 0.4
    intervalRoutingMove = [int(numberOfCus*N_Technican/a1), int(numberOfCus*N_Technican/a2)]
    intervalSamplingMove = [int(numCusInDroneRange*D/edurance*a3), int(numCusInDroneRange*D/edurance*a4)]
    bestFitness = 100000000000
    tabuList = [[], [], [], [], []]
    iterationDestroyTabu = [[], [], [], [], []]
    tabuStatus = []
    feasible = False
    iterationIndex = 1
    ITnotImproved = 0
    ITnotImprovedWithITr = 0
    totalIT = 0
    bestIT = 0
    numIT = 300
    ITr = 5 
    while True:
        if iterationIndex > 300:
            ITmax = 0
        elif iterationIndex > 200:
            ITmax = 20
        elif iterationIndex > 100:
            ITmax = 30
        # reset alpha 
        if alpha1 > 1000 or alpha1 < 0.001:
            alpha1 = 1
        if alpha2 > 1000 or alpha2 < 0.001:
            alpha2 = 1
        # currentFitness = bestLocalFitness
        nameOfNeighbor = ""
        totalIT += 1
        bestLocalSolution = []
        bestLocalFitness = 10000000000000
        improvedSolution = False
        localFeasible = True
        N_Technican = len(ListMoveOfTech)
        # remove tabu status
        for i in range(len(iterationDestroyTabu)):
            j = 0
            while j != len(iterationDestroyTabu[i]):
                if iterationDestroyTabu[i][j] == iterationIndex:
                    iterationDestroyTabu[i].remove(iterationDestroyTabu[i][j])
                    tabuList[i].remove(tabuList[i][j])
                else:
                    j += 1

        # position of customer in drone trip
        for i in range(totalPoint):
            TheDroneRouteOfPoint[i] = (-1)*(i + 1)
            OrderOfPointInDroneRoute[i] = (-1)*(i + 1)
        for i in range(len(allRouteOfDrones)):
            for j in range(1, len(allRouteOfDrones[i])):
                thisPoint = allRouteOfDrones[i][j]
                TheDroneRouteOfPoint[thisPoint] = i
                OrderOfPointInDroneRoute[thisPoint] = j
        # select neighborhood 
        routingProbability = r/(1+4*r)
        samplingProbability = 1/(1+4*r)
        randomProba = random.random()
        wheel = []
        selectProbability = 0
        selectedNeighbor = 0
        for  i in range(3):
            selectProbability += routingProbability
            wheel.append(selectProbability)
        for i in range(1):
            selectProbability += samplingProbability
            wheel.append(selectProbability)
        for i in range(1):
            selectProbability += routingProbability
            wheel.append(selectProbability)
        for i in range(5):
            if randomProba < wheel[i]:
                selectedNeighbor = i + 1
                break
        # selectedNeighbor = 2
        if selectedNeighbor == 1:
            #relocation move
            nameOfNeighbor = "relocation"
            bestLocalSolution = [ListMoveOfTech, allRouteOfDrones, visited]
            for i in range(N_Technican):
                numberPointOfRoute1 = len(ListMoveOfTech[i])
                for j in range(1, numberPointOfRoute1):
                    pointPreRelocation = ListMoveOfTech[i][j]
                    N_Technican = len(ListMoveOfTech)
                    for u in range(N_Technican):
                        numberPointOfRoute2 = len(ListMoveOfTech[u])
                        for k in range(1, numberPointOfRoute2):
                            #create copy
                            copyListMove = copy.deepcopy(ListMoveOfTech)
                            #                     
                            copyListMove[i].remove(pointPreRelocation)
                            copyRoutesOfDrone = copy.deepcopy(allRouteOfDrones)
                            copyVisitedPoint = copy.deepcopy(visited)
                            #
                            pointSucRelocation = ListMoveOfTech[u][k]
                            if pointPreRelocation == pointSucRelocation or(j == k + 1 and u == i):
                                continue
                            
                            #if pre and succ in the same route
                            if i == u:
                                if k > j:
                                    copyListMove[u].insert(k, pointPreRelocation)
                                    orderInsert = k
                                else:
                                    copyListMove[u].insert(k+1, pointPreRelocation)
                                    orderInsert = k + 1
                            #if pre and succ in the different route
                            else:
                                copyListMove[u].insert(k+1, pointPreRelocation) 
                                orderInsert = k + 1
                            # if len(copyListMove[u]) == 1 or len(copyListMove[i]) == 1:
                            #     continue
                            if len(copyListMove[i]) == 1:
                                copyListMove.remove(copyListMove[i])
                            # move on drone trip
                            if visited[pointPreRelocation] == 1:
                                changingDroneTrip = random.randrange(1,4)
                                for changingDroneTrip in range(1,4):
                                    # remove x in drone trip
                                    if changingDroneTrip == 1:
                                        copyRoutesOfDrone = copy.deepcopy(allRouteOfDrones)
                                        copyVisitedPoint = copy.deepcopy(visited)
                                        checkBreaktion = False
                                        for t in range(len(copyRoutesOfDrone)):
                                            for v in range(len(copyRoutesOfDrone[t])):
                                                findPoint = copyRoutesOfDrone[t][v]
                                                if findPoint == pointPreRelocation:
                                                    if len(copyRoutesOfDrone[t]) <= 2:
                                                        copyRoutesOfDrone.remove(copyRoutesOfDrone[t])
                                                        checkBreaktion  = True
                                                        copyVisitedPoint[pointPreRelocation] = 0 
                                                        break
                                                    else:
                                                        copyRoutesOfDrone[t].remove(findPoint)
                                                        checkBreaktion  = True
                                                        copyVisitedPoint[pointPreRelocation] = 0
                                                        break
                                            if checkBreaktion == True:
                                                break
                                        #
                                        fitnessLocal = fitness(copyListMove, copyRoutesOfDrone, copyVisitedPoint)
                                        tmpTabuStatus = pointPreRelocation
                                        if fitnessLocal - bestLocalFitness  < (-1)*0.00001 and ( (feasible == True and fitnessLocal - bestFitness < (-1)*0.00001) or (tmpTabuStatus not in tabuList[selectedNeighbor - 1]) ):
                                            bestLocalFitness = fitnessLocal
                                            bestLocalSolution = [copyListMove, copyRoutesOfDrone, copyVisitedPoint]
                                            bestTabuStatus = pointPreRelocation
                                            printSolution = [pointPreRelocation, pointSucRelocation]
                                            localFeasible = feasible
                                    # denote x' is direct predecessor of x in technical route, try replace x by x'
                                    elif changingDroneTrip == 2:
                                        copyRoutesOfDrone = copy.deepcopy(allRouteOfDrones)
                                        copyVisitedPoint = copy.deepcopy(visited)
                                        # if x doesn't have predecessor just remove x in drone trip
                                        if j == 1:
                                            checkBreaktion = False
                                            for t in range(len(copyRoutesOfDrone)):
                                                for v in range(len(copyRoutesOfDrone[t])):
                                                    findPoint = copyRoutesOfDrone[t][v]
                                                    if findPoint == pointPreRelocation:
                                                        if len(copyRoutesOfDrone[t]) <= 2:
                                                            copyRoutesOfDrone.remove(copyRoutesOfDrone[t])
                                                            checkBreaktion  = True
                                                            copyVisitedPoint[pointPreRelocation] = 0
                                                            break
                                                        else:
                                                            copyRoutesOfDrone[t].remove(findPoint)
                                                            checkBreaktion  = True
                                                            copyVisitedPoint[pointPreRelocation] = 0
                                                            break
                                                if checkBreaktion == True:
                                                    break
                                        # try to replace x by x'   
                                        else:
                                            directPrePointInTechRoute = ListMoveOfTech[i][j - 1]
                                            # if x' is not visited and can reach by drone
                                            if copyVisitedPoint[directPrePointInTechRoute] == 0 and directPrePointInTechRoute in cusInDroneFlyRange :
                                                checkBreaktion = False
                                                for t in range(len(copyRoutesOfDrone)):
                                                    for v in range(len(copyRoutesOfDrone[t])):
                                                        findPoint = copyRoutesOfDrone[t][v]
                                                        if findPoint == pointPreRelocation:
                                                            copyRoutesOfDrone[t][v] = directPrePointInTechRoute
                                                            checkBreaktion
                                                            copyVisitedPoint[pointPreRelocation] = 0
                                                            copyVisitedPoint[directPrePointInTechRoute] = 1
                                                            break
                                                    if checkBreaktion == True:
                                                        break
                                            # if x' is visited just remove x 
                                            else:
                                                checkBreaktion = False
                                                for t in range(len(copyRoutesOfDrone)):
                                                    for v in range(len(copyRoutesOfDrone[t])):
                                                        findPoint = copyRoutesOfDrone[t][v]
                                                        if findPoint == pointPreRelocation:
                                                            if len(copyRoutesOfDrone[t]) <= 2:
                                                                copyRoutesOfDrone.remove(copyRoutesOfDrone[t])
                                                                checkBreaktion  = True
                                                                copyVisitedPoint[pointPreRelocation] = 0
                                                                break
                                                            else:
                                                                copyRoutesOfDrone[t].remove(findPoint)
                                                                checkBreaktion  = True
                                                                copyVisitedPoint[pointPreRelocation] = 0
                                                                break
                                                    if checkBreaktion == True:
                                                        break  
                                        #
                                        fitnessLocal = fitness(copyListMove, copyRoutesOfDrone, copyVisitedPoint)
                                        tmpTabuStatus = pointPreRelocation
                                        if fitnessLocal - bestLocalFitness  < (-1)*0.00001 and ( (feasible == True and fitnessLocal - bestFitness < (-1)*0.00001) or (tmpTabuStatus not in tabuList[selectedNeighbor - 1]) ):
                                            bestLocalFitness = fitnessLocal
                                            bestLocalSolution = [copyListMove, copyRoutesOfDrone, copyVisitedPoint]
                                            bestTabuStatus = pointPreRelocation
                                            printSolution = [pointPreRelocation, pointSucRelocation]
                                            localFeasible = feasible
                                    # x'' denote the direct successor of x in technical route
                                    # try to replace x by x''
                                    else:
                                        copyRoutesOfDrone = copy.deepcopy(allRouteOfDrones)
                                        copyVisitedPoint = copy.deepcopy(visited)
                                        # if x doesn't have the successor, just remove x in drone trip
                                        if j == numberPointOfRoute1 - 1:
                                            checkBreaktion = False
                                            for t in range(len(copyRoutesOfDrone)):
                                                for v in range(len(copyRoutesOfDrone[t])):
                                                    findPoint = copyRoutesOfDrone[t][v]
                                                    if findPoint == pointPreRelocation:
                                                        if len(copyRoutesOfDrone[t]) <= 2:
                                                            copyRoutesOfDrone.remove(copyRoutesOfDrone[t])
                                                            checkBreaktion  = True
                                                            copyVisitedPoint[pointPreRelocation] = 0
                                                            break
                                                        else:
                                                            copyRoutesOfDrone[t].remove(findPoint)
                                                            checkBreaktion  = True
                                                            copyVisitedPoint[pointPreRelocation] = 0
                                                            break
                                                if checkBreaktion == True:
                                                    break   
                                        # try to replace x by x''
                                        else:
                                            directSuccPointInTechRoute = ListMoveOfTech[i][j + 1]
                                            # if x'' is not visited and can reach by drone
                                            if copyVisitedPoint[directSuccPointInTechRoute] == 0 and directSuccPointInTechRoute in cusInDroneFlyRange:
                                                checkBreaktion = False
                                                for t in range(len(copyRoutesOfDrone)):
                                                    for v in range(len(copyRoutesOfDrone[t])):
                                                        findPoint = copyRoutesOfDrone[t][v]
                                                        if findPoint == pointPreRelocation:
                                                            copyRoutesOfDrone[t][v] = directSuccPointInTechRoute
                                                            checkBreaktion
                                                            copyVisitedPoint[pointPreRelocation] = 0
                                                            copyVisitedPoint[directSuccPointInTechRoute] = 1
                                                            break
                                                    if checkBreaktion == True:
                                                        break
                                            else:
                                                tripIndex = TheDroneRouteOfPoint[pointPreRelocation]
                                                if tripIndex != -1:
                                                    if len(copyRoutesOfDrone[tripIndex]) <= 2:
                                                        copyRoutesOfDrone.remove(copyRoutesOfDrone[tripIndex])
                                                        copyVisitedPoint[pointPreRelocation] = 0
                                                    else:
                                                        copyRoutesOfDrone[tripIndex].remove(pointPreRelocation)
                                                        copyVisitedPoint[pointPreRelocation] = 0   
                                        #
                                        fitnessLocal = fitness(copyListMove, copyRoutesOfDrone, copyVisitedPoint)
                                        tmpTabuStatus = pointPreRelocation
                                        if fitnessLocal - bestLocalFitness  < (-1)*0.00001 and ( (feasible == True and fitnessLocal - bestFitness < (-1)*0.00001) or (tmpTabuStatus not in tabuList[selectedNeighbor - 1]) ):
                                            bestLocalFitness = fitnessLocal
                                            bestLocalSolution = [copyListMove, copyRoutesOfDrone, copyVisitedPoint]
                                            bestTabuStatus = pointPreRelocation
                                            printSolution = [pointPreRelocation, pointSucRelocation]
                                            localFeasible = feasible
                            else:
                                #
                                resultTemp = insertPointInRelocation(ListMoveOfTech, copyRoutesOfDrone, copyVisitedPoint, pointPreRelocation, u, orderInsert, copyListMove)
                                if resultTemp != (0, 0, 0):
                                    (copyRoutesOfDrone, copyVisitedPoint, fitnessLocal) = resultTemp
                                tmpTabuStatus = pointPreRelocation
                                if fitnessLocal - bestLocalFitness  < (-1)*0.00001 and ( (feasible == True and fitnessLocal - bestFitness < (-1)*0.00001) or (tmpTabuStatus not in tabuList[selectedNeighbor - 1]) ):
                                    bestLocalFitness = fitnessLocal
                                    bestLocalSolution = [copyListMove, copyRoutesOfDrone, copyVisitedPoint]
                                    bestTabuStatus = pointPreRelocation
                                    printSolution = [pointPreRelocation, pointSucRelocation]
                                    localFeasible = feasible
                    # if len(ListMoveOfTech[i]) != 2 and N_Technican < N_Technican_max:
                    #     # try to creat new route for new technican
                    #     # create copy
                    #     copyListMove = copy.deepcopy(ListMoveOfTech)
                    #     copyListMove[i].remove(pointPreRelocation)
                    #     copyRoutesOfDrone = copy.deepcopy(allRouteOfDrones)
                    #     copyVisitedPoint = copy.deepcopy(visited)
                    #     #
                    #     newRoute = [0, pointPreRelocation]
                    #     copyListMove.insert(1, newRoute)
                    #     #
                    #     print(copyListMove)
                    #     fitnessLocal = fitness(copyListMove, copyRoutesOfDrone, copyVisitedPoint)
                    #     tmpTabuStatus = pointPreRelocation
                    #     if fitnessLocal - bestLocalFitness  < (-1)*0.00001 and ( (feasible == True and fitnessLocal - bestFitness < (-1)*0.00001) or (tmpTabuStatus not in tabuList[selectedNeighbor - 1]) ):
                    #         bestLocalFitness = fitnessLocal
                    #         bestLocalSolution = [copyListMove, copyRoutesOfDrone, copyVisitedPoint]
                    #         bestTabuStatus = pointPreRelocation
                    #         printSolution = [pointPreRelocation, pointSucRelocation]
                    #         localFeasible = feasible
                    

            tabuList[selectedNeighbor - 1].append(bestTabuStatus)
            iterationOfTabuStatus = random.randrange(intervalRoutingMove[0], intervalRoutingMove[1])
            iterationDestroyTabu[selectedNeighbor - 1].append( iterationIndex + iterationOfTabuStatus)
            if bestLocalFitness - bestFitness < (-1)*0.00001 and localFeasible == True:
                bestSolution = copy.deepcopy(bestLocalSolution)
                bestFitness = bestLocalFitness
                improvedSolution = True

        elif selectedNeighbor == 2:
            #exchange move
            nameOfNeighbor = "exchange move"
            bestLocalSolution = [ListMoveOfTech, allRouteOfDrones, visited]
            for i in range(N_Technican):
                numberPointOfRoute1 = len(ListMoveOfTech[i])
                for j in range(1, numberPointOfRoute1):
                    pointPreSwap = ListMoveOfTech[i][j]
                    for u in range(N_Technican):
                        numberPointOfRoute2 = len(ListMoveOfTech[u])
                        for k in range(1, numberPointOfRoute2):
                            #create copy
                            copyListMove = copy.deepcopy(ListMoveOfTech)
                            copyTimeVisit = copy.deepcopy(TimeVisitPoint)
                            copyTimeGuess = copy.deepcopy(TimeGuessOfTech)
                            #                     
                            copyRoutesOfDrone = copy.deepcopy(allRouteOfDrones)
                            copyVisitedPoint = copy.deepcopy(visited)
                            #
                            pointSucSwap = ListMoveOfTech[u][k]
                            if pointPreSwap == pointSucSwap:
                                continue
                            # swap 2 point
                            copyListMove[i][j] = pointSucSwap
                            copyListMove[u][k] = pointPreSwap
                            # move on drone trip
                            # Either x or y is visited by drone, but not both
                            # x is visited by drone and y is not
                            if copyVisitedPoint[pointPreSwap] == 1 and copyVisitedPoint[pointSucSwap] == 0:
                                for changingDroneTrip in range(1,3):
                                    copyRoutesOfDrone = copy.deepcopy(allRouteOfDrones)
                                    copyVisitedPoint = copy.deepcopy(visited)
                                    #just remove x from drone trip
                                    if changingDroneTrip == 1:
                                        checkBreaktion = False
                                        for t in range(len(copyRoutesOfDrone)):
                                            for v in range(len(copyRoutesOfDrone[t])):
                                                findPoint = copyRoutesOfDrone[t][v]
                                                if findPoint == pointPreSwap:
                                                    if len(copyRoutesOfDrone[t]) <= 2:
                                                        copyRoutesOfDrone.remove(copyRoutesOfDrone[t])
                                                        checkBreaktion = True
                                                        copyVisitedPoint[findPoint] = 0
                                                        break
                                                    else:
                                                        checkBreaktion = True
                                                        copyVisitedPoint[findPoint] = 0
                                                        copyRoutesOfDrone[t].remove(findPoint)
                                                        break
                                            if checkBreaktion == True:
                                                break
                                        #
                                        fitnessLocal = fitness(copyListMove, copyRoutesOfDrone, copyVisitedPoint)
                                        tmpTabuStatus = [pointSucSwap, pointPreSwap]
                                        tmpTabuStatus1 = [pointPreSwap, pointSucSwap]
                                        if fitnessLocal - bestLocalFitness  < (-1)*0.00001 and ( (feasible == True and fitnessLocal - bestFitness < (-1)*0.00001) or (tmpTabuStatus not in tabuList[selectedNeighbor - 1] and tmpTabuStatus1 not in tabuList[selectedNeighbor - 1]) ):
                                            bestLocalFitness = fitnessLocal
                                            bestLocalSolution = [copyListMove, copyRoutesOfDrone, copyVisitedPoint]
                                            bestTabuStatus = [pointPreSwap, pointSucSwap]
                                            bestTabuStatus1 = [pointSucSwap, pointPreSwap]
                                            printSolution = [pointPreSwap, pointSucSwap]
                                            localFeasible = feasible
                                    # replace x by y on the drone trip
                                    if changingDroneTrip == 2:
                                        checkBreaktion = False
                                        for t in range(len(copyRoutesOfDrone)):
                                            for v in range(len(copyRoutesOfDrone[t])):
                                                findPoint = copyRoutesOfDrone[t][v]
                                                if findPoint == pointPreSwap:
                                                    # if drone can't reach y, just remove x and not replace 
                                                    if pointSucSwap in cusInDroneFlyRange:
                                                        copyRoutesOfDrone[t][v] = pointSucSwap
                                                        copyVisitedPoint[pointPreSwap] = 0
                                                        copyVisitedPoint[pointSucSwap] = 1 
                                                        checkBreaktion = True
                                                    else:
                                                        copyRoutesOfDrone[t].remove(pointPreSwap)
                                                        copyVisitedPoint[pointPreSwap] = 0
                                                    break
                                            if checkBreaktion == True:
                                                break
                                        #
                                        fitnessLocal = fitness(copyListMove, copyRoutesOfDrone, copyVisitedPoint)
                                        tmpTabuStatus = [pointSucSwap, pointPreSwap]
                                        tmpTabuStatus1 = [pointPreSwap, pointSucSwap]
                                        if fitnessLocal - bestLocalFitness  < (-1)*0.00001 and ( (feasible == True and fitnessLocal - bestFitness < (-1)*0.00001) or (tmpTabuStatus not in tabuList[selectedNeighbor - 1] and tmpTabuStatus1 not in tabuList[selectedNeighbor - 1]) ):
                                            bestLocalFitness = fitnessLocal
                                            bestLocalSolution = [copyListMove, copyRoutesOfDrone, copyVisitedPoint]
                                            bestTabuStatus = [pointPreSwap, pointSucSwap]
                                            bestTabuStatus1 = [pointSucSwap, pointPreSwap]
                                            printSolution = [pointPreSwap, pointSucSwap]
                                            localFeasible = feasible
                            # y is visited by drone and x is not
                            elif copyVisitedPoint[pointSucSwap] == 1 and copyVisitedPoint[pointPreSwap] == 0:
                                for changingDroneTrip in range(1,3):
                                    copyRoutesOfDrone = copy.deepcopy(allRouteOfDrones)
                                    copyVisitedPoint = copy.deepcopy(visited)
                                    #just remove y from drone trip
                                    if changingDroneTrip == 1:
                                        checkBreaktion = False
                                        for t in range(len(copyRoutesOfDrone)):
                                            for v in range(len(copyRoutesOfDrone[t])):
                                                findPoint = copyRoutesOfDrone[t][v]
                                                if findPoint == pointSucSwap:
                                                    if len(copyRoutesOfDrone[t]) <= 2:
                                                        copyRoutesOfDrone.remove(copyRoutesOfDrone[t])
                                                        checkBreaktion = True
                                                        copyVisitedPoint[findPoint] = 0
                                                        break
                                                    else:
                                                        checkBreaktion = True
                                                        copyVisitedPoint[findPoint] = 0
                                                        copyRoutesOfDrone[t].remove(findPoint)
                                                        break
                                            if checkBreaktion == True:
                                                break
                                        #
                                        fitnessLocal = fitness(copyListMove, copyRoutesOfDrone, copyVisitedPoint)
                                        tmpTabuStatus = [pointSucSwap, pointPreSwap]
                                        tmpTabuStatus1 = [pointPreSwap, pointSucSwap]
                                        if fitnessLocal - bestLocalFitness  < (-1)*0.00001 and ( (feasible == True and fitnessLocal - bestFitness < (-1)*0.00001) or (tmpTabuStatus not in tabuList[selectedNeighbor - 1] and tmpTabuStatus1 not in tabuList[selectedNeighbor - 1]) ):
                                            bestLocalFitness = fitnessLocal
                                            bestLocalSolution = [copyListMove, copyRoutesOfDrone, copyVisitedPoint]
                                            bestTabuStatus = [pointPreSwap, pointSucSwap]
                                            bestTabuStatus1 = [pointSucSwap, pointPreSwap]
                                            printSolution = [pointPreSwap, pointSucSwap]
                                            localFeasible = feasible
                                    # replace y by x on the drone trip
                                    if changingDroneTrip == 2:
                                        checkBreaktion = False
                                        for t in range(len(copyRoutesOfDrone)):
                                            for v in range(len(copyRoutesOfDrone[t])):
                                                findPoint = copyRoutesOfDrone[t][v]
                                                if findPoint == pointSucSwap:
                                                    # if drone can't reach y, just remove x and not replace 
                                                    if pointPreSwap in cusInDroneFlyRange:
                                                        copyRoutesOfDrone[t][v] = pointPreSwap
                                                        copyVisitedPoint[pointSucSwap] = 0
                                                        copyVisitedPoint[pointPreSwap] = 1
                                                    else:
                                                        copyRoutesOfDrone[t].remove(pointSucSwap)
                                                        copyVisitedPoint[pointSucSwap] = 0
                                                    break
                                            if checkBreaktion == True:
                                                break
                                        #
                                        fitnessLocal = fitness(copyListMove, copyRoutesOfDrone, copyVisitedPoint)
                                        tmpTabuStatus = [pointSucSwap, pointPreSwap]
                                        tmpTabuStatus1 = [pointPreSwap, pointSucSwap]
                                        if fitnessLocal - bestLocalFitness  < (-1)*0.00001 and ( (feasible == True and fitnessLocal - bestFitness < (-1)*0.00001) or (tmpTabuStatus not in tabuList[selectedNeighbor - 1] and tmpTabuStatus1 not in tabuList[selectedNeighbor - 1]) ):
                                            bestLocalFitness = fitnessLocal
                                            bestLocalSolution = [copyListMove, copyRoutesOfDrone, copyVisitedPoint]
                                            bestTabuStatus = [pointPreSwap, pointSucSwap]
                                            bestTabuStatus1 = [pointSucSwap, pointPreSwap]
                                            printSolution = [pointPreSwap, pointSucSwap]
                                            localFeasible = feasible
                            # both x and y is visited by drone
                            elif copyVisitedPoint[pointPreSwap] == 1 and copyVisitedPoint[pointSucSwap] == 1:
                                for changingDroneTrip in range(1,5):
                                    copyRoutesOfDrone = copy.deepcopy(allRouteOfDrones)
                                    copyVisitedPoint = copy.deepcopy(visited)
                                    # swap x and y on the drone route
                                    if changingDroneTrip == 1:
                                        copyRoutesOfDrone[TheDroneRouteOfPoint[pointPreSwap]][OrderOfPointInDroneRoute[pointPreSwap]] = pointSucSwap
                                        copyRoutesOfDrone[TheDroneRouteOfPoint[pointSucSwap]][OrderOfPointInDroneRoute[pointSucSwap]] = pointPreSwap
                                        #
                                        fitnessLocal = fitness(copyListMove, copyRoutesOfDrone, copyVisitedPoint)
                                        tmpTabuStatus = [pointSucSwap, pointPreSwap]
                                        tmpTabuStatus1 = [pointPreSwap, pointSucSwap]
                                        if fitnessLocal - bestLocalFitness  < (-1)*0.00001 and ( (feasible == True and fitnessLocal - bestFitness < (-1)*0.00001) or (tmpTabuStatus not in tabuList[selectedNeighbor - 1] and tmpTabuStatus1 not in tabuList[selectedNeighbor - 1]) ):
                                            bestLocalFitness = fitnessLocal
                                            bestLocalSolution = [copyListMove, copyRoutesOfDrone, copyVisitedPoint]
                                            bestTabuStatus = [pointPreSwap, pointSucSwap]
                                            bestTabuStatus1 = [pointSucSwap, pointPreSwap]
                                            printSolution = [pointPreSwap, pointSucSwap]
                                            localFeasible = feasible
                                    #swap x and y in corresponding drone’s trip and then remove x from drone’s trip;
                                    elif changingDroneTrip == 2:
                                        copyRoutesOfDrone[TheDroneRouteOfPoint[pointPreSwap]][OrderOfPointInDroneRoute[pointPreSwap]] = pointSucSwap
                                        copyRoutesOfDrone[TheDroneRouteOfPoint[pointSucSwap]][OrderOfPointInDroneRoute[pointSucSwap]] = pointPreSwap
                                        if len(copyRoutesOfDrone[TheDroneRouteOfPoint[pointSucSwap]]) <= 2:
                                            copyRoutesOfDrone.remove(copyRoutesOfDrone[TheDroneRouteOfPoint[pointSucSwap]])
                                            copyVisitedPoint[pointPreSwap] = 0
                                        else:
                                            copyRoutesOfDrone[TheDroneRouteOfPoint[pointSucSwap]].remove(pointPreSwap)
                                            copyVisitedPoint[pointPreSwap] = 0
                                        #
                                        fitnessLocal = fitness(copyListMove, copyRoutesOfDrone, copyVisitedPoint)
                                        tmpTabuStatus = [pointSucSwap, pointPreSwap]
                                        tmpTabuStatus1 = [pointPreSwap, pointSucSwap]
                                        if fitnessLocal - bestLocalFitness  < (-1)*0.00001 and ( (feasible == True and fitnessLocal - bestFitness < (-1)*0.00001) or (tmpTabuStatus not in tabuList[selectedNeighbor - 1] and tmpTabuStatus1 not in tabuList[selectedNeighbor - 1]) ):
                                            bestLocalFitness = fitnessLocal
                                            bestLocalSolution = [copyListMove, copyRoutesOfDrone, copyVisitedPoint]
                                            bestTabuStatus = [pointPreSwap, pointSucSwap]
                                            bestTabuStatus1 = [pointSucSwap, pointPreSwap]
                                            printSolution = [pointPreSwap, pointSucSwap]
                                            localFeasible = feasible
                                    #swap x and y in corresponding drone’s trip and then remove y from drone’s trip
                                    elif changingDroneTrip == 3:
                                        copyRoutesOfDrone[TheDroneRouteOfPoint[pointSucSwap]][OrderOfPointInDroneRoute[pointSucSwap]] = pointPreSwap
                                        copyRoutesOfDrone[TheDroneRouteOfPoint[pointPreSwap]][OrderOfPointInDroneRoute[pointPreSwap]] = pointSucSwap
                                        if len(copyRoutesOfDrone[TheDroneRouteOfPoint[pointPreSwap]]) <= 2:
                                            copyRoutesOfDrone.remove(copyRoutesOfDrone[TheDroneRouteOfPoint[pointPreSwap]])
                                            copyVisitedPoint[pointSucSwap] = 0
                                        else:
                                            copyRoutesOfDrone[TheDroneRouteOfPoint[pointPreSwap]].remove(pointSucSwap)
                                            copyVisitedPoint[pointSucSwap] = 0
                                        #
                                        fitnessLocal = fitness(copyListMove, copyRoutesOfDrone, copyVisitedPoint)
                                        tmpTabuStatus = [pointSucSwap, pointPreSwap]
                                        tmpTabuStatus1 = [pointPreSwap, pointSucSwap]
                                        if fitnessLocal - bestLocalFitness  < (-1)*0.00001 and ( (feasible == True and fitnessLocal - bestFitness < (-1)*0.00001) or (tmpTabuStatus not in tabuList[selectedNeighbor - 1] and tmpTabuStatus1 not in tabuList[selectedNeighbor - 1]) ):
                                            bestLocalFitness = fitnessLocal
                                            bestLocalSolution = [copyListMove, copyRoutesOfDrone, copyVisitedPoint]
                                            bestTabuStatus = [pointPreSwap, pointSucSwap]
                                            bestTabuStatus1 = [pointSucSwap, pointPreSwap]
                                            printSolution = [pointPreSwap, pointSucSwap]
                                            localFeasible = feasible
                                    # remove both x and y from corresponding drone’s trips.
                                    else:
                                        copyRoutesOfDrone[TheDroneRouteOfPoint[pointSucSwap]].remove(pointSucSwap)
                                        copyVisitedPoint[pointSucSwap] = 0
                                        copyRoutesOfDrone[TheDroneRouteOfPoint[pointPreSwap]].remove(pointPreSwap)
                                        copyVisitedPoint[pointPreSwap] = 0
                                        tripIndex = 0
                                        while tripIndex != len(copyRoutesOfDrone):
                                            if len(copyRoutesOfDrone[tripIndex]) == 1:
                                                copyRoutesOfDrone.remove(copyRoutesOfDrone[tripIndex])
                                            else:
                                                tripIndex += 1
                                        fitnessLocal = fitness(copyListMove, copyRoutesOfDrone, copyVisitedPoint)
                                        tmpTabuStatus = [pointSucSwap, pointPreSwap]
                                        tmpTabuStatus1 = [pointPreSwap, pointSucSwap]
                                        if fitnessLocal - bestLocalFitness  < (-1)*0.00001 and ( (feasible == True and fitnessLocal - bestFitness < (-1)*0.00001) or (tmpTabuStatus not in tabuList[selectedNeighbor - 1] and tmpTabuStatus1 not in tabuList[selectedNeighbor - 1]) ):
                                            bestLocalFitness = fitnessLocal
                                            bestLocalSolution = [copyListMove, copyRoutesOfDrone, copyVisitedPoint]
                                            bestTabuStatus = [pointPreSwap, pointSucSwap]
                                            bestTabuStatus1 = [pointSucSwap, pointPreSwap]
                                            printSolution = [pointPreSwap, pointSucSwap]
                                            localFeasible = feasible
                            else:
                                fitnessLocal = fitness(copyListMove, copyRoutesOfDrone, copyVisitedPoint)
                                tmpTabuStatus = [pointSucSwap, pointPreSwap]
                                tmpTabuStatus1 = [pointPreSwap, pointSucSwap]
                                if fitnessLocal - bestLocalFitness  < (-1)*0.00001 and ( (feasible == True and fitnessLocal - bestFitness < (-1)*0.00001) or (tmpTabuStatus not in tabuList[selectedNeighbor - 1] and tmpTabuStatus1 not in tabuList[selectedNeighbor - 1]) ):
                                    bestLocalFitness = fitnessLocal
                                    bestLocalSolution = [copyListMove, copyRoutesOfDrone, copyVisitedPoint]
                                    bestTabuStatus = [pointPreSwap, pointSucSwap]
                                    bestTabuStatus1 = [pointSucSwap, pointPreSwap]
                                    printSolution = [pointPreSwap, pointSucSwap]
                                    localFeasible = feasible
            tabuList[selectedNeighbor - 1].append(bestTabuStatus)
            tabuList[selectedNeighbor - 1].append(bestTabuStatus1)
            iterationOfTabuStatus = random.randrange(intervalRoutingMove[0], intervalRoutingMove[1])
            iterationDestroyTabu[selectedNeighbor - 1].append( iterationIndex + iterationOfTabuStatus)
            iterationDestroyTabu[selectedNeighbor - 1].append( iterationIndex + iterationOfTabuStatus)
            if bestLocalFitness - bestFitness < (-1)*0.0001 and localFeasible == True:
                bestSolution = copy.deepcopy(bestLocalSolution)
                bestFitness = bestLocalFitness
                improvedSolution = True
        elif selectedNeighbor == 3:
            nameOfNeighbor = "2 opt"
            bestLocalSolution = [ListMoveOfTech, allRouteOfDrones, visited]
            excuted = False
            twoPointInSameRoute = True
            # emanating (x x') and (y y')  then add (x y) and (x' y')
            for i in range(N_Technican):
                numberPointOfTechRoute = len(ListMoveOfTech[i])
                #find x
                for j in range(1, numberPointOfTechRoute - 1):
                    firstPoint = ListMoveOfTech[i][j]
                    succPointOfFirstPoint = ListMoveOfTech[i][j + 1]
                    if numberPointOfTechRoute - j >= 4:
                        firstPoint = ListMoveOfTech[i][j]
                        succPointOfFirstPoint = ListMoveOfTech[i][j + 1]
                        for k in range(j + 2, numberPointOfTechRoute - 1):
                            secondPoint = ListMoveOfTech[i][k]
                            succPointOfSecondPoint = ListMoveOfTech[i][k + 1]
                            numberPointBetween = k - j
                            #create copy
                            copyListMove = copy.deepcopy(ListMoveOfTech)
                            copyTimeVisit = copy.deepcopy(TimeVisitPoint)
                            copyTimeGuess = copy.deepcopy(TimeGuessOfTech)
                            copyRoutesOfDrone = copy.deepcopy(allRouteOfDrones)
                            copyVisitedPoint = copy.deepcopy(visited)
                            # swap every point betwwen x' and y
                            for u in range(int(numberPointBetween/2)):
                                tmp  = copyListMove[i][j + 1 + u]
                                copyListMove[i][j + 1 + u] = copyListMove[i][k - u]
                                copyListMove[i][k - u] = tmp
                            # in drone trip    
                            changingDroneTrip = random.randrange(1,3)
                            for changingDroneTrip in range(1,3):
                                copyRoutesOfDrone = copy.deepcopy(allRouteOfDrones)
                                copyVisitedPoint = copy.deepcopy(visited)
                                #remove all customers in this segment from the drone’s trips
                                if changingDroneTrip == 1:
                                    for u in range(numberPointBetween):
                                        deletePoint = ListMoveOfTech[i][j + 1 +u]
                                        if visited[deletePoint] == 1:
                                            RouteOfDelPoint = TheDroneRouteOfPoint[deletePoint]

                                            copyRoutesOfDrone[RouteOfDelPoint].remove(deletePoint)
                                            copyVisitedPoint[deletePoint] = 0
                                    #
                                    copyRoutesOfDrone = deleteEmptyRoute(copyRoutesOfDrone)
                                    fitnessLocal = fitness(copyListMove, copyRoutesOfDrone, copyVisitedPoint)
                                    tmpTabuStatus = [firstPoint, secondPoint]
                                    if fitnessLocal - bestLocalFitness  < (-1)*0.00001 and ( (feasible == True and fitnessLocal - bestFitness < (-1)*0.00001) or (tmpTabuStatus not in tabuList[selectedNeighbor - 1]) ):
                                        bestLocalFitness = fitnessLocal
                                        bestLocalSolution = [copyListMove, copyRoutesOfDrone, copyVisitedPoint]
                                        bestTabuStatus = [firstPoint, succPointOfFirstPoint]
                                        excuted = True
                                        printSolution = [firstPoint, secondPoint]
                                        twoPointInSameRoute = True
                                        localFeasible = feasible
                                #swap customers in this segment between drone’s trips
                                elif changingDroneTrip == 2:
                                    for u in range(int(numberPointBetween/2)):
                                        preSwapPoint = copyListMove[i][j + 1 + u]
                                        droneTripOfPrePoint = TheDroneRouteOfPoint[preSwapPoint]
                                        indexOfPrePointInDroneTrip = OrderOfPointInDroneRoute[preSwapPoint]
                                        succSwapPoint = copyListMove[i][k - u]
                                        droneTripOfSuccPoint = TheDroneRouteOfPoint[succSwapPoint]
                                        indexOfSuccPointInDroneTrip = OrderOfPointInDroneRoute[succSwapPoint]
                                        if visited[preSwapPoint] == 1 and visited[succSwapPoint] == 1:
                                            copyRoutesOfDrone[droneTripOfPrePoint][indexOfPrePointInDroneTrip] = succSwapPoint
                                            copyRoutesOfDrone[droneTripOfSuccPoint][indexOfSuccPointInDroneTrip] = preSwapPoint
                                        elif visited[preSwapPoint] == 0 and visited[succSwapPoint] == 1:
                                            if preSwapPoint in cusInDroneFlyRange:
                                                copyRoutesOfDrone[droneTripOfSuccPoint][indexOfSuccPointInDroneTrip] = preSwapPoint
                                                copyVisitedPoint[succSwapPoint] = 0
                                                copyVisitedPoint[preSwapPoint] = 1
                                            else:
                                                copyRoutesOfDrone[droneTripOfSuccPoint].remove(succSwapPoint)
                                                copyVisitedPoint[succSwapPoint] = 0
                                        elif visited[preSwapPoint] == 1 and visited[succSwapPoint] == 0:
                                            if succSwapPoint in cusInDroneFlyRange:
                                                copyRoutesOfDrone[droneTripOfPrePoint][indexOfPrePointInDroneTrip] = succSwapPoint
                                                copyVisitedPoint[succSwapPoint] = 1
                                                copyVisitedPoint[preSwapPoint] = 0
                                            else:
                                                copyRoutesOfDrone[droneTripOfPrePoint].remove(preSwapPoint)
                                                copyVisitedPoint[preSwapPoint] = 0
                                    #
                                    fitnessLocal = fitness(copyListMove, copyRoutesOfDrone, copyVisitedPoint)
                                    tmpTabuStatus = [firstPoint, secondPoint]
                                    if fitnessLocal - bestLocalFitness  < (-1)*0.00001 and ( (feasible == True and fitnessLocal - bestFitness < (-1)*0.00001) or (tmpTabuStatus not in tabuList[selectedNeighbor - 1]) ):
                                        bestLocalFitness = fitnessLocal
                                        bestLocalSolution = [copyListMove, copyRoutesOfDrone, copyVisitedPoint]
                                        bestTabuStatus = [firstPoint, succPointOfFirstPoint]
                                        excuted = True
                                        printSolution = [firstPoint, secondPoint]
                                        twoPointInSameRoute = True
                                        localFeasible = feasible
                    if i != N_Technican - 1:
                        #find y
                        for u in range(i+1, N_Technican):
                            N_point = len(ListMoveOfTech[u])
                            for v in range(1, N_point - 1):
                                secondPoint = ListMoveOfTech[u][v]
                                succPointOfSecondPoint = ListMoveOfTech[u][v + 1]
                                #create copy
                                copyListMove = copy.deepcopy(ListMoveOfTech)
                                copyTimeVisit = copy.deepcopy(TimeVisitPoint)
                                copyTimeGuess = copy.deepcopy(TimeGuessOfTech)
                                copyRoutesOfDrone = copy.deepcopy(allRouteOfDrones)
                                copyVisitedPoint = copy.deepcopy(visited)
                                #
                                segment2nd = []
                                segment4th = []
                                for t in range(j + 1, numberPointOfTechRoute):
                                    segment2nd.append(ListMoveOfTech[i][t])
                                    copyListMove[i].remove(ListMoveOfTech[i][t])
                                for t in range(v + 1, N_point):
                                    segment4th.append(ListMoveOfTech[u][t])
                                    copyListMove[u].remove(ListMoveOfTech[u][t])
                                for t in segment2nd:
                                    copyListMove[u].append(t)
                                for t in segment4th:
                                    copyListMove[i].append(t)
                                #In this case, each route is divided into two segments
                                # if there exist a drone’s trip either visits customers in both 1st segment and 4th segment or customers in both 3rd segment and 2nd segment
                                # remove in drone trip
                                for t in range(1, j + 1):
                                    pointInFirstSequence = ListMoveOfTech[i][t]
                                    for k in range(v + 1, N_point):
                                        pointInForthSequence = ListMoveOfTech[u][k]
                                        if copyVisitedPoint[pointInFirstSequence] == 1 and copyVisitedPoint[pointInForthSequence] == 1:
                                            if TheDroneRouteOfPoint[pointInFirstSequence] == TheDroneRouteOfPoint[pointInForthSequence]:
                                                changingDroneTrip = random.randrange(1,4)
                                                droneTrip = TheDroneRouteOfPoint[pointInFirstSequence] 
                                                if changingDroneTrip == 1:
                                                    copyRoutesOfDrone[droneTrip].remove(pointInFirstSequence)
                                                    copyVisitedPoint[pointInFirstSequence] = 0
                                                elif changingDroneTrip == 2:
                                                    copyRoutesOfDrone[droneTrip].remove(pointInForthSequence)
                                                    copyVisitedPoint[pointInForthSequence] = 0
                                                else:
                                                    copyRoutesOfDrone[droneTrip].remove(pointInFirstSequence)
                                                    copyRoutesOfDrone[droneTrip].remove(pointInForthSequence)
                                                    copyVisitedPoint[pointInForthSequence] = 0
                                                    copyVisitedPoint[pointInFirstSequence] = 0
                                            elif TheDroneRouteOfPoint[pointInFirstSequence] > TheDroneRouteOfPoint[pointInForthSequence]:
                                                changingDroneTrip = random.randrange(1,4)
                                                if changingDroneTrip == 1:
                                                    droneTrip = TheDroneRouteOfPoint[pointInFirstSequence]
                                                    copyRoutesOfDrone[droneTrip].remove(pointInFirstSequence)
                                                    copyVisitedPoint[pointInFirstSequence] = 0
                                                elif changingDroneTrip == 2:
                                                    droneTrip = TheDroneRouteOfPoint[pointInForthSequence]
                                                    copyRoutesOfDrone[droneTrip].remove(pointInForthSequence)
                                                    copyVisitedPoint[pointInForthSequence] = 0
                                                else:
                                                    droneTrip = TheDroneRouteOfPoint[pointInFirstSequence]
                                                    copyRoutesOfDrone[droneTrip].remove(pointInFirstSequence)
                                                    copyVisitedPoint[pointInFirstSequence] = 0
                                                    droneTrip = TheDroneRouteOfPoint[pointInForthSequence]
                                                    copyRoutesOfDrone[droneTrip].remove(pointInForthSequence)
                                                    copyVisitedPoint[pointInForthSequence] = 0
                                            
                                #
                                for t in range(j + 1, numberPointOfTechRoute):
                                    pointInSecondSequence = ListMoveOfTech[i][t]
                                    for k in range(1, v + 1):
                                        pointInThirdSequence = ListMoveOfTech[u][k]
                                        if copyVisitedPoint[pointInThirdSequence] == 1 and copyVisitedPoint[pointInSecondSequence] == 1:
                                            if TheDroneRouteOfPoint[pointInSecondSequence] == TheDroneRouteOfPoint[pointInThirdSequence]:
                                                changingDroneTrip = random.randrange(1,4)
                                                droneTrip = TheDroneRouteOfPoint[pointInSecondSequence] 
                                                if changingDroneTrip == 1:
                                                    copyRoutesOfDrone[droneTrip].remove(pointInSecondSequence)
                                                    copyVisitedPoint[pointInSecondSequence] = 0
                                                elif changingDroneTrip == 2:
                                                    copyRoutesOfDrone[droneTrip].remove(pointInThirdSequence)
                                                    copyVisitedPoint[pointInThirdSequence] = 0
                                                else:
                                                    copyRoutesOfDrone[droneTrip].remove(pointInSecondSequence)
                                                    copyRoutesOfDrone[droneTrip].remove(pointInThirdSequence)
                                                    copyVisitedPoint[pointInThirdSequence] = 0
                                                    copyVisitedPoint[pointInSecondSequence] = 0 
                                            elif TheDroneRouteOfPoint[pointInThirdSequence] > TheDroneRouteOfPoint[pointInSecondSequence]:
                                                changingDroneTrip = random.randrange(1,4)
                                                if changingDroneTrip == 1:
                                                    droneTrip = TheDroneRouteOfPoint[pointInThirdSequence]
                                                    copyRoutesOfDrone[droneTrip].remove(pointInThirdSequence)
                                                    copyVisitedPoint[pointInThirdSequence] = 0
                                                elif changingDroneTrip == 2:
                                                    droneTrip = TheDroneRouteOfPoint[pointInSecondSequence]
                                                    copyRoutesOfDrone[droneTrip].remove(pointInSecondSequence)
                                                    copyVisitedPoint[pointInSecondSequence] = 0
                                                else:
                                                    droneTrip = TheDroneRouteOfPoint[pointInThirdSequence]
                                                    copyRoutesOfDrone[droneTrip].remove(pointInThirdSequence)
                                                    copyVisitedPoint[pointInThirdSequence] = 0
                                                    droneTrip = TheDroneRouteOfPoint[pointInSecondSequence]
                                                    copyRoutesOfDrone[droneTrip].remove(pointInSecondSequence)
                                                    copyVisitedPoint[pointInSecondSequence] = 0
                                copyRoutesOfDrone = deleteEmptyRoute(copyRoutesOfDrone)
                                fitnessLocal = fitness(copyListMove, copyRoutesOfDrone, copyVisitedPoint)
                                tmpTabuStatus = [firstPoint, secondPoint]
                                tmpTabuStatus1 = [secondPoint, firstPoint]
                                if fitnessLocal - bestLocalFitness  < (-1)*0.00001 and ( (feasible == True and fitnessLocal - bestFitness < (-1)*0.00001) or ((tmpTabuStatus not in tabuList[selectedNeighbor - 1]) and (tmpTabuStatus1 not in tabuList[selectedNeighbor - 1])) ):
                                    bestLocalFitness = fitnessLocal
                                    bestLocalSolution = [copyListMove, copyRoutesOfDrone, copyVisitedPoint]
                                    bestTabuStatus = [firstPoint, secondPoint]
                                    bestTabuStatus1 = [secondPoint, firstPoint]       
                                    excuted = True
                                    printSolution = [firstPoint, secondPoint]
                                    twoPointInSameRoute = False
                                    localFeasible = feasible
            if excuted == True:
                if twoPointInSameRoute == True:
                    tabuList[selectedNeighbor - 1].append(bestTabuStatus)
                    iterationOfTabuStatus = random.randrange(intervalRoutingMove[0], intervalRoutingMove[1])
                    iterationDestroyTabu[selectedNeighbor - 1].append( iterationIndex + iterationOfTabuStatus)
                else:
                    tabuList[selectedNeighbor - 1].append(bestTabuStatus)
                    tabuList[selectedNeighbor - 1].append(bestTabuStatus1)
                    iterationOfTabuStatus = random.randrange(intervalRoutingMove[0], intervalRoutingMove[1])
                    iterationDestroyTabu[selectedNeighbor - 1].append( iterationIndex + iterationOfTabuStatus)
                    iterationDestroyTabu[selectedNeighbor - 1].append( iterationIndex + iterationOfTabuStatus)
            if bestLocalFitness - bestFitness < (-1)*0.00001 and localFeasible == True:
                bestSolution = copy.deepcopy(bestLocalSolution)
                bestFitness = bestLocalFitness
                improvedSolution = True
        elif selectedNeighbor == 4:
            bestLocalSolution = [ListMoveOfTech, allRouteOfDrones, visited]
            #deletion move
            nameOfNeighbor = "deletion"
            excuted = False
            # remove a point from drone trip
            if len(allRouteOfDrones) != 0:
                for i in range(len(allRouteOfDrones)):
                    sizeOfDroneTrip = len(allRouteOfDrones[i])
                    for j in range(1, sizeOfDroneTrip):
                        removePoint = allRouteOfDrones[i][j]
                        preOfRemovePoint = allRouteOfDrones[i][j - 1]
                        if j == sizeOfDroneTrip - 1:
                            sucOfRemovePoint = 0
                        else:
                            sucOfRemovePoint = allRouteOfDrones[i][j + 1]
                        #create copy
                        copyListMove = copy.deepcopy(ListMoveOfTech)
                        copyTimeVisit = copy.deepcopy(TimeVisitPoint)
                        copyRoutesOfDrone = copy.deepcopy(allRouteOfDrones)
                        copyVisitedPoint = copy.deepcopy(visited)
                        # remove
                        if sizeOfDroneTrip  <= 2:
                            copyRoutesOfDrone.remove(copyRoutesOfDrone[i])
                            copyVisitedPoint[removePoint] = 0
                        else:
                            copyRoutesOfDrone[i].remove(removePoint)
                            copyVisitedPoint[removePoint] = 0
                        fitnessLocal = fitness(copyListMove, copyRoutesOfDrone, copyVisitedPoint)
                        tmpTabuStatus = [removePoint, preOfRemovePoint, sucOfRemovePoint] #[point, preOfPoint, sucOfPoint]
                        if fitnessLocal - bestLocalFitness  < (-1)*0.00001 and ( (feasible == True and fitnessLocal - bestFitness < (-1)*0.00001) or ((tmpTabuStatus not in tabuList[selectedNeighbor])) ):
                            bestLocalFitness = fitnessLocal
                            bestLocalSolution = [copyListMove, copyRoutesOfDrone, copyVisitedPoint]
                            bestTabuStatus =  [removePoint, preOfRemovePoint, sucOfRemovePoint] #[point, preOfPoint, sucOfPoint]
                            printSolution = removePoint
                            localFeasible = feasible
                            excuted = True
                if excuted == False:
                    # if deletion not delete any point just select other neighborhood
                    continue
                tabuList[selectedNeighbor - 1].append(bestTabuStatus)
                iterationOfTabuStatus = random.randrange(intervalRoutingMove[0], intervalRoutingMove[1])
                iterationDestroyTabu[selectedNeighbor - 1].append( iterationIndex + iterationOfTabuStatus)
                if bestLocalFitness - bestFitness < (-1)*0.00001 and localFeasible == True:
                    bestSolution = copy.deepcopy(bestLocalSolution)
                    bestFitness = bestLocalFitness
                    improvedSolution = True
            else:
                # if deletion not delete any point just select other neighborhood
                continue
        else:
            #insertion move
            nameOfNeighbor = "insertion"
            bestLocalSolution = [ListMoveOfTech, allRouteOfDrones, visited]
            excuted = False
            #
            theTechRouteOfPoint = np.zeros(totalPoint, int)
            orderOfPointInTechRoute = np.zeros(totalPoint, int) 
            for i in range(N_Technican):
                for j in range(1, len(ListMoveOfTech[i])):
                    thisPoint = ListMoveOfTech[i][j]
                    theTechRouteOfPoint[thisPoint] = i
                    orderOfPointInTechRoute[thisPoint] = j
            for i in range(1, totalPoint):
                #insert a unvisited point to every position
                if visited[i] == 0 and i in cusInDroneFlyRange :
                    if N_Technican > 1:
                        if len(allRouteOfDrones) > 0:
                            for j in range(len(allRouteOfDrones)):
                                creatNewRoute = True
                                droneTripSize = len(allRouteOfDrones[j])
                                insertable = True
                                # check insertalbe and create new route
                                for u in range(1, droneTripSize):
                                    if theTechRouteOfPoint[allRouteOfDrones[j][u]] == theTechRouteOfPoint[i]:
                                        insertable = False
                                        break
                                if j != 0:
                                    for u in range(j):
                                        for v in range(1, len(allRouteOfDrones[u])):
                                            foundPoint = allRouteOfDrones[u][v]
                                            if theTechRouteOfPoint[foundPoint] == theTechRouteOfPoint[i]:
                                                if orderOfPointInTechRoute[foundPoint] > orderOfPointInTechRoute[i]:
                                                    insertable = False
                                                    creatNewRoute = False
                                                    break
                                for u in range(j, len(allRouteOfDrones)):
                                    for v in range(1, len(allRouteOfDrones[u])):
                                        foundPoint = allRouteOfDrones[u][v]
                                        if theTechRouteOfPoint[foundPoint] == theTechRouteOfPoint[i]:
                                            if orderOfPointInTechRoute[foundPoint] < orderOfPointInTechRoute[i]:
                                                insertable = False
                                                creatNewRoute = False
                                                break      
                                if creatNewRoute:
                                    #create copy
                                    copyListMove = copy.deepcopy(ListMoveOfTech)
                                    copyRoutesOfDrone = copy.deepcopy(allRouteOfDrones)
                                    copyVisitedPoint = copy.deepcopy(visited)
                                    #
                                    newRoute = [depot, i]
                                    copyRoutesOfDrone.insert(j, newRoute)
                                    copyVisitedPoint[i] = 1
                                    #
                                    fitnessLocal = fitness(copyListMove, copyRoutesOfDrone, copyVisitedPoint)
                                    tmpTabuStatus = [i, 0, 0] #[point, prePoint, succPoint]
                                    if fitnessLocal - bestLocalFitness  < (-1)*0.00001 and ( (feasible == True and fitnessLocal - bestFitness < (-1)*0.00001) or ( (tmpTabuStatus not in tabuList[selectedNeighbor - 2])) ):
                                        bestLocalFitness = fitnessLocal
                                        bestLocalSolution = [copyListMove, copyRoutesOfDrone, copyVisitedPoint]
                                        bestTabuStatus = [i, 0, 0] #[point, prePoint, succPoint]
                                        excuted = True
                                        printSolution = i     
                                        localFeasible = feasible    
                                if insertable == False:
                                    continue

                                # insert to current route
                                for k in range(1, droneTripSize + 1):
                                    #
                                    preOfRemovePoint = copyRoutesOfDrone[j][k - 1]
                                    if k == droneTripSize:
                                        sucOfRemovePoint = 0
                                    else:
                                        sucOfRemovePoint = copyRoutesOfDrone[j][k]
                                    #create copy
                                    copyListMove = copy.deepcopy(ListMoveOfTech)
                                    copyRoutesOfDrone = copy.deepcopy(allRouteOfDrones)
                                    copyVisitedPoint = copy.deepcopy(visited)
                                    #
                                    copyRoutesOfDrone[j].insert(k, i)
                                    copyVisitedPoint[i] = 1
                                    #
                                    fitnessLocal = fitness(copyListMove, copyRoutesOfDrone, copyVisitedPoint)
                                    tmpTabuStatus = [i, preOfRemovePoint, sucOfRemovePoint] #[point, prePoint, succPoint]
                                    if fitnessLocal - bestLocalFitness  < (-1)*0.00001 and ( (feasible == True and fitnessLocal - bestFitness < (-1)*0.00001) or ( (tmpTabuStatus not in tabuList[selectedNeighbor - 2])) ):
                                        bestLocalFitness = fitnessLocal
                                        bestLocalSolution = [copyListMove, copyRoutesOfDrone, copyVisitedPoint]
                                        bestTabuStatus = [i, preOfRemovePoint, sucOfRemovePoint] #[point, prePoint, succPoint]
                                        excuted = True
                                        printSolution = i
                                        localFeasible = feasible
                        else:
                            
                            #create copy
                            copyListMove = copy.deepcopy(ListMoveOfTech)
                            copyRoutesOfDrone = copy.deepcopy(allRouteOfDrones)
                            copyVisitedPoint = copy.deepcopy(visited)
                            routeOfDrone = [0, i]
                            copyRoutesOfDrone.append(routeOfDrone)
                            copyVisitedPoint[i] = 1
                            fitnessLocal = fitness(copyListMove, copyRoutesOfDrone, copyVisitedPoint)
                            tmpTabuStatus = [i, 0, 0]#[point, prePoint, succPoint]
                            if fitnessLocal - bestLocalFitness  < (-1)*0.00001 and ( (feasible == True and fitnessLocal - bestFitness < (-1)*0.00001) or ((tmpTabuStatus not in tabuList[selectedNeighbor - 2])) ):
                                bestLocalFitness = fitnessLocal
                                bestLocalSolution = [copyListMove, copyRoutesOfDrone, copyVisitedPoint]
                                bestTabuStatus = [i, 0, 0]#[point, prePoint, succPoint]
                                excuted = True
                                printSolution = i
                                localFeasible = feasible
                    else:
                        #create copy
                        
                        indexRoute = 0
                        for indexRoute in range(len(allRouteOfDrones) + 1):
                            # check insertalbe and create new route
                            copyListMove = copy.deepcopy(ListMoveOfTech)
                            copyRoutesOfDrone = copy.deepcopy(allRouteOfDrones)
                            copyVisitedPoint = copy.deepcopy(visited)
                            routeOfDrone = [0, i]
                            copyRoutesOfDrone.insert(indexRoute, routeOfDrone)
                            copyVisitedPoint[i] = 1
                            fitnessLocal = fitness(copyListMove, copyRoutesOfDrone, copyVisitedPoint)
                            tmpTabuStatus = [i, 0, 0]#[point, prePoint, succPoint]
                            if fitnessLocal - bestLocalFitness  < (-1)*0.00001 and ( (feasible == True and fitnessLocal - bestFitness < (-1)*0.00001) or ((tmpTabuStatus not in tabuList[selectedNeighbor - 2])) ):
                                bestLocalFitness = fitnessLocal
                                bestLocalSolution = [copyListMove, copyRoutesOfDrone, copyVisitedPoint]
                                bestTabuStatus = [i, 0, 0]#[point, prePoint, succPoint]
                                excuted = True
                                printSolution = i
                                localFeasible = feasible
            if excuted == True:
                tabuList[selectedNeighbor - 1].append(bestTabuStatus)
                iterationOfTabuStatus = random.randrange(intervalRoutingMove[0], intervalRoutingMove[1])
                iterationDestroyTabu[selectedNeighbor - 1].append( iterationIndex + iterationOfTabuStatus)
                if bestLocalFitness - bestFitness < (-1)*0.00001 and localFeasible == True: 
                    bestSolution = copy.deepcopy(bestLocalSolution)
                    bestFitness = bestLocalFitness
                    improvedSolution = True
            else:
                # if insertion not insert any point just select other neighborhood
                continue
        # checkFeasible = fitnessWithFeasible(bestLocalSolution[0], bestLocalSolution[1], bestLocalSolution[2])
        if feasibleWithDroneEdurance == True :
            alpha1 = alpha1/(1 + beta)
        else:
            alpha1 = alpha1*(1 + beta)
        if feasibleWithMaximumWorking == True:
            alpha2 = alpha2/(1 + beta)
        else:
            alpha2 = alpha2*(1 + beta)
        if feasibleWithDroneEdurance and feasibleWithMaximumWorking and improvedSolution:
            bestFeasibleFitness = bestLocalFitness
            bestFeasibleSolution = copy.deepcopy(bestLocalSolution)
            bestIT = totalIT
        # if currentFitness > bestLocalFitness:
        ListMoveOfTech = copy.deepcopy(bestLocalSolution[0])
        allRouteOfDrones = copy.deepcopy(bestLocalSolution[1])
        visited = copy.deepcopy(bestLocalSolution[2])
        print(bestLocalSolution)
        print("best: " + str(bestFitness))
        iterationIndex += 1
        if improvedSolution == False:
            ITnotImproved += 1
            ITnotImprovedWithITr += 1
        else:
            ITnotImproved = 0
            ITnotImprovedWithITr = 0
        if ITnotImprovedWithITr > ITr:
            r += deltaR
            ITnotImprovedWithITr = 0
        # if ITnotImproved >= ITmax:
        #     break
        if iterationIndex > numIT:
            break           
    previous_tech_routes = copy.deepcopy(bestSolution[0])
    previous_drone_routes = copy.deepcopy(bestSolution[1])
print(bestSolution)
                
