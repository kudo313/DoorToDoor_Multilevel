from os import name, remove
import random
from typing import List
from black import diff
import numpy as np
import math
import matplotlib.pyplot as plt
import random
import time
import copy
import openpyxl
from numpy.core.fromnumeric import size
import time

from regex import F

# const
numberOfCus  = 10
numberOfDepot = 1
minDis = 1000000000
nearestPoint = 0
D = 4*60
alpha1 = 1
alpha2 = 1
beta = 0.5
depot = 0 # repair something with 0 by depot
timeRateTech = 1/0.58
timeRateDrone = 1/0.83
edurance = 30
N_Technican = 3
N_Technican_max = 3
# read data from txt
f_name = "20.20.2"
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

# prepare for matching

# 
#  
#
while True:
    # create grid 
    h = 1
    list_cell = {}
    for i in totalPoint:
        x_posi = coor_matrix[i][0]/h
        y_posi = coor_matrix[i][1]/h
        posi = (x_posi, y_posi)
        list_cell[posi].append(i)


