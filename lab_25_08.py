from cmath import sqrt
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
total_new_point = totalPoint
# 
#  
#
while total_new_point > N_Technican:
    print(1)
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
    print(list_cell)

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
                if i in unmatched_list:
                    matched_point = (found_point, i)
                    list_matched_point[total_new_point] = matched_point
                    new_coor = [(current_coor_matrix[found_point][0] + current_coor_matrix[i][0])/2, (current_coor_matrix[found_point][1] + current_coor_matrix[i][1])/2]
                    new_coor_matrix.append(new_coor)
                    unmatched_list.remove(found_point)
                    unmatched_list.remove(i)
                    total_new_point += 1
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
    print(list_matched_point)
    
print(dict_matched_point)

