# Yuanzhe Liu
# 5/2019
# A* for a map

#==========imports====================
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
import utils as ut
import math
import user
#==========define Macros==============
ROAD_DIRECTION = 0
START_X = 1
START_Y = 2
END_X = 3
END_Y = 4
X=0
Y=1
G=2
#=========functions===================
def importmap(map_path):
    # Read from a map data file and save it to a float array array
    # @params: map_path: path to map file, @return: float array array
    # The map file must have the following format:
    # 1. each line is a road segment
    # 2. each line is "(ROAD_DIRECTION START_X START_Y END_X END_Y)"
    mapdata = np.loadtxt(map_path, usecols=(0,1,2,3,4),converters={0:ut.deli_first_col})
    return mapdata

def formatmap(mapdata):
    # Convert a mapdata to a dictionary, the value is array of [x,y,length of road]
    # @params: mapdata
    # @return: dictionary: {(startx,starty): [[endx1,endy1,l1],[endx2,endy2,l2]]}
    map = {}
    for road in mapdata:
        start = [road[START_X], road[START_Y]]
        end = [road[END_X], road[END_Y]]
        key = (road[START_X], road[START_Y])
        d = ut.distance(start,end)
        end.append(d)
        if key not in map:    # if the start is not in dictionary already
            map[key] = [end]
        else:   #else append the pair
            temp = map[key]
            temp.append(end)
        if road[ROAD_DIRECTION] == 2: # if it is a two-way, add both direction
            start = [road[END_X], road[END_Y]]
            end = [road[START_X], road[START_Y]]
            key = (road[END_X], road[END_Y])
            d = ut.distance(start,end)
            end.append(d)
            if key not in map:    # if the start is not in dictionary already
                map[key] = [end]
            else:   #else add the pair
                temp = map[key]
                temp.append(end)
    return map

def find_start_coor(start,map):
    # find the starting coordinates
    # @params: start, map, @return: [x,y] (a point on map)
    # for simplicity, we start from the cloest crossroad that closest to start
    d = math.inf
    temp_d = 0
    s = (0,0)
    for x in map:
        temp_d = ut.distance([x[X],x[Y]],start)
        if temp_d < d:
            s = (x[X],x[Y])
            d = temp_d
    return s

def find_goal_coor(goal,map):
    # find the starting coordinates
    # @params: goal, map, @return: [x,y] (a point on map)
    # for simplicity, we start from the cloest crossroad that closest to goal
    d = math.inf
    temp_d = 0
    s = (0,0)
    for x in map:
        temp_d = ut.distance([x[X],x[Y]],goal)
        if temp_d < d:
            s = (x[X],x[Y])
            d = temp_d
    return s

def Astar(start,goal,map):
    path=[]
    searching = True
    current = start
    openlist = []
    pathlist = []
    searchedlist = []
    #initialization
    nodes_expanded = 1
    searchedlist.append(start)
    for x in map[start]:
        openlist.append(x)
        pathlist.append([start])
    # start searching
    i = 0
    while (searching):
        i = i + 1
        #print("iteration ", i, "openlist:")
        #print(openlist)
        #print(pathlist)
        f = math.inf
        next = None
        g = 0
        # find the next node
        for x in openlist:
            if (x[X],x[Y]) in searchedlist: continue
            temp_f = x[G] + ut.distance((x[X],x[Y]),goal)   # f(n) = g(n) + h(n)
            if temp_f < f:
                # print("search x", x, "f(n)=",temp_f) #TODO: delete
                next = x.copy()
                f = temp_f
        idx = openlist.index(next)
        openlist.pop(idx)
        path = pathlist[idx].copy()
        pathlist.pop(idx)
        # if the next node is the goal, then we have found the path
        if next[X] == goal[X] and next[Y] == goal[Y]:
            nodes_expanded = nodes_expanded + 1
            path.append((next[X],next[Y]))
            print("A* search expanded ", nodes_expanded, " nodes")
            return (path,next[G])
        # open next node
        nodes_expanded = nodes_expanded + 1
        try:
            for x in map[(next[X],next[Y])]:
                if (x[X],x[Y]) in searchedlist: continue
                openlist.append([x[X],x[Y],x[G]+next[G]])
                t = path.copy()
                t.append((next[X],next[Y]))
                pathlist.append(t)
                searchedlist.append((next[X],next[Y]))
        except:
            continue

def Uniform_cost(start,goal,map):
    path=[]
    searching = True
    current = start
    openlist = []
    pathlist = []
    searchedlist = []
    #initialization
    nodes_expanded = 1
    searchedlist.append(start)
    for x in map[start]:
        openlist.append(x)
        pathlist.append([start])
    # start searching
    i = 0
    while (searching):
        i = i + 1
        f = math.inf
        next = None
        g = 0
        # find the next node
        for x in openlist:
            if (x[X],x[Y]) in searchedlist: continue
            temp_f = x[G]    # f(n) = g(n)
            if temp_f < f:
                next = x.copy()
                f = temp_f
        #print(next)
        idx = openlist.index(next)
        openlist.pop(idx)
        path = pathlist[idx].copy()
        pathlist.pop(idx)
        # if the next node is the goal, then we have found the path
        if next[X] == goal[X] and next[Y] == goal[Y]:
            nodes_expanded = nodes_expanded + 1
            path.append((next[X],next[Y]))
            print("Uniform cost search expanded ", nodes_expanded, " nodes")
            return (path,next[G])
        # open next node
        nodes_expanded = nodes_expanded + 1
        try:
            for x in map[(next[X],next[Y])]:
                if (x[X],x[Y]) in searchedlist: continue
                openlist.append([x[X],x[Y],x[G]+next[G]])
                t = path.copy()
                t.append((next[X],next[Y]))
                pathlist.append(t)
                searchedlist.append((next[X],next[Y]))
        except:
            continue

# function to draw the map
def draw_map(map,path):
    # draw the map
    figure(num=None, figsize=(6, 9), dpi=80, facecolor='w', edgecolor='k')
    for s in map:
        for e in map[s]:
            x = [s[X],e[X]]
            y = [s[Y],e[Y]]
            plt.plot(x,y,'k')
    # draw the path
    n = len(path)
    for i in range(n-1):
        s = path[i]
        e = path[i+1]
        x = [s[X],e[X]]
        y = [s[Y],e[Y]]
        plt.plot(x,y,'r')
    plt.show()

mapdata = importmap("map.txt")
map = formatmap(mapdata)
mapdata = None
start = find_start_coor(user.START, map)
goal = find_goal_coor(user.GOAL, map)
(p1,l1) = Astar(start,goal,map)
(p2,l2) = Uniform_cost(start,goal,map)
print("Distance by A*: ",l1)
print("Distance by ucs: ",l2)
print("Both searches found the same path") if (p1 == p2) else ("Found different path")
draw_map(map, p1)
