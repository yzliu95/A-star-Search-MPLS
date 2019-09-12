# Yuanzhe Liu
# 5/2019
# A* for a map
# this files holds some utility functions

import math
# rules to convert the first column of map data
def deli_first_col(str):
    return 1 if (str == b'(1') else 2

# calculate distance between two points
def distance(a,b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

# function to print map
def print_map(map):
    for x in map:
        print(x,":",map[x])
