def find_start_coor(raw_coor,mapdata):
    # find the starting coordinates
    # @params: raw_coor: [x,y], mapdata, @return: [x,y] (a point on map)
    # for simplicity, we start from the cloest crossroad
    d = math.inf
    temp_d = 0
    coor = [math.inf, math.inf]
    for road in mapdata:        # check all roads
        temp_d = ut.distance(raw_coor,[road[START_X],road[START_Y]])
        if (temp_d < d):
            d = temp_d
            coor = [road[START_X],road[START_Y]]
            if road[ROAD_DIRECTION] == 2:
                return

    return
