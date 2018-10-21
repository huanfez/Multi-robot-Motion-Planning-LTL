
gridWidth = 10
gridLength = 10

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

# define neighbor regions
def neighbors(curPos):
    neighbors = []     
    for row in range(-1,2):
        for col in range(-1,2):
            if (curPos[0] - row) > 0 and (curPos[0] - row) <= 10 and (curPos[1] - col) > 0 and (curPos[1] - col) <= 10:
                neighbors = neighbors + [twoD2oneD(curPos[0] - row, curPos[1] - col)]
    return neighbors

# Obstacles detection
def obstacleDtc(Obs, curPos):
    return list(set(neighbors(curPos)) & set(Obs))

# define surrounded regions
def surrounds(curPos):
    if curPos[0] > 1 and curPos[0] < 10 and curPos[1] > 1 and curPos[1] < 10:
        surround = [twoD2oneD(curPos[0], curPos[1] - 1), twoD2oneD(curPos[0] - 1, curPos[1]), \
                     twoD2oneD(curPos[0] + 1, curPos[1]), twoD2oneD(curPos[0], curPos[1] + 1)]
    elif curPos[0] == 1 and curPos[1] > 1 and curPos[1] < 10:
        surround = [twoD2oneD(curPos[0], curPos[1] - 1), \
                     twoD2oneD(curPos[0] + 1, curPos[1]), twoD2oneD(curPos[0], curPos[1] + 1)]
    elif curPos[0] == 10 and curPos[1] > 1 and curPos[1] < 10:
        surround = [twoD2oneD(curPos[0], curPos[1] - 1), \
                     twoD2oneD(curPos[0] - 1, curPos[1]), twoD2oneD(curPos[0], curPos[1] + 1)]
    elif curPos[0] > 1 and curPos[0] < 10 and curPos[1] == 1:
        surround = [twoD2oneD(curPos[0] - 1, curPos[1]), twoD2oneD(curPos[0] + 1, curPos[1]), \
                     twoD2oneD(curPos[0], curPos[1] + 1)]
    elif curPos[0] > 1 and curPos[0] < 10 and curPos[1] == 10:
        surround = [twoD2oneD(curPos[0], curPos[1] - 1), twoD2oneD(curPos[0] - 1, curPos[1]), \
                     twoD2oneD(curPos[0] + 1, curPos[1])]
    elif curPos[0] == 1 and curPos[1] == 1:
        surround = [twoD2oneD(curPos[0] + 1, curPos[1]), twoD2oneD(curPos[0], curPos[1] + 1)]
    elif curPos[0] == 1 and curPos[1] == 10:
        surround = [twoD2oneD(curPos[0], curPos[1] - 1), twoD2oneD(curPos[0] + 1, curPos[1])]
    elif curPos[0] == 10 and curPos[1] == 1:
        surround = [twoD2oneD(curPos[0] - 1, curPos[1]), twoD2oneD(curPos[0], curPos[1] + 1)]
    elif curPos[0] == 10 and curPos[1] == 10:
        surround = [twoD2oneD(curPos[0], curPos[1] - 1), twoD2oneD(curPos[0] - 1, curPos[1])]
    return surround

# Agent collision detection
def CollisionDtc(othNex, curPos):
    return list(set(surrounds(curPos)) & set(othNex))
