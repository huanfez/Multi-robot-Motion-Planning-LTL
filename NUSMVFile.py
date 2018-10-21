import subprocess

# tasks = [[55, 88, 99], [63, 72], [43, 79]]
# startPos = [15, 19, 91]
# m = 0
# expansion = 0
# obstacles = [66, 77]
gridWidth = 10
gridLength = 10

# ltlSpecs = 'LTLSPEC ! ( F x.state = ' + str(goal) + ')'

# Two dimensional coordinates into one
def twoD2oneD(dataX, dataY):
    data = dataX + (dataY - 1) * gridWidth
    return data

# One dimensional coordinates into two
def oneD2twoD(data):
    y = (data-1)/gridWidth + 1
    x = data%gridWidth
    if x == 0:
        x = 10
    return x, y

# Find local area
def getChunk(start, goal, expansion):
    x = [(oneD2twoD(start))[0], (oneD2twoD(goal))[0]]
    y = [(oneD2twoD(start))[1], (oneD2twoD(goal))[1]]
    gridChunk = [min(x)-expansion, max(x)+expansion, min(y)-expansion, max(y)+expansion]

    if gridChunk[0] < 1:
        gridChunk[0] = 1
    if gridChunk[1] > gridWidth:
        gridChunk[1] = gridWidth
    if gridChunk[2] < 1:
        gridChunk[2] = 1
    if gridChunk[3] > gridLength:
        gridChunk[3] = gridLength

    return gridChunk

# Generate local states
def genLocStates(gridChunks,obstacles):
    states = []

    for yy in range(gridChunks[2], gridChunks[3]+1):
        for xx in range(gridChunks[0], gridChunks[1]+1):
            if twoD2oneD(xx, yy) not in obstacles:
                states.append(twoD2oneD(xx, yy))

    return states

# Generate state transition process
def GridTransitions(gridChunks, locStates, obstacles):
    transStr = []
    xl = gridChunks[0]
    xu = gridChunks[1]
    yl = gridChunks[2]
    yu = gridChunks[3]

    for i in locStates:
        x = (oneD2twoD(i))[0]
        y = (oneD2twoD(i))[1]
    
        if x > xl and x < xu:
            if y > yl and y < yu:
                transition = [i-gridWidth, i-1, i+1, i+gridWidth, i]
            elif y == yl:
                transition = [i-1, i+1, i+gridWidth, i]
            else: # y == yu
                transition = [i-gridWidth, i-1, i+1, i]
        elif x == xl:
            if y > yl and y < yu:
                transition = [i-gridWidth, i+1, i+gridWidth, i]
            elif y == yl:
                transition = [i+1, i+gridWidth, i]
            else: # y == yu
                transition = [i-gridWidth, i+1, i]
        else: # x == xu
            if y > yl and y < yu:
                transition = [i-gridWidth, i-1, i+gridWidth, i]
            elif y == yl:
                transition = [i-1, i+gridWidth, i]
            else: # y == yu
                transition = [i-gridWidth, i-1, i]

        transition = [e for e in transition if e not in obstacles]
        transition = [e for e in transition if e in locStates]

        # Write states to the smv file
        if transition != []:
            transStr.append([i,transition])

    return transStr          

# Generate NuSMV file
def makeNusmvFile(ltlSpecs, start, goal, expansion, obstacles):
    f = open('MotionPlan.smv','w') 
 
    f.write('MODULE main\n') 
    f.write('VAR\n') 
    f.write('x : grid;\n') 
    f.write('%s\n' %ltlSpecs) 
    f.write('MODULE grid\n')
    f.write('VAR\n')

    gridChunks = getChunk(start, goal, expansion)
    localStates = genLocStates(gridChunks, obstacles)
    f.write('state : {%s} ;\n' % ", ".join( repr(st) for st in localStates)) # be careful, here is a list
    f.write('ASSIGN\n')
    f.write('init(state) := %.0f;\n' %start)
    f.write('next(state) :=\n')
    f.write('case\n')

    GridTrans = GridTransitions(gridChunks, localStates, obstacles)
    for transition in GridTrans:
        f.write('state = %.0f : {%s};\n' % (transition[0],", ".join( repr(tran) for tran in transition[1]))) # be careful, here is a list

    f.write('TRUE : state;\n')
    f.write('esac;')

    f.close()
    return 0

# Define nusmv planning function
def NuSMVPlan():
    path = []
    twoDpath = []

    p = subprocess.Popen('NuSMV MotionPlan.smv', shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    for line in p.stdout.readlines():
        numpos = line.find("x.state = ")
        if numpos != -1:
            # print line[numpos+10:(numpos+12)],
            path.append(line[numpos+10:(numpos+12)])
    retval = p.wait()
    print path
    pathR = map(int,path)
    for position in pathR:
        twoDpath.append(oneD2twoD(position))
    #print twoDpath
    del twoDpath[0]
    if twoDpath != []:
        del twoDpath[-1]
    # print twoDpath
    return twoDpath
