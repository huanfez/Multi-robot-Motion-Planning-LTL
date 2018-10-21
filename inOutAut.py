# read a text file as a list of lines
# find the last line, change to a file you have

def readAut(fileName):
    #fileHandle = open ( '/media/hz/New Volume/GAP/GAP_manual/decomAut.txt',"r" )
    fileHandle = open ( fileName,"r" )
    lineList = fileHandle.readlines()
    fileHandle.close()
    print "The last line is:"
    # print lineList[len(lineList)-1]
    # or simply
    print lineList[-1]

    finiteWords = []
    auts = lineList[-1][:]

    ls = auts.find('[')
    le = auts.find(']') + 1
    while ls !=-1 and le !=-1:
        finiteWords.append(auts[ls:le])
        auts = auts[le:]
        ls = auts.find('[')
        le = auts.find(']') + 1

    tasks = []
    for finiteWord in finiteWords:
        subTasks = []
        fws = finiteWord.find('"')
        fwe = finiteWord[fws+1:].find('"')
        while fws !=-1 and fwe !=-1:
            subTasks.append(finiteWord[fws+1:fws+fwe+1])
            #print finiteWord[fws+1:fws+fwe+1]
            finiteWord = finiteWord[fws+fwe+2:]
            fws = finiteWord.find('"')
            fwe = finiteWord[fws+1:].find('"')
        tasks.append(subTasks)
    print tasks
    return tasks

#def reshAut():

