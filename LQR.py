#!/usr/bin/python

import numpy as np
import scipy.linalg
import matplotlib.pyplot as plt

def dlqr(A,B,Q,R):
    """
    Solve the discrete time lqr controller.
    x[k+1] = A x[k] + B u[k]
    cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    """
    # first, solve the ricatti equation
    P = np.matrix(scipy.linalg.solve_discrete_are(A, B, Q, R))
    # compute the LQR gain
    K = np.matrix(scipy.linalg.inv(B.T*P*B+R)*(B.T*P*A))
    return -K

def ctrOutput(curPos,nexPos,coef):
    A = np.matrix([[1,0],[0,1]])
    B = coef*np.matrix([[1,0],[0,1]])
    Q = np.matrix([[1,0],[0,1]])
    R = np.matrix([[1,0],[0,1]])
    #print dlqr(A,B,Q,R)

    K = dlqr(A,B,Q,R)
    sysInput = (np.array(K)).dot(np.array(curPos) - np.array(nexPos))
    sysInput = sysInput/np.linalg.norm(sysInput)
    newPos = (np.array(A)).dot(np.array(curPos)) + (np.array(B)).dot(sysInput)
    return newPos[0][0], newPos[1][0]

def updatePos(newPos):
    # Delete old circles

    # Add new circles
    circle1 = plt.Circle(newPos, 0.2)
    circle2 = plt.Circle(newPos, 0.55)

    fig = plt.gcf()
    ax = fig.gca()

    ax.add_artist(circle1)
    ax.add_artist(circle2)

    return 0


# curPos = [[1],[1]]
# nexPos = [[1],[2]]
# coef = 0.1

# newPos = ctrOutput(curPos,nexPos,coef)
# updatePos(newPos)
