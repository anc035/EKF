#! /usr/bin/env python
import numpy as np
import math
stateSize=3
controlSize=2
def stateFunc(state,controlInputU,rate):
    rate=1.0/rate
    state[0]=state[0]+math.cos(state[2])*controlInputU[0]*rate
    state[1]=state[1]+math.sin(state[2])*controlInputU[0]*rate
    state[2]=state[2]+controlInputU[1]*rate
    return state

def computeJacobian(state,controlInputU,rate):
    rate=1.0/rate
    x=np.copy(state)
    row1=[0,0,-math.sin(x[2])*controlInputU[0]*rate]
    row2=[0,0,math.cos(x[2])*controlInputU[0]*rate]
    row3=[0,0,0]
    return np.array([row1,row2,row3])
