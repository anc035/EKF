import rospy
import numpy as np
stateSize=6
controlSize=4

def rotation(yaw):
    return np.array([[np.cos(yaw), -np.sin(yaw)],[np.sin(yaw), np.cos(yaw)]])

def stateFunc(state, u, rate):
    dt = 1./rate
    x0 = np.copy(state)
    x1 = np.zeros([6,1])
    R = rotation(x0[3,0])
    x1[0]= x0[0]+dt*u[0]*np.cos(u[3])-dt*u[1]*np.sin(u[3])
    x1[1]= x0[1]+dt*u[0]*np.sin(u[3])+dt*u[1]*np.cos(u[3])
    #x1[0:2] = x0[0:2] + dt * np.dot(R,u[0:2])
    x1[2] = x0[2] + dt * u[2]
    x1[3] = 0
    x1[4] = 0
    x1[5] = x0[5] + dt * u[3]
    return x1

def computeJacobian(state, u, rate):
    dt = 1./rate
    x = np.copy(state)
    row1 = [ 1, 0, 0, 0, 0, -dt*u[0]*np.sin(x[3])-dt*u[1]*np.cos(x[3]) ]
    row2 = [ 0, 1, 0, 0, 0, dt*u[0]*np.cos(x[3])-dt*u[1]*np.sin(x[3]) ]
    row3 = [ 0, 0, 1, 0, 0, 0]
    row4 = [ 0, 0, 0, 0, 0, 0]
    row5 = [ 0, 0, 0, 0, 0, 0]
    row6 = [ 0, 0, 0, 0, 0, 1]
    
    return np.array([row1, row2, row3, row4, row5, row6])
    


