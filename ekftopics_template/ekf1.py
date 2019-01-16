#! /usr/bin/env python
import sys
import numpy as np
import rospy

#numpy-ize the messages so it can easily be used for equations
from rospy.numpy_msg import numpy_msg
#import custom messages see msg folder for details
from ekftopics_template.msg import *
controlInputU=0
def initKalman(stateSize):
    #initializing state and control matrix
    stateX=np.zeros([stateSize,1])
    I=np.eye(stateSize)
    Q=np.eye(stateSize)
    P=np.eye(stateSize)
    J=np.eye(stateSize)
    return stateX,I,Q,P,J

def ekf_predict(stateX,controlInputU,Q,P,J,rate):
    stateX=model.stateFunc(stateX,controlInputU,rate)
    J=model.computeJacobian(stateX,controlInputU,rate)
    stateCovarP=np.dot(J,np.dot(P,J.T))+Q
    return(stateX,stateCovarP)

def controlCallback(data):
    rospy.loginfo(data.control)
    global controlInputU
    controlInputU=data.control
    return controlInputU

def measureCallback(data,stateX,P,I):
    #stateMeasured nx1 covarmeasured nxn
    stateMeasured=data.stateMeasured
    covarMeasured=np.diag(data.covarMeasured)
    #plug into update equations
    stateX,P=ekf_update(stateX,P,stateMeasured,I,covarMeasured)
    return stateX,P


def ekf_update(stateX,P,stateMeasured,I,R):
    KalmanGain=np.dot(P,inv(P+R))
    stateX=stateX+np.dot(KalmanGain,(stateMeasured-stateX))
    P=P-np.dot(KalmanGain,P)

    return (stateX,P)

def main():
    rospy.init_node('ekf_node')
    T=10
    rate= rospy.Rate(T)
    stateX,I,Q,P,J=initKalman(model.stateSize)

    pub=rospy.Publisher('ekf_out',numpy_msg(ekfState),queue_size=1)
    rospy.Subscriber('ekfmeasurement', numpy_msg(ekfMeasurement), measureCallback,(stateX,P,I))
    rospy.Subscriber('ekfControlInput',numpy_msg(ekfControl),controlCallback)

    while not rospy.is_shutdown():
        stateX,P=ekf_predict(stateX,controlInputU,Q,P,J,T)
        pub.publish(stateX)

        rate.sleep()


if __name__== '__main__':
    try:
    #import robot model file
        model=rospy.get_param('model_name')
        rospy.loginfo(model)
        if model=='Omni':
            import OmniBot as model
        elif model=='Quad':
            import Quad as model
        else: 
            rospy.loginfo('Invalid Robot model')
            sys.exit(1)
        controlInputU=np.zeros([model.controlSize,1]) 
        main()
    except rospy.ROSInterruptException:
        pass


