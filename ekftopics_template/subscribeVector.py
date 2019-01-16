#! /usr/bin/env python
import rospy
import numpy as np
from rospy.numpy_msg import numpy_msg
from ekftopics_template.msg  import *

def callback(data,iden):
    covarMeasured=np.diag(data.covarMeasured)
    print covarMeasured
    

def listener():
    iden=np.eye(3)
    rospy.init_node('listener')
    rospy.Subscriber('controlInput',numpy_msg(ekfMeasurement),callback,(iden))
    rospy.spin()

if __name__=='__main__':
    listener()

