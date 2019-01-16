#! /usr/bin/env python
import rospy
import numpy as np
from rospy.numpy_msg import numpy_msg
from ekftopics_template.msg  import *

def callback(data):
   print data 

def listener():
    rospy.init_node('listener')
    rospy.Subscriber('ekf_out',numpy_msg(ekfState),callback)
    rospy.spin()

if __name__=='__main__':
    listener()

