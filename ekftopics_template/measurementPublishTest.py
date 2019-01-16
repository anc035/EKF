#! /usr/bin/env python
import rospy
import numpy as np
from rospy.numpy_msg import numpy_msg
from ekftopics_template.msg  import *
def talker():
    rospy.init_node('controlInputTester',anonymous=True)

    pub=rospy.Publisher('controlInput',numpy_msg(ekfMeasurement),queue_size=1)
    rate=rospy.Rate(1)
    while not rospy.is_shutdown():
        omni=ekfMeasurement()
        omni.covarMeasured=np.array([1,2,2,2,4,4,3,3,3],dtype=np.float64)
        rospy.loginfo(rate)
        rospy.loginfo(omni)
        pub.publish(omni)
        rate.sleep()


if __name__=='__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass



