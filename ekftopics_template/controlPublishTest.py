#! /usr/bin/env python
import rospy
import numpy as np
from rospy.numpy_msg import numpy_msg
from ekftopics_template.msg  import *
def talker():
    rospy.init_node('controlInputTester',anonymous=True)

    pub=rospy.Publisher('ekfControlInput',numpy_msg(ekfControl),queue_size=1)
    rate=rospy.Rate(1)
    while not rospy.is_shutdown():
        omni=ekfControl()
        omni.control=np.array([[1],[0],[0],[1]],dtype=np.float64)
        rospy.loginfo(rate)
        rospy.loginfo(omni)
        pub.publish(omni)
        rate.sleep()


if __name__=='__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass



