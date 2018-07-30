#!/usr/bin/env python

from controller_msgs.msg import *
import rospy
import time

def footStatus():
    print ">>>> check dat status yo"

if __name__ == '__main__':
    rospy.init_node('ValkyrieShakeout')

    subscriber_uut = rospy.Subscriber("/ihmc/valkyrie/humanoid_control/output/footstep_status", FootstepStatusMessage, footStatus)
    time.sleep(1.0)
    subscriber_uut.unregister()
    print ">>>> Done!!!"
