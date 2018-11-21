#!/usr/bin/env python
from yaml import load, dump
try:
  from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
  from yaml import Loader, Dumper

import rospy
from rospy_message_converter import message_converter
from controller_msgs.msg import *
import numpy as np


def wait(t):
  rospy.sleep(t)

def getMaxT(val):
  global maxT
  if len(val)>0:
    maxT = max(maxT, val[-1].time)

if __name__ == '__main__':
  rospy.init_node('ValkyrieShakeout')
  if not rospy.has_param('~DataFile'):
    print('Please specify the data file (YAML)!')
  else:
    # Load YAML data
    data = load(open(rospy.get_param('~DataFile', '')))
    # Setup ROS node   
    pubFingers = rospy.Publisher('/ihmc/valkyrie/humanoid_control/input/valkyrie_hand_finger_trajectory', ValkyrieHandFingerTrajectoryMessage, queue_size=10, latch=True)
    print('Preparing the finger message')
    message = message_converter.convert_dictionary_to_ros_message('controller_msgs/ValkyrieHandFingerTrajectoryMessage', data)
    print('Executing finger trajectory..')

    # Get Maximum Trajectory Time
    maxT = 0.0
    for msg in message.jointspace_trajectory.joint_trajectory_messages:
      getMaxT(msg.trajectory_points)
    pubFingers.publish(message)
    print('Waiting for execution...')
    wait(maxT)
    print('Done')
