#!/usr/bin/env python
from yaml import load, dump
try:
  from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
  from yaml import Loader, Dumper

from rospy_message_converter import message_converter
from controller_msgs.msg import *
import os
import sys
import rospy
import time


if __name__ == '__main__':
  rospy.init_node('ValkyrieShakeout')
  if not rospy.has_param('~DataFile'):
    print('Please specify the data file (YAML)!')
  else:
    # Load YAML data
    data = load(open(rospy.get_param('~DataFile', '')))
    wait_time = 3.0
    # Setup ROS node   
    pub = rospy.Publisher('/ihmc/valkyrie/humanoid_control/input/foot_load_bearing', FootLoadBearingMessage, queue_size=10, latch=True)

    # Prepare message
    print("Preparing Message")
    message = message_converter.convert_dictionary_to_ros_message('controller_msgs/FootLoadBearingMessage', data)

    while not rospy.is_shutdown():
        print("Publishing Message")
        pub.publish(message)
        print("Waiting for execution ...")        
        time.sleep(wait_time)
        break

    print('Done')

