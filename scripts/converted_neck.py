#!/usr/bin/env python
from yaml import load, dump
try:
  from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
  from yaml import Loader, Dumper

from rospy_message_converter import message_converter
from controller_msgs.msg import *
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Point, Quaternion, Vector3
import sys
import os
import rospy
import numpy as np
import tf
from tf import TransformListener
from tf_conversions import transformations
import tf_conversions.posemath as pm
import time
from copy import deepcopy

def status(m):
  global ready
  global stop
  global pauseAt
  if m.data=='STANDING':
    ready = True
  else:
    ready = False

def wait(t, pause=False):
  global robotTime
  t0 = robotTime
  while robotTime-t0<t and not rospy.is_shutdown():
    time.sleep(0.01)

def getMaxT(val):
  global maxT
  if len(val)>0:
    maxT = max(maxT, val[-1].time)

def callback(m):
  global stop
  global data
  global robotTime
  robotTime = m.header.stamp.to_sec()
  if ready:
    if not stop:
      stop = True

if __name__ == '__main__':
  rospy.init_node('ValkyrieShakeout')
  if not rospy.has_param('~DataFile'):
    print('Please specify the data file (YAML)!')
  else:
    stop = False
    ready = False
    pause = False
    # Load YAML data
    data = load(open(rospy.get_param('~DataFile', '')))
    # Setup ROS node
    robotTime = 0
    
    pubWhole = rospy.Publisher('/ihmc/valkyrie/humanoid_control/input/neck_trajectory', NeckTrajectoryMessage, queue_size=10)
    print('Waiting for robot pose and robot to stop moving...')
    time.sleep(0.5)
    rospy.Subscriber("/ihmc_ros/valkyrie/output/robot_pose", Odometry, callback)
    rospy.Subscriber("/ihmc_ros/valkyrie/output/robot_motion_status", String, status)
    while not rospy.is_shutdown():
      if stop:
        print('Preparing the neck message')
        message = message_converter.convert_dictionary_to_ros_message('controller_msgs/NeckTrajectoryMessage', data)
        print('Executing prep move...')
        maxT = 0.0
        for msg in message.jointspace_trajectory.joint_trajectory_messages:
          getMaxT(msg.trajectory_points)
        pubWhole.publish(message)
        print('Waiting for execution...')
        wait(maxT, True)
        break
      else:
        time.sleep(0.1)
    print('Done')
