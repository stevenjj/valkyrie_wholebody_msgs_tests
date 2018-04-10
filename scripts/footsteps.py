#!/usr/bin/env python
from yaml import load, dump
try:
  from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
  from yaml import Loader, Dumper

from rospy_message_converter import message_converter
from ihmc_msgs.msg import *
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Point, Quaternion
import sys
import os
import rospy
import numpy as np
import tf
from tf_conversions import transformations
import tf_conversions.posemath as pm
import time

def status(m):
  global ready
  global lst_ready
  global stop
  global has_stopped_moving
  if m.data=='STANDING':
    ready = True
  else:
    ready = False
  if stop and ready==True and lst_ready==False:
    has_stopped_moving = True
  lst_ready = ready

def callback(m):
  global pub
  global stop
  global data
  if ready:
    if not stop:
      print('Preparing the footstep message')
      # Reset step height to zero
      m.pose.pose.position.z = 0.0
      # Get current robot pose
      pose = pm.fromMsg(m.pose.pose)
      # Get footstep trajectopry from YAML 
      message = message_converter.convert_dictionary_to_ros_message('ihmc_msgs/FootstepDataListRosMessage', data)
      # Update the footstep frames
      for step in message.footstep_data_list:
        pstep = pose*pm.Frame(pm.Rotation.Quaternion(step.orientation.x, step.orientation.y, step.orientation.z, step.orientation.w), pm.Vector(step.location.x, step.location.y, step.location.z))
        msg = pm.toMsg(pstep)
        step.location = msg.position
        step.orientation = msg.orientation
      print('Relocated footsteps to a local frame')
      print('Publishing...')
      pub.publish(message)
      stop = True
      print('Waiting for execution...')

if __name__ == '__main__':
  rospy.init_node('ValkyrieShakeout')
  if not rospy.has_param('~DataFile'):
    print('Please specify the data file (YAML)!')
  else:
    stop = False
    lst_ready = False
    ready = False
    has_stopped_moving = False
    # Load YAML data
    data = load(open(rospy.get_param('~DataFile', '')))
    # Setup ROS node
    
    pub = rospy.Publisher('/ihmc_ros/valkyrie/control/footstep_list', FootstepDataListRosMessage, queue_size=10)
    print('Waiting for robot pose and robot to stop moving...')
    time.sleep(0.5)
    rospy.Subscriber("/ihmc_ros/valkyrie/output/robot_pose", Odometry, callback)
    rospy.Subscriber("/ihmc_ros/valkyrie/output/robot_motion_status", String, status)
    while not has_stopped_moving and not rospy.is_shutdown():
      time.sleep(0.1)
    print('Done')

