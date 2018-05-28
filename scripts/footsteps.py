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
from tf import TransformListener
from tf_conversions import transformations
import tf_conversions.posemath as pm
import time

def status(m):
  global ready
  global lstReady
  global stop
  global hasStoppedMoving
  if m.data=='STANDING':
    ready = True
  else:
    ready = False
  if stop and ready==True and lstReady==False:
    hasStoppedMoving = True
  lstReady = ready

def footStatus(m):
  global pause
  global pauseAt
  global ready
  global pubPause
  global lastStep

  if pauseAt>0:
    if m.status == 0 and m.footstep_index >= pauseAt:
      print('Pausing the walking ...')
      pause = True
      message = PauseWalkingRosMessage()
      message.pause = True
      message.unique_id = 1
      pubPause.publish(message)
  if m.status == 1:
     lastStep = m.footstep_index

def callback(m):
  global pub
  global stop
  global data
  global tfListener
  if ready:
    if not stop:
      print('Preparing the footstep message')

      # Get current robot pose
      pos1, rot1 = tfListener.lookupTransform("/leftFoot", "/pelvis",rospy.Time())
      pos2, rot2 = tfListener.lookupTransform("/rightFoot", "/pelvis",rospy.Time())
      pos = (np.array(pos1)+np.array(pos2))*0.5
      rot = pm.transformations.quaternion_slerp(rot1,rot2,0.5)
      midFeet = pm.Frame(pm.Rotation.Quaternion(rot[0], rot[1], rot[2], rot[3]), pm.Vector(pos[0], pos[1], pos[2]))
      # release/0.10 changed from ankle to sole frames for specifying footsteps. This is a transform extracted by Doug
      # soleToAnkleFrame.setTranslation(new Vector3D(footLength / 2.0 - footBack, 0.0, -ValkyriePhysicalProperties.ankleHeight))
      # with: footLength = 0.24, footBack = 0.068, ankleHeight = 0.09
      ankleToSoleOffset = pm.Vector(0.24 / 2.0 - 0.068, 0.0, -0.09)
      pose = pm.fromMsg(m.pose.pose)*midFeet.Inverse()*pm.Frame(pm.Rotation(), ankleToSoleOffset)
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
    lstReady = False
    ready = False
    hasStoppedMoving = False
    pauseAt = rospy.get_param('~PauseAtStep', '-1')
    pause = False
    lastStep = -1
    # Load YAML data
    data = load(open(rospy.get_param('~DataFile', '')))
    # Setup ROS node
    
    tfListener = TransformListener()
    pub = rospy.Publisher('/ihmc_ros/valkyrie/control/footstep_list', FootstepDataListRosMessage, queue_size=10)
    pubPause = rospy.Publisher('/ihmc_ros/valkyrie/control/pause_walking', PauseWalkingRosMessage, queue_size=10)
    print('Waiting for robot pose and robot to stop moving...')
    time.sleep(0.5)
    rospy.Subscriber("/ihmc_ros/valkyrie/output/robot_pose", Odometry, callback)
    rospy.Subscriber("/ihmc_ros/valkyrie/output/robot_motion_status", String, status)
    rospy.Subscriber("/ihmc_ros/valkyrie/output/footstep_status", FootstepStatusRosMessage, footStatus)
    while not hasStoppedMoving and not rospy.is_shutdown():
      time.sleep(0.1)
    print('Done')
    if pause:
      if lastStep == pauseAt:
      	print('Stepping paused as requested')
      else:
        print('Pausing failed')
