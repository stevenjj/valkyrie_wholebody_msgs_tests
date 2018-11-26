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

GAZEBO_ENV = False
def check_environment_variables():
  global GAZEBO_ENV
  if ('IS_GAZEBO' in os.environ and (os.environ['IS_GAZEBO'] == 'true')):
    print 'shell environment variable IS_GAZEBO is set to true'
    GAZEBO_ENV = True
  else:
    GAZEBO_ENV = False

def status(m):
  global ready
  global stop
  global GAZEBO_ENV
  global lstReady

  if GAZEBO_ENV:
    ready = True  
    lstReady = ready
  else:
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
    if m.footstep_status == 0 and m.footstep_index >= pauseAt:
      print('Pausing the walking ...')
      pause = True
      message = PauseWalkingMessage()
      message.pause = True #    message.sequence_id = 1
      pubPause.publish(message)
  if m.footstep_status == 1:
     lastStep = m.footstep_index

def callback(m):
  global pub
  global stop
  global data
  global tfListener
  global numberOfFootstepsInList
  if ready:
    if not stop:
      print('Preparing the footstep message')
      # Wait for transform
      tfListener.waitForTransform("/pelvis", "/leftFoot", rospy.Time(), rospy.Duration(4.0))
      tfListener.waitForTransform("/pelvis", "/rightFoot", rospy.Time(), rospy.Duration(4.0))

      # Get current robot pose
      pos1, rot1 = tfListener.lookupTransform("/pelvis", "/leftFoot",rospy.Time())
      pos2, rot2 = tfListener.lookupTransform("/pelvis", "/rightFoot",rospy.Time())

      pos = (np.array(pos1)+np.array(pos2))*0.5
      rot = pm.transformations.quaternion_slerp(rot1,rot2,0.5)
      midFeet = pm.Frame(pm.Rotation.Quaternion(rot[0], rot[1], rot[2], rot[3]), pm.Vector(pos[0], pos[1], pos[2]))


      # From IHMC's ValkyriePhysicalProperties.java:
      # soleToAnkleFrame.setTranslation(new Vector3D(footLength / 2.0 - footBack, 0.0, -ValkyriePhysicalProperties.ankleHeight));
      footsizeReduction = 0.01
      footLength = 0.25 - footsizeReduction
      ankleHeight = 0.09
      footBack = 0.073 - footsizeReduction/2.0

      ankleToSoleOffset = pm.Vector(footLength/2.0 - footBack, 0.0, -ankleHeight)
      pose = pm.fromMsg(m.pose.pose)*midFeet*pm.Frame(pm.Rotation(), ankleToSoleOffset)

      # Get footstep trajectopry from YAML 
      message = message_converter.convert_dictionary_to_ros_message('controller_msgs/FootstepDataListMessage', data)
      # Update the footstep frames
      numberOfFootstepsInList = len(message.footstep_data_list)
      for step in message.footstep_data_list:
        pstep = pose*pm.Frame(pm.Rotation.Quaternion(step.orientation.x, step.orientation.y, step.orientation.z, step.orientation.w), pm.Vector(step.location.x, step.location.y, step.location.z))
        msg = pm.toMsg(pstep)
        step.location = msg.position
        # Normalize orientation
        quat_original = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        quat_normalized = quat_original/np.linalg.norm(quat_original)
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = quat_normalized[0], quat_normalized[1], quat_normalized[2], quat_normalized[3]
        # Update step orientation
        step.orientation = msg.orientation
      print('Relocated footsteps to a local frame')
      print('Publishing...')
      pub.publish(message)
      stop = True
      print('Waiting for execution...')

if __name__ == '__main__':
  check_environment_variables()  
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
    numberOfStatusesReceived = 0
    numberOfFootstepsInList = 100
    # Load YAML data
    data = load(open(rospy.get_param('~DataFile', '')))
    #data = load(open('/home/val/uoe_ws/src/valkyrie_testing_edi/data/converted_walk_turn_ccw.yaml'))
    # Setup ROS node  

    tfListener = TransformListener()
    pub = rospy.Publisher('/ihmc/valkyrie/humanoid_control/input/footstep_data_list', FootstepDataListMessage, queue_size=10, latch=True)
    pubPause = rospy.Publisher('/ihmc/valkyrie/humanoid_control/input/pause_walking', PauseWalkingMessage, queue_size=10, latch=True)
    print('Waiting for robot pose and robot to stop moving...')
    time.sleep(0.5)
    rospy.Subscriber("/ihmc_ros/valkyrie/output/robot_pose", Odometry, callback)
    rospy.Subscriber("/ihmc_ros/valkyrie/output/robot_motion_status", String, status)
    rospy.Subscriber("/ihmc/valkyrie/humanoid_control/output/footstep_status", FootstepStatusMessage, footStatus)

    # Parse Test
    # print('Testing message parse')
    # message = message_converter.convert_dictionary_to_ros_message('controller_msgs/FootstepDataListMessage', data)    
    # pub.publish(message)
    # print('Published parsed message test')
    # End Parse test


    while not hasStoppedMoving and not rospy.is_shutdown():
        time.sleep(0.1)
    print('Done')
    if pause:
      if lastStep == pauseAt:
      	print('Stepping paused as requested')
      else:
        print('Pausing failed')
