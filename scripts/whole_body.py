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
  global pubStop
  t0 = robotTime
  while robotTime-t0<t and not rospy.is_shutdown():
    if pause and pauseAt>0.0 and pauseAt<robotTime-t0:
      msg=StopAllTrajectoryRosMessage()
      msg.unique_id = 1
      pubStop.publish(msg)
      break
    time.sleep(0.01)

def extract_first(val, t):
  global maxT
  if len(val)>0:
    maxT = max(maxT, val[-1].time)
    del val[1:len(val)]
    val[0].time=t

def transformSE3(msg):
  global pose
  curPos = pm.Frame(pm.Rotation.Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w), pm.Vector(msg.position.x, msg.position.y, msg.position.z))
  curVel = pm.Twist(pm.Vector(msg.linear_velocity.x, msg.linear_velocity.y, msg.linear_velocity.z), pm.Vector(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z))
  pos = pose*curPos
  vel = pose*curVel
  tmp = pm.toMsg(pos)
  msg.position = tmp.position
  msg.orientation = tmp.orientation
  msg.linear_velocity = Vector3(vel.vel.x(), vel.vel.y(), vel.vel.z())
  msg.angular_velocity = Vector3(vel.rot.x(), vel.rot.y(), vel.rot.z())

def transformSO3(msg):
  global pose
  curPos = pm.Frame(pm.Rotation.Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w), pm.Vector(0,0,0))
  curVel = pm.Twist(pm.Vector(0,0,0), pm.Vector(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z))
  pos = pose*curPos
  vel = pose*curVel
  tmp = pm.toMsg(pos)
  msg.orientation = tmp.orientation
  msg.angular_velocity = Vector3(vel.rot.x(), vel.rot.y(), vel.rot.z())


def getEndposeTrajectory(msg, t):
  message = deepcopy(msg)
  extract_first(message.left_hand_trajectory_message.taskspace_trajectory_points, t)
  extract_first(message.right_hand_trajectory_message.taskspace_trajectory_points, t)
  extract_first(message.left_foot_trajectory_message.taskspace_trajectory_points, t)
  extract_first(message.right_foot_trajectory_message.taskspace_trajectory_points, t)
  extract_first(message.chest_trajectory_message.taskspace_trajectory_points, t)
  extract_first(message.pelvis_trajectory_message.taskspace_trajectory_points, t)
  for traj in message.left_arm_trajectory_message.joint_trajectory_messages:
    extract_first(traj.trajectory_points, t)
  for traj in message.right_arm_trajectory_message.joint_trajectory_messages:
    extract_first(traj.trajectory_points, t)
  for traj in message.spine_trajectory_message.joint_trajectory_messages:
    extract_first(traj.trajectory_points, t)

  message.left_hand_trajectory_message.execution_mode = 0	
  message.right_hand_trajectory_message.execution_mode = 0
  message.left_foot_trajectory_message.execution_mode = 0
  message.right_foot_trajectory_message.execution_mode = 0
  message.chest_trajectory_message.execution_mode = 0
  message.pelvis_trajectory_message.execution_mode = 0
  message.left_arm_trajectory_message.execution_mode = 0
  message.right_arm_trajectory_message.execution_mode = 0
  message.spine_trajectory_message.execution_mode = 0

  return message

def transformWholeBody(message):
  for msg in message.chest_trajectory_message.taskspace_trajectory_points:
    transformSO3(msg)
  for msg in message.pelvis_trajectory_message.taskspace_trajectory_points:
    transformSE3(msg)
  for msg in message.left_foot_trajectory_message.taskspace_trajectory_points:
    transformSE3(msg)
  for msg in message.right_foot_trajectory_message.taskspace_trajectory_points:
    transformSE3(msg)

def callback(m):
  global pubWhole
  global stop
  global data
  global robotTime
  global pose
  global tfListener
  robotTime = m.header.stamp.to_sec()
  if ready:
    if not stop:
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
      stop = True

if __name__ == '__main__':
  rospy.init_node('ValkyrieShakeout')
  if not rospy.has_param('~DataFile'):
    print('Please specify the data file (YAML)!')
  else:
    stop = False
    ready = False
    pauseAt = float(rospy.get_param('~PauseAt', '-1'))
    prepTime = float(rospy.get_param('~PrepTime', '3.0'))
    pause = False
    lastStep = -1
    # Load YAML data
    data = load(open(rospy.get_param('~DataFile', '')))
    # Setup ROS node
    robotTime = 0
    
    tfListener = TransformListener()
    pubWhole = rospy.Publisher('/ihmc_ros/valkyrie/control/whole_body_trajectory', WholeBodyTrajectoryRosMessage, queue_size=10)
    pubStop = rospy.Publisher('/ihmc_ros/valkyrie/control/stop_all_trajectories', StopAllTrajectoryRosMessage, queue_size=10)
    print('Waiting for robot pose and robot to stop moving...')
    time.sleep(0.5)
    rospy.Subscriber("/ihmc_ros/valkyrie/output/robot_pose", Odometry, callback)
    rospy.Subscriber("/ihmc_ros/valkyrie/output/robot_motion_status", String, status)
    while not rospy.is_shutdown():
      if stop:
        print('Preparing the footstep message')
        # Get footstep trajectopry from YAML 
        message = message_converter.convert_dictionary_to_ros_message('ihmc_msgs/WholeBodyTrajectoryRosMessage', data)
        transformWholeBody(message)
        print('Executing prep move...')
        maxT = 0.0
        msg = getEndposeTrajectory(message, prepTime)
        pubWhole.publish(msg)
        wait(prepTime)
        print('Executing whole body trajectory...')
        pubWhole.publish(message)
        print('Waiting for execution...')
        wait(maxT, True)
        break
      else:
        time.sleep(0.1)
    print('Done')
