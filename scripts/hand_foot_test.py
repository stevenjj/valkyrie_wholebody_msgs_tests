#!/usr/bin/env python

# Import YAML
from yaml import load, dump
try:
  from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
  from yaml import Loader, Dumper

from rospy_message_converter import message_converter

import rospy
import numpy as np

from controller_msgs.msg import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Vector3

import tf
from tf import TransformListener
from tf_conversions import transformations
import tf_conversions.posemath as pm


global robot_pose_ready, footsteps_sent, upperbody_trajectory_sent, update_robot_pose_once

robot_pose_ready = False
footsteps_sent = False
upperbody_trajectory_sent = False
update_robot_pose_once = False

# def load_message(self):
#     data = load(open(self.filepath))
#     if self.message_type in ACCEPTED_MESSAGES:
#         self.message = message_converter.convert_dictionary_to_ros_message(self.message_type, data)
#         return True
#     else:
#         print "Error!", self.message_type , " not in list of accepted messages"
#         return False


def robot_pose_callback(msg):
    global robot_pose_ready
    global robot_pose
    global leftFoot
    global rightFoot
    global midFeet
    global rightPalm
    global update_robot_pose_once

    if not(update_robot_pose_once):

        #print "Robot Pose Callback"
        # print msg.pose.pose
        # print "Waiting for Transforms..."
        # Wait for transform
        tfListener.waitForTransform("/pelvis", "/leftCOP_Frame", rospy.Time(), rospy.Duration(4.0))
        tfListener.waitForTransform("/pelvis", "/rightCOP_Frame", rospy.Time(), rospy.Duration(4.0))
        #print "Transforms ready"
        # Get current robot pose
        pos1, rot1 = tfListener.lookupTransform("/pelvis", "/leftCOP_Frame",rospy.Time())
        pos2, rot2 = tfListener.lookupTransform("/pelvis", "/rightCOP_Frame",rospy.Time())
        pos_ra, rot_ra = tfListener.lookupTransform("/pelvis", "/rightPalm",rospy.Time())

        leftFoot = pm.Frame(pm.Rotation.Quaternion(rot1[0], rot1[1], rot1[2], rot1[3]), pm.Vector(pos1[0], pos1[1], pos1[2]))
        rightFoot = pm.Frame(pm.Rotation.Quaternion(rot2[0], rot2[1], rot2[2], rot2[3]), pm.Vector(pos2[0], pos2[1], pos2[2]))
        rightPalm = pm.Frame(pm.Rotation.Quaternion(rot_ra[0], rot_ra[1], rot_ra[2], rot_ra[3]), pm.Vector(pos_ra[0], pos_ra[1], pos_ra[2]))

        # Mid Feet Pose
        mid_pos = (np.array(pos1)+np.array(pos2))*0.5
        mid_rot = pm.transformations.quaternion_slerp(rot1,rot2,0.5)
        midFeet = pm.Frame(pm.Rotation.Quaternion(mid_rot[0], mid_rot[1], mid_rot[2], mid_rot[3]), pm.Vector(mid_pos[0], mid_pos[1], mid_pos[2]))
        
        robot_pose = pm.fromMsg(msg.pose.pose)
        leftFoot = pm.fromMsg(msg.pose.pose)*leftFoot
        rightFoot = pm.fromMsg(msg.pose.pose)*rightFoot
        midFeet = pm.fromMsg(msg.pose.pose)*midFeet
        rightPalm = pm.fromMsg(msg.pose.pose)*rightPalm

        # print "Left Foot Pose"
        # print leftFoot
        # print "Right Foot Pose"
        # print rightFoot
        # print "Mid Feet Pose"
        # print midFeet
        # print "Right Palm Pose"
        # print rightPalm

        robot_pose_ready = True
        update_robot_pose_once = True
    return

def transformSE3(msg):
  global midFeet
  curPos = pm.Frame(pm.Rotation.Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w), pm.Vector(msg.position.x, msg.position.y, msg.position.z))
  curVel = pm.Twist(pm.Vector(msg.linear_velocity.x, msg.linear_velocity.y, msg.linear_velocity.z), pm.Vector(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z))
  pos = midFeet*curPos
  vel = midFeet*curVel
  tmp = pm.toMsg(pos)
  msg.position = tmp.position
  msg.orientation = tmp.orientation
  # Normalize orientation
  quat_original = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
  quat_normalized = quat_original/np.linalg.norm(quat_original)
  # Update orientation
  msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = quat_normalized[0], quat_normalized[1], quat_normalized[2], quat_normalized[3]
  msg.linear_velocity = Vector3(vel.vel.x(), vel.vel.y(), vel.vel.z())
  msg.angular_velocity = Vector3(vel.rot.x(), vel.rot.y(), vel.rot.z())

def transformSO3(msg):
  global midFeet
  curPos = pm.Frame(pm.Rotation.Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w), pm.Vector(0,0,0))
  curVel = pm.Twist(pm.Vector(0,0,0), pm.Vector(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z))
  pos = midFeet*curPos
  vel = midFeet*curVel
  tmp = pm.toMsg(pos)
  msg.orientation = tmp.orientation
  # Normalize orientation
  quat_original = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
  quat_normalized = quat_original/np.linalg.norm(quat_original)
  # Update orientation
  msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = quat_normalized[0], quat_normalized[1], quat_normalized[2], quat_normalized[3]  
  msg.angular_velocity = Vector3(vel.rot.x(), vel.rot.y(), vel.rot.z())


def transform_hand_SE3(msg):
  global robot_pose
  curPos = pm.Frame(pm.Rotation.Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w), pm.Vector(msg.position.x, msg.position.y, msg.position.z))
  curVel = pm.Twist(pm.Vector(msg.linear_velocity.x, msg.linear_velocity.y, msg.linear_velocity.z), pm.Vector(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z))
  pos = robot_pose*curPos
  vel = robot_pose*curVel
  tmp = pm.toMsg(pos)
  msg.position = tmp.position
  msg.orientation = tmp.orientation

  # Normalize orientation
  quat_original = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
  quat_normalized = quat_original/np.linalg.norm(quat_original)
  # Update orientation
  msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = quat_normalized[0], quat_normalized[1], quat_normalized[2], quat_normalized[3]
  msg.linear_velocity = Vector3(vel.vel.x(), vel.vel.y(), vel.vel.z())
  msg.angular_velocity = Vector3(vel.rot.x(), vel.rot.y(), vel.rot.z())

def transformWholeBody(message):
  for msg in message.right_hand_trajectory_message.se3_trajectory.taskspace_trajectory_points:
    transform_hand_SE3(msg)
  for msg in message.left_hand_trajectory_message.se3_trajectory.taskspace_trajectory_points:
    transform_hand_SE3(msg)
  for msg in message.chest_trajectory_message.so3_trajectory.taskspace_trajectory_points:
    transformSO3(msg)
  for msg in message.pelvis_trajectory_message.se3_trajectory.taskspace_trajectory_points:
    transformSE3(msg)
  for msg in message.left_foot_trajectory_message.se3_trajectory.taskspace_trajectory_points:
    transformSE3(msg)
  for msg in message.right_foot_trajectory_message.se3_trajectory.taskspace_trajectory_points:
    transformSE3(msg)

def prepare_wholebody_message(message):
    transformWholeBody(message)    
    # Assign a message id
    message.left_hand_trajectory_message.se3_trajectory.queueing_properties.execution_mode = 0    
    message.right_hand_trajectory_message.se3_trajectory.queueing_properties.execution_mode = 0
    message.left_foot_trajectory_message.se3_trajectory.queueing_properties.execution_mode = 0
    message.right_foot_trajectory_message.se3_trajectory.queueing_properties.execution_mode = 0
    message.chest_trajectory_message.so3_trajectory.queueing_properties.execution_mode = 0
    message.pelvis_trajectory_message.se3_trajectory.queueing_properties.execution_mode = 0
    message.left_arm_trajectory_message.jointspace_trajectory.queueing_properties.execution_mode = 0
    message.right_arm_trajectory_message.jointspace_trajectory.queueing_properties.execution_mode = 0

    # Assign a message id
    message.left_hand_trajectory_message.se3_trajectory.queueing_properties.message_id = 2 #message.left_hand_trajectory_message.sequence_id  
    message.right_hand_trajectory_message.se3_trajectory.queueing_properties.message_id = 2 #message.right_hand_trajectory_message.sequence_id    
    message.left_foot_trajectory_message.se3_trajectory.queueing_properties.message_id = 2 #message.left_foot_trajectory_message.sequence_id    
    message.right_foot_trajectory_message.se3_trajectory.queueing_properties.message_id = 2 #message.right_foot_trajectory_message.sequence_id    
    message.chest_trajectory_message.so3_trajectory.queueing_properties.message_id = 2 #message.chest_trajectory_message.sequence_id
    message.pelvis_trajectory_message.se3_trajectory.queueing_properties.message_id = 2 #message.pelvis_trajectory_message.sequence_id  
    message.left_arm_trajectory_message.jointspace_trajectory.queueing_properties.message_id = 2 # message.left_arm_trajectory_message.sequence_id
    message.right_arm_trajectory_message.jointspace_trajectory.queueing_properties.message_id = 2 # message.right_arm_trajectory_message.sequence_id

def send_wholebody_message():
    global upperbody_trajectory_sent, pubWholeBody, wholebody_data
    print "Preparing Whole Body Message..."
    message = message_converter.convert_dictionary_to_ros_message('controller_msgs/WholeBodyTrajectoryMessage', wholebody_data)
    prepare_wholebody_message(message)

    print "Publishing Whole Body Message..."    
    pubWholeBody.publish(message)
    upperbody_trajectory_sent = True

def send_hand_message():
    global upperbody_trajectory_sent, pubHandTrajectory, hand_data
    print "Preparing Hand Message..."
    message = message_converter.convert_dictionary_to_ros_message('controller_msgs/HandTrajectoryMessage', hand_data)
    message.se3_trajectory.queueing_properties.message_id = 3 #message.left_hand_trajectory_message.sequence_id  
    message.se3_trajectory.queueing_properties.previous_message_id = 2
    message.se3_trajectory.queueing_properties.execution_mode = QueueableMessage().EXECUTION_MODE_QUEUE   

    for msg in message.se3_trajectory.taskspace_trajectory_points:
        transformSE3(msg)

    print "Publishing Hand Message..."    
    pubHandTrajectory.publish(message)
    upperbody_trajectory_sent = True

def send_footsteps():
    global footsteps_sent, pubFootsteps, footsteps_data, midFeet

    # Get footstep trajectopry from YAML 
    message = message_converter.convert_dictionary_to_ros_message('controller_msgs/FootstepDataListMessage', footsteps_data)
    # Update the footstep frames
    numberOfFootstepsInList = len(message.footstep_data_list)
    for step in message.footstep_data_list:
        pstep = midFeet*pm.Frame(pm.Rotation.Quaternion(step.orientation.x, step.orientation.y, step.orientation.z, step.orientation.w), pm.Vector(step.location.x, step.location.y, step.location.z))
        msg = pm.toMsg(pstep)
        step.location = msg.position
        # Normalize orientation
        quat_original = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        quat_normalized = quat_original/np.linalg.norm(quat_original)
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = quat_normalized[0], quat_normalized[1], quat_normalized[2], quat_normalized[3]
        # Update step orientation
        step.orientation = msg.orientation
    print "Relocated footsteps to a local frame"

    message.queueing_properties.message_id = 2
    message.queueing_properties.previous_message_id = 1

    # message.queueing_properties.execution_mode = QueueableMessage().EXECUTION_MODE_QUEUE   
    message.queueing_properties.execution_mode = QueueableMessage().EXECUTION_MODE_OVERRIDE

    print "Sending Footsteps..."
    pubFootsteps.publish(message)
    footsteps_sent = True


def send_stop_trajectories():
    global pubStopTrajectories
    message = StopAllTrajectoryMessage()
    message.sequence_id = 1
    pubStopTrajectories.publish(message)

def send_prepare_for_locomotion():
    global pubPrepareForLocomotion
    message = PrepareForLocomotionMessage()
    message.sequence_id = 1
    message.prepare_manipulation = False
    message.prepare_pelvis = True
    pubPrepareForLocomotion.publish(message)

def footstep_status_callback(msg):
    print "Footstep Status: "

    foot = "Right Foot"
    status = "Started"
    if msg.robot_side == FootstepStatusMessage().ROBOT_SIDE_LEFT:
        foot = "Left Foot"
    if msg.footstep_status == FootstepStatusMessage().FOOTSTEP_STATUS_COMPLETED:
        status = "Completed"

    print "  ", foot, status, "step index:", msg.footstep_index


if __name__ == '__main__':
    global tfListener, pubFootsteps, footsteps_data, pubWholeBody, pubHandTrajectory, pubStopTrajectories, pubPrepareForLocomotion, wholebody_data, footsteps_sent, upperbody_trajectory_sent, hand_data
    rospy.init_node('Valkyrie_Hand_Feet_Test')

    # Load Data
    if not rospy.has_param('~FootstepsDataFile'):
        print('Please specify the FootstepsDataFile parameter (YAML)!')
        exit()
    else:
        # Load YAML data
        footsteps_data = load(open(rospy.get_param('~FootstepsDataFile', '')))

    if not rospy.has_param('~WholeBodyDataFile'):
        print('Please specify the WholeBodyDataFile parameter (YAML)!')
        exit()
    else:
        # Load YAML data
        wholebody_data = load(open(rospy.get_param('~WholeBodyDataFile', '')))

    if not rospy.has_param('~HandDataFile'):
        print('Please specify the HandDataFile parameter (YAML)!')
        exit()
    else:
        # Load YAML data
        hand_data = load(open(rospy.get_param('~HandDataFile', '')))
    

    tfListener = TransformListener()
    rospy.Subscriber("/ihmc_ros/valkyrie/output/robot_pose", Odometry, robot_pose_callback)
    rospy.Subscriber("/ihmc/valkyrie/humanoid_control/output/footstep_status", FootstepStatusMessage, footstep_status_callback)

    pubFootsteps = rospy.Publisher('/ihmc/valkyrie/humanoid_control/input/footstep_data_list', FootstepDataListMessage, queue_size=10, latch=True)
    pubWholeBody = rospy.Publisher('/ihmc/valkyrie/humanoid_control/input/whole_body_trajectory', WholeBodyTrajectoryMessage, queue_size=10, latch=True)
    pubStopTrajectories = rospy.Publisher('/ihmc/valkyrie/humanoid_control/input/stop_all_trajectory', StopAllTrajectoryMessage, queue_size=10, latch=True )
    pubHandTrajectory = rospy.Publisher('/ihmc/valkyrie/humanoid_control/input/hand_trajectory', HandTrajectoryMessage, queue_size=10, latch=True)
    pubPrepareForLocomotion = rospy.Publisher('/ihmc/valkyrie/humanoid_control/input/prepare_for_locomotion', PrepareForLocomotionMessage, queue_size=10, latch=True)

    while not rospy.is_shutdown():
        if not(robot_pose_ready):
            print "Waiting for robot pose..."
            print "  Robot Pose Status:", robot_pose_ready

        if robot_pose_ready and not footsteps_sent:
            print "Robot Pose is Ready."
            # print "  Sending Stop All Trajectories"
            # send_stop_trajectories()
            
            print "  Sending Prepare For Locomotion Message"
            send_prepare_for_locomotion()
            rospy.sleep(1.0)

            print "  Preparing Footsteps Message..."
            send_footsteps()
            rospy.sleep(0.1)

            print "  Preparing Hand Message.."
            send_hand_message()            
            rospy.sleep(1.0)

        rospy.sleep(0.1)

        if footsteps_sent and upperbody_trajectory_sent:
            rospy.sleep(5.0)
            break

        continue

    print "Program Ends"

'''      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyTransform controlFrameToWristTransform = new RigidBodyTransform();
         controlFrameToWristTransform.setTranslation(0.025, robotSide.negateIfRightSide(0.07), 0.0);
         controlFrameToWristTransform.appendYawRotation(robotSide.negateIfRightSide(Math.PI * 0.5));
         handControlFrameToWristTransforms.put(robotSide, controlFrameToWristTransform);
      }
   }
'''
