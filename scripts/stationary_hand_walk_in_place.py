#!/usr/bin/env python

# Import YAML
from yaml import load, dump
try:
  from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
  from yaml import Loader, Dumper
# Import Message Converter
from rospy_message_converter import message_converter
# Import Rospy and numpy
import rospy
# Import Messages
from controller_msgs.msg import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Vector3
# Import TFs
import tf
from tf import TransformListener
# Import Math
from tf_conversions import transformations
import tf_conversions.posemath as pm
import numpy as np


class StationaryHandWhileWalkingExecutor:
    def __init__(self):
        self.robot_pose = pm.Frame(pm.Rotation.Quaternion(0, 0, 0, 1), pm.Vector(0.0, 0.0, 0.0))
        self.mid_feet_pose = pm.Frame(pm.Rotation.Quaternion(0, 0, 0, 1), pm.Vector(0.0, 0.0, 0.0))

        self.right_hand_pose = pm.Frame(pm.Rotation.Quaternion(0, 0, 0, 1), pm.Vector(0.0, 0.0, 0.0))
        self.left_hand_pose = pm.Frame(pm.Rotation.Quaternion(0, 0, 0, 1), pm.Vector(0.0, 0.0, 0.0))

        self.right_hand_ihmc_offset = pm.Frame(pm.Rotation.RPY(0.0, 0.0, -np.pi/2.0), pm.Vector(0.025, -0.07, 0.0))
        self.left_hand_ihmc_offset = pm.Frame(pm.Rotation.RPY(0.0, 0.0, np.pi/2.0), pm.Vector(0.025, 0.07, 0.0))

        self.footsteps_msg = FootstepDataListMessage()
        self.hand_msg = HandTrajectoryMessage()

        self.robot_pose_ready = False
        self.update_robot_pose_once = False
        self.messages_sent = False
        self.completed_footsteps = False

    def initialize(self):
        # Declare Subsrcibers
        self.robot_pose_sub = rospy.Subscriber("/ihmc_ros/valkyrie/output/robot_pose", Odometry, self.robot_pose_callback)
        self.footstep_status_sub =  rospy.Subscriber("/ihmc/valkyrie/humanoid_control/output/footstep_status", FootstepStatusMessage, self.footstep_status_callback)
        # Declare Publishers
        self.pubPrepareForLocomotion = rospy.Publisher('/ihmc/valkyrie/humanoid_control/input/prepare_for_locomotion', PrepareForLocomotionMessage, queue_size=10, latch=True)
        self.pubStopAllTrajectories = rospy.Publisher('/ihmc/valkyrie/humanoid_control/input/stop_all_trajectory', StopAllTrajectoryMessage, queue_size=10, latch=True)

        self.pubFootsteps = rospy.Publisher('/ihmc/valkyrie/humanoid_control/input/footstep_data_list', FootstepDataListMessage, queue_size=10, latch=True)
        self.pubHandTrajectory = rospy.Publisher('/ihmc/valkyrie/humanoid_control/input/hand_trajectory', HandTrajectoryMessage, queue_size=10, latch=True)

        # Declare a TF Listener
        self.tfListener = TransformListener()
    
        # Get Footsteps Data
        if rospy.has_param('~FootstepsDataFile'):
            # Load YAML data
            self.footsteps_data = load(open(rospy.get_param('~FootstepsDataFile', '')))
            self.footsteps_msg = message_converter.convert_dictionary_to_ros_message('controller_msgs/FootstepDataListMessage', self.footsteps_data)
        else:
            print('Please specify the FootstepsDataFile ROS parameter (YAML)! Check the launch file for this script.')
            exit()

        # Get Hand Data
        if rospy.has_param('~HandDataFile'):
            # Load YAML data
            self.hand_data = load(open(rospy.get_param('~HandDataFile', '')))
            self.hand_msg = message_converter.convert_dictionary_to_ros_message('controller_msgs/HandTrajectoryMessage', self.hand_data)
        else:
            print('Please specify the HandDataFile ROS parameter (YAML)! Check the launch file for this script.')
            exit()

        rospy.loginfo("Successful Initialization")
        print "Number of Footsteps: ", len(self.footsteps_msg.footstep_data_list)


    def robot_pose_callback(self, msg):
        if not self.update_robot_pose_once: 
            try:
                #print "Transforms ready"
                # Get current robot pose
                now = rospy.Time()
                self.tfListener.waitForTransform("/pelvis", "/leftCOP_Frame", now, rospy.Duration(4.0))
                self.tfListener.waitForTransform("/pelvis", "/rightCOP_Frame", now, rospy.Duration(4.0))
                self.tfListener.waitForTransform("/pelvis", "/rightPalm", now, rospy.Duration(4.0))
                self.tfListener.waitForTransform("/pelvis", "/leftPalm", now, rospy.Duration(4.0))

                pos1, rot1 = self.tfListener.lookupTransform("/pelvis", "/leftCOP_Frame", now)
                pos2, rot2 = self.tfListener.lookupTransform("/pelvis", "/rightCOP_Frame", now)
                pos_rp, rot_rp = self.tfListener.lookupTransform("/pelvis", "/rightPalm", now)
                pos_lp, rot_lp = self.tfListener.lookupTransform("/pelvis", "/leftPalm", now)

                leftFoot = pm.Frame(pm.Rotation.Quaternion(rot1[0], rot1[1], rot1[2], rot1[3]), pm.Vector(pos1[0], pos1[1], pos1[2]))
                rightFoot = pm.Frame(pm.Rotation.Quaternion(rot2[0], rot2[1], rot2[2], rot2[3]), pm.Vector(pos2[0], pos2[1], pos2[2]))
                rightPalm = pm.Frame(pm.Rotation.Quaternion(rot_rp[0], rot_rp[1], rot_rp[2], rot_rp[3]), pm.Vector(pos_rp[0], pos_rp[1], pos_rp[2]))
                leftPalm = pm.Frame(pm.Rotation.Quaternion(rot_lp[0], rot_lp[1], rot_lp[2], rot_lp[3]), pm.Vector(pos_lp[0], pos_lp[1], pos_lp[2]))

                # Mid Feet Pose
                mid_pos = (np.array(pos1)+np.array(pos2))*0.5
                mid_rot = pm.transformations.quaternion_slerp(rot1,rot2,0.5)
                midFeet = pm.Frame(pm.Rotation.Quaternion(mid_rot[0], mid_rot[1], mid_rot[2], mid_rot[3]), pm.Vector(mid_pos[0], mid_pos[1], mid_pos[2]))
                
                self.robot_pose = pm.fromMsg(msg.pose.pose)
                self.mid_feet_pose = pm.fromMsg(msg.pose.pose)*midFeet
                self.right_hand_pose = pm.fromMsg(msg.pose.pose)*rightPalm*self.right_hand_ihmc_offset
                self.left_hand_pose = pm.fromMsg(msg.pose.pose)*leftPalm*self.left_hand_ihmc_offset
                
                self.robot_pose_ready = True
                self.update_robot_pose_once = True
                rospy.loginfo("Robot Pose is Ready!")
            except:
                rospy.logerr("Could not get the Robot pose. Will try again at the next callback.")


    def footstep_status_callback(self, msg):
        print "Footstep Status: "
        foot = "Right Foot"
        status = "Started"
        if msg.robot_side == FootstepStatusMessage().ROBOT_SIDE_LEFT:
            foot = "Left Foot"
        if msg.footstep_status == FootstepStatusMessage().FOOTSTEP_STATUS_COMPLETED:
            status = "Completed"

        #print "  ", foot, status, "step index:", msg.footstep_index
        rospy.loginfo("  %s %s with step index: %i", foot, status, msg.footstep_index)

        if status == "Completed" and self.messages_sent and (msg.footstep_index == len(self.footsteps_msg.footstep_data_list)):
            self.completed_footsteps = True
            rospy.loginfo("Completed all the footsteps!")


    def transformSE3_hand(self, msg, side):
      curPos = pm.Frame(pm.Rotation.Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w), pm.Vector(msg.position.x, msg.position.y, msg.position.z))
      curVel = pm.Twist(pm.Vector(msg.linear_velocity.x, msg.linear_velocity.y, msg.linear_velocity.z), pm.Vector(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z))
      pos = curPos
      vel = curVel
      if side == HandTrajectoryMessage().ROBOT_SIDE_LEFT:
          pos = self.left_hand_pose*curPos
          vel = self.left_hand_pose*curVel        
      else:
          pos = self.right_hand_pose*curPos
          vel = self.right_hand_pose*curVel        
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

    def send_hand_message(self):
        rospy.loginfo("Preparing Hand Message...")        
        self.hand_msg.se3_trajectory.queueing_properties.message_id = 2  
        self.hand_msg.se3_trajectory.queueing_properties.previous_message_id = 1
        # Override also works, but we're leaving it to queue as it is the correct implementation
        self.hand_msg.se3_trajectory.queueing_properties.execution_mode = QueueableMessage().EXECUTION_MODE_QUEUE  

        for msg in self.hand_msg.se3_trajectory.taskspace_trajectory_points:
            self.transformSE3_hand(msg, self.hand_msg.robot_side)

        rospy.loginfo("Publishing Hand Message...")
        self.pubHandTrajectory.publish(self.hand_msg)

    def send_footsteps(self):
        for step in self.footsteps_msg.footstep_data_list:
            pstep = self.mid_feet_pose*pm.Frame(pm.Rotation.Quaternion(step.orientation.x, step.orientation.y, step.orientation.z, step.orientation.w), pm.Vector(step.location.x, step.location.y, step.location.z))
            msg = pm.toMsg(pstep)
            step.location = msg.position
            # Normalize orientation
            quat_original = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
            quat_normalized = quat_original/np.linalg.norm(quat_original)
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = quat_normalized[0], quat_normalized[1], quat_normalized[2], quat_normalized[3]
            # Update step orientation
            step.orientation = msg.orientation
        rospy.loginfo("Relocated footsteps to a local frame")

        self.footsteps_msg.queueing_properties.message_id = 1
        self.footsteps_msg.queueing_properties.execution_mode = QueueableMessage().EXECUTION_MODE_OVERRIDE

        rospy.loginfo("Publishing Footstep Data List...")
        self.pubFootsteps.publish(self.footsteps_msg)

    def send_prepare_for_locomotion(self):
        rospy.loginfo("Sending PrepareForLocomotionMessage...")
        message = PrepareForLocomotionMessage()
        message.sequence_id = 1
        message.prepare_manipulation = False
        message.prepare_pelvis = True
        self.pubPrepareForLocomotion.publish(message)

    def send_stop_trajectories(self):
        rospy.loginfo("Sending StopAllTrajectoryMessage...")
        message = StopAllTrajectoryMessage()
        message.sequence_id = 1
        self.pubStopAllTrajectories.publish(message)        

    def send_messages(self):
       self.send_stop_trajectories()
       rospy.sleep(1.0) # 1.0
       self.send_prepare_for_locomotion()
       rospy.sleep(1.0) # 1.0
       self.send_footsteps()
       self.send_hand_message()
       rospy.sleep(1.0)

       self.messages_sent = True        

    def run(self):
        while not rospy.is_shutdown():
            if self.robot_pose_ready and not self.messages_sent:
                self.send_messages()
            if self.messages_sent and self.completed_footsteps:
                break

            rospy.sleep(0.1)

if __name__ == '__main__':
    rospy.init_node('Valkyrie_Hand_Feet_Test')
    executor_object = StationaryHandWhileWalkingExecutor()
    executor_object.initialize()
    executor_object.run()

    print "Program Ends"
