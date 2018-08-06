#!/usr/bin/env python

# Import YAML
from yaml import load, dump
try:
  from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
  from yaml import Loader, Dumper

# Import ROS Messages
from controller_msgs.msg import *
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Point, Quaternion, Vector3

# Import ROS Utilities
from rospy_message_converter import message_converter
import tf
from tf import TransformListener
from tf_conversions import transformations
import tf_conversions.posemath as pm

# Import Standard Python Modules
import sys
import os
import time
import numpy as np

import rospy

# Define accepted types
ACCEPTED_MESSAGES = ["controller_msgs/GoHomeMessage",
					 "controller_msgs/NeckTrajectoryMessage",
					 "controller_msgs/FootstepDataListMessage",
					 "controller_msgs/FootLoadBearingMessage",
					 "controller_msgs/WholeBodyTrajectoryMessage"]

#"controller_msgs/SpineTrajectoryMessage"

# Define states
STATE_LOAD_MESSAGE = 0
STATE_UPDATE_ROBOT_POSE = 1
STATE_PUBLISH_MSG = 2
STATE_WAIT_FOR_EXECUTION = 3
STATE_TERMINATE_PROGRAM = 4
STATE_INDEX_TO_NAME = {0 : "STATE_LOAD_MESSAGE", 
					   1 : "STATE_UPDATE_ROBOT_POSE", 
					   2 : "STATE_PUBLISH_MSG", 
					   3 : "STATE_WAIT_FOR_EXECUTION", 
					   4: "STATE_TERMINATE_PROGRAM"}

ROBOT_STATUSES = ["STANDING", "IN_MOTION"]

class TestParams:
	def __init__(self):
		self.filepath = ""
		self.wait_time_after_publishing = 0.0
		self.update_pose = True
		self.queued_message = False # whether or not the message is part of a queue
		self.pause_at_step = -1 # If negative, steps will not be paused
		self.pause_at_time = -1.0 # If negative, the trajectory will not be paused
		self.message_type = "unspecified message type"
		self.message = None

	def load_message(self):
		data = load(open(self.filepath))
		if self.message_type in ACCEPTED_MESSAGES:
			self.message = message_converter.convert_dictionary_to_ros_message(self.message_type, data)
			return True
		else:
			print "Error!", self.message_type , " not in list of accepted messages"
			return False

class Test_Suite_State_Machine:
	def __init__(self):
		# Robot related Variables
		self.robot_status = "Unknown Status"
		self.robot_pose = pm.Frame(pm.Rotation.Quaternion(0, 0, 0, 1), pm.Vector(0.0, 0.0, 0.0))
		self.robot_time = None
		self.updated_robot_pose = False
		self.robot_status = "unspecified status"
		self.robot_stopped_moving = False

		self.footstep_pause_interrupt = False
		self.state = STATE_LOAD_MESSAGE # Initialize state to load message	

		self.list_of_tests = []
		self.current_test = None
		self.test_index = -1 # Starts at invalid index

		self.package_path = ""

	def set_package_path(self, package_path_input):
		self.package_path = package_path_input

	def load_data(self, data): 
		# Load the test
		print("Loading the test suite data file...")

		for test_info in data['list_of_tests']:
			print "  Test has params: ", test_info
			test_params = TestParams()
			# Populate Fields
			test_params.filepath = self.package_path + "/" + test_info['filepath']
			test_params.message_type =  test_info['message_type']

			if 'wait_time_after_publishing' in test_info:
				test_params.wait_time_after_publishing =  test_info['wait_time_after_publishing']
			else:
				test_params.wait_time_after_publishing = test_info['default_wait_time_after_publishing']				

			if 'update_pose' in test_info:
				test_params.update_pose = test_info['update_pose']
			if 'pause_at_step' in test_info:
				test_params.pause_at_step =  test_info['pause_at_step']
			if 'pause_at_time' in test_info:
				test_params.pause_at_time =  test_info['pause_at_time']
			if 'queued_message' in test_info:
				test_params.queued_message =  test_info['queued_message']

			try:
				if test_params.load_message():
					self.list_of_tests.append(test_params)
					print "Successfully Loaded test ", test_params.filepath
				else:
					return False
			except Exception as error_message:
				print "Error in loading the yaml file message! "
				print error_message
				print " "
				return False

		# Set index to valid number
		if len(self.list_of_tests) > 0:
			self.test_index = 0
		return True

	def robot_pose_callback(self, msg):
		# Update robot time
		self.robot_time = msg.header.stamp.to_sec()		

		# Gets the SE3 transform from world to the midfeet frame.
		if self.state == STATE_UPDATE_ROBOT_POSE and not(self.updated_robot_pose):
			print ('  Robot Pose Callback')
			# Get current robot pose
			pos1, rot1 = tfListener.lookupTransform("/leftFoot", "/pelvis",rospy.Time())
			pos2, rot2 = tfListener.lookupTransform("/rightFoot", "/pelvis",rospy.Time())
			pos = (np.array(pos1)+np.array(pos2))*0.5
			rot = pm.transformations.quaternion_slerp(rot1,rot2,0.5)
			midFeet = pm.Frame(pm.Rotation.Quaternion(rot[0], rot[1], rot[2], rot[3]), pm.Vector(pos[0], pos[1], pos[2]))
			ankleToSoleOffset = pm.Vector(0.24 / 2.0 - 0.068, 0.0, -0.09)
			self.robot_pose = pm.fromMsg(msg.pose.pose)*midFeet.Inverse()*pm.Frame(pm.Rotation(), ankleToSoleOffset)			

			'''
			# Ideally we look up the CoP frames correctly.
			pos1, rot1 =  tfListener.lookupTransform("/pelvis", "/leftFootCoPFrame", rospy.Time())
			pos2, rot2 =  tfListener.lookupTransform("/pelvis", "/rightFootCoPFrame", rospy.Time())
			# Then the robot pose should be:
			pos = (np.array(pos1)+np.array(pos2))*0.5
			rot = pm.transformations.quaternion_slerp(rot1,rot2,0.5)
			midFeet = pm.Frame(pm.Rotation.Quaternion(rot[0], rot[1], rot[2], rot[3]), pm.Vector(pos[0], pos[1], pos[2]))			
			self.robot_pose = pm.fromMsg(m.pose.pose)*midFeet		
			'''
			self.updated_robot_pose = True

		return

	def robot_motion_status_callback(self, msg):
		self.robot_status = msg.data
		if self.robot_status == "STANDING":
			self.robot_stopped_moving = True
		else:
			self.robot_stopped_moving = False

	def footstep_status_callback(self, msg):
		global pubPauseWalking
		# Check footstep status
		# If pause is requested. Do an interrupt and publish stop message then wait()
		# change state to load next test
		if (self.current_test.message_type == "controller_msgs/FootstepDataListMessage"):
			if self.current_test.pause_at_step > 0:
				if msg.footstep_status == 0 and msg.footstep_index >= self.current_test.pause_at_step:
					print('Pausing the walking ...')
					pause = True
					message = PauseWalkingMessage()
					message.pause = True     
					message.sequence_id = 10
					pubPauseWalking.publish(message)

					# Change state and enable an interrupt
					self.footstep_pause_interrupt = True

	# Data is written w.r.t Midfoot frame. These transformations will express position and orientation w.r.t World Frame 
	def transformSO3(self, msg):
		curPos = pm.Frame(pm.Rotation.Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w), pm.Vector(0,0,0))
		curVel = pm.Twist(pm.Vector(0,0,0), pm.Vector(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z))
		pos = self.robot_pose*curPos
		vel = self.robot_pose*curVel
		tmp = pm.toMsg(pos)
		msg.orientation = tmp.orientation
		# Normalize orientation
		quat_original = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
		quat_normalized = quat_original/np.linalg.norm(quat_original)
		# Update orientation
		msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = quat_normalized[0], quat_normalized[1], quat_normalized[2], quat_normalized[3]
		msg.angular_velocity = Vector3(vel.rot.x(), vel.rot.y(), vel.rot.z())

	def transformSE3(self, msg):
		curPos = pm.Frame(pm.Rotation.Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w), pm.Vector(msg.position.x, msg.position.y, msg.position.z))
		curVel = pm.Twist(pm.Vector(msg.linear_velocity.x, msg.linear_velocity.y, msg.linear_velocity.z), pm.Vector(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z))
		pos = self.robot_pose*curPos
		vel = self.robot_pose*curVel
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

	def transformFootsteps(self, msg):
		numberOfFootstepsInList = len(msg.footstep_data_list)
		for step in msg.footstep_data_list:
			pstep = self.robot_pose*pm.Frame(pm.Rotation.Quaternion(step.orientation.x, step.orientation.y, step.orientation.z, step.orientation.w), pm.Vector(step.location.x, step.location.y, step.location.z))
			msg = pm.toMsg(pstep)
			step.location = msg.position
			# Normalize orientation
			quat_original = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
			quat_normalized = quat_original/np.linalg.norm(quat_original)
			msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = quat_normalized[0], quat_normalized[1], quat_normalized[2], quat_normalized[3]
			# Update step orientation
			step.orientation = msg.orientation

	def prepare_wholebody_msg(self, msg):
		print "Transforming wholebody messages to world frame as needed..."
		for this_message in msg.chest_trajectory_message.so3_trajectory.taskspace_trajectory_points:
			self.transformSO3(this_message)
		for this_message in msg.pelvis_trajectory_message.se3_trajectory.taskspace_trajectory_points:
			self.transformSE3(this_message)
		for this_message in msg.left_foot_trajectory_message.se3_trajectory.taskspace_trajectory_points:
			self.transformSE3(this_message)
		for this_message in msg.right_foot_trajectory_message.se3_trajectory.taskspace_trajectory_points:
			self.transformSE3(this_message)

		print "Enforcing all of these messages to have an execution mode of OVERRIDE"
		msg.left_hand_trajectory_message.se3_trajectory.queueing_properties.execution_mode = 0	
		msg.right_hand_trajectory_message.se3_trajectory.queueing_properties.execution_mode = 0
		msg.left_foot_trajectory_message.se3_trajectory.queueing_properties.execution_mode = 0
		msg.right_foot_trajectory_message.se3_trajectory.queueing_properties.execution_mode = 0
		msg.chest_trajectory_message.so3_trajectory.queueing_properties.execution_mode = 0
		msg.pelvis_trajectory_message.se3_trajectory.queueing_properties.execution_mode = 0
		msg.left_arm_trajectory_message.jointspace_trajectory.queueing_properties.execution_mode = 0
		msg.right_arm_trajectory_message.jointspace_trajectory.queueing_properties.execution_mode = 0

		print "Enforcing all of these messages to have an identical message id"
		msg.left_hand_trajectory_message.se3_trajectory.queueing_properties.message_id = 2  
		msg.right_hand_trajectory_message.se3_trajectory.queueing_properties.message_id = 2    
		msg.left_foot_trajectory_message.se3_trajectory.queueing_properties.message_id = 2   
		msg.right_foot_trajectory_message.se3_trajectory.queueing_properties.message_id = 2  
		msg.chest_trajectory_message.so3_trajectory.queueing_properties.message_id = 2 
		msg.pelvis_trajectory_message.se3_trajectory.queueing_properties.message_id = 2
		msg.left_arm_trajectory_message.jointspace_trajectory.queueing_properties.message_id = 2
		msg.right_arm_trajectory_message.jointspace_trajectory.queueing_properties.message_id = 2 


	# -----------------------------------------------------------------------------------------------
	def output_status(self):
		print "  Running Test", (self.test_index + 1) , "out of", len(self.list_of_tests), "tests"

	def change_state_to(self, new_state):
		self.state = new_state
		print "  Changing state to ", STATE_INDEX_TO_NAME[self.state]

	# Processing Logic Definition of each state 
	def process_state_load_message(self):
		self.disable_interrupts()
		# We have Programs to run
		if ((self.test_index >= 0) and self.test_index < len(self.list_of_tests)):
			self.current_test = self.list_of_tests[self.test_index]
			print "  Loaded yaml file:", self.current_test.filepath
			print "    with message type:", self.current_test.message_type			

			# Check if this test wants to update the pose or not
			if self.current_test.update_pose:
				self.change_state_to(STATE_UPDATE_ROBOT_POSE)					
			else:
				self.change_state_to(STATE_PUBLISH_MSG)
		# We don't have tests to run
		elif (self.test_index < 0):
			print "  Warning: No tests to run."
			self.change_state_to(STATE_TERMINATE_PROGRAM)					
		# We have finished the test suite
		else:
			print "  Published all test messages"
			self.change_state_to(STATE_TERMINATE_PROGRAM)		

	def process_state_update_robot_pose(self):
		print('  Waiting for the robot pose to be updated...')
		if (self.updated_robot_pose):
			self.updated_robot_pose = False
			self.change_state_to(STATE_PUBLISH_MSG)

	def process_state_publish_msg(self):
		global pubWhole, pubNeck, pubSpine, pubFootSteps, pubFootLoadBearing, pubGoHome, pubStopTrajectory, pubPauseWalking
		print('  Preparing message...')
		can_publish = False

		if self.current_test.message_type in ACCEPTED_MESSAGES:
			can_publish = True
			if self.current_test.message_type == "controller_msgs/GoHomeMessage":
				pubGoHome.publish(self.current_test.message)
			elif self.current_test.message_type == "controller_msgs/NeckTrajectoryMessage":
				pubNeck.publish(self.current_test.message)
			elif self.current_test.message_type == "controller_msgs/SpineTrajectoryMessage":
				pubSpine.publish(self.current_test.message)
			elif self.current_test.message_type == "controller_msgs/FootstepDataListMessage":
				self.transformFootsteps(self.current_test.message)
				pubFootSteps.publish(self.current_test.message)
			elif self.current_test.message_type == "controller_msgs/FootLoadBearingMessage":
				pubFootLoadBearing.publish(self.current_test.message)
			elif self.current_test.message_type == "controller_msgs/WholeBodyTrajectoryMessage":
				self.prepare_wholebody_msg(self.current_test.message)
				pubWhole.publish(self.current_test.message)
			else:
				can_publish = False

		if can_publish:
			print "    Publishing message..."
			self.change_state_to(STATE_WAIT_FOR_EXECUTION)
		else:
			print "    Error! Message type", self.current_test.message_type, "is not supported"
			self.change_state_to(STATE_TERMINATE_PROGRAM)

	def process_state_wait_for_execution(self):
		print('  Waiting for execution...')
		if self.current_test.queued_message:
			print "  Waiting for", self.current_test.wait_time_after_publishing, "seconds"
		else:
			print "  Waiting for", self.current_test.wait_time_after_publishing, "seconds and for the robot to stop moving"

		self.wait(self.current_test.wait_time_after_publishing)
		# Load the next message only if the robot is standing or if the message is in the middle of a queue
		while not rospy.is_shutdown():
			if self.robot_stopped_moving or self.current_test.queued_message:
				# Ready for the next message
				if self.test_index < len(self.list_of_tests):
					self.test_index += 1 
				self.change_state_to(STATE_LOAD_MESSAGE)
				break
			time.sleep(0.01)


	def disable_interrupts(self):
		self.footstep_pause_interrupt = False

	def wait(self, time_to_wait):
		global pubStopTrajectory
		start_time = rospy.Time().now().to_sec() #self.robot_time
		current_time = rospy.Time().now().to_sec()
		interval = current_time - start_time
		while (interval) < time_to_wait and not rospy.is_shutdown():
			current_time = rospy.Time().now().to_sec()
			interval = current_time - start_time

			if self.footstep_pause_interrupt:
				self.disable_interrupts()
				break
			if (self.current_test.pause_at_time > 0.0) and (interval > self.current_test.pause_at_time):
				msg = StopAllTrajectoryMessage()
				msg.sequence_id = 10
				pubStopTrajectory.publish(msg)
				break
			time.sleep(0.01)


	# Main State Machine Loop
	def run(self):
		print STATE_INDEX_TO_NAME[self.state]
		if self.test_index < len(self.list_of_tests):
			self.output_status()

		# Begin State Machine Logic Loop
		# STATE_LOAD_MESSAGE
		if self.state == STATE_LOAD_MESSAGE:
			self.process_state_load_message()
		# STATE_UPDATE_ROBOT_POSE 
		elif self.state == STATE_UPDATE_ROBOT_POSE:
			self.process_state_update_robot_pose()
		# STATE_PUBLISH_MSG
		elif self.state == STATE_PUBLISH_MSG:
			self.process_state_publish_msg()
		# STATE_WAIT_FOR_EXECUTION 
		elif self.state == STATE_WAIT_FOR_EXECUTION:
			self.process_state_wait_for_execution()
		# STATE_TERMINATE_PROGRAM
		elif (self.state == STATE_TERMINATE_PROGRAM):
			print "    Terminating program"
			return False
		else:
			print self.state, "is an unknown state"
			return False

		return True


def run_test(data):
	global pubWhole, pubNeck, pubSpine, pubFootSteps, pubFootLoadBearing, pubGoHome, pubStopTrajectory, pubPauseWalking
	global tfListener

	tfListener = TransformListener()
	test_suite_obj = Test_Suite_State_Machine()

	# Set package path
	package_path = rospy.get_param('~TestSuitePackagePath', '')
	test_suite_obj.set_package_path(package_path)

	# Advertise Publishers
	pubWhole = rospy.Publisher('/ihmc/valkyrie/humanoid_control/input/whole_body_trajectory', WholeBodyTrajectoryMessage, queue_size=10, latch=True)
	pubNeck = rospy.Publisher('/ihmc/valkyrie/humanoid_control/input/neck_trajectory', NeckTrajectoryMessage, queue_size=10, latch=True)
    #pubSpine = rospy.Publisher('/ihmc/valkyrie/humanoid_control/input/spine_trajectory', SpineTrajectoryMessage, queue_size=10, latch=True)
	pubFootSteps = rospy.Publisher('/ihmc/valkyrie/humanoid_control/input/footstep_data_list', FootstepDataListMessage, queue_size=10, latch=True)
	pubFootLoadBearing = rospy.Publisher('/ihmc/valkyrie/humanoid_control/input/foot_load_bearing', FootLoadBearingMessage, queue_size=10, latch=True)
	pubGoHome = rospy.Publisher('/ihmc/valkyrie/humanoid_control/input/go_home', GoHomeMessage, queue_size=10, latch=True)
	pubStopTrajectory = rospy.Publisher('/ihmc/valkyrie/humanoid_control/input/stop_all_trajectory', StopAllTrajectoryMessage, queue_size=10, latch=True)
	pubPauseWalking = rospy.Publisher('/ihmc/valkyrie/humanoid_control/input/pause_walking', PauseWalkingMessage, queue_size=10, latch=True)

	# Define Subscribers
	rospy.Subscriber("/ihmc_ros/valkyrie/output/robot_pose", Odometry, test_suite_obj.robot_pose_callback)
	rospy.Subscriber("/ihmc_ros/valkyrie/output/robot_motion_status", String, test_suite_obj.robot_motion_status_callback)
	rospy.Subscriber("/ihmc/valkyrie/humanoid_control/output/footstep_status", FootstepStatusMessage, test_suite_obj.footstep_status_callback)

	# Load data and run tests
	if test_suite_obj.load_data(data):
	    # Run the Loop
	    while not rospy.is_shutdown() and test_suite_obj.run():
	   		time.sleep(0.5)

	else:
	    print ("Error occured in reading the input data file")

	print('Done')	


if __name__ == '__main__':
    rospy.init_node('ValkyrieTestSuite')
    if not rospy.has_param('~TestSuiteDataFile'):
        print('Please specify the test suite data file (YAML)!')
    else:	
        data = load(open(rospy.get_param('~TestSuiteDataFile', '')))
        run_test(data)




