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
from geometry_msgs.msg import Point, Quaternion

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
ACCEPTED_TYPES = ["wholebody", "neck", "walk", "load", "go_home", "wholebody_with_pause", "walk_with_pause"]
ACCEPTED_MESSAGES = ["controller_msgs/FootstepDataListMessage", 
					 "controller_msgs/GoHomeMessage"]

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

# Do not change these default values! If you do, be sure to change the queueable messages as well
QUEABLE_STARTING_MESSAGE_ID = 100 # message id which indicates this is the start of a queued message 
QUEABLE_ENDING_MESSAGE_ID = 200 # message id which indicates this is the last message part of the queue
# A message_id with QUEABLE_STARTING_MESSAGE_ID < message_id < QUEABLE_ENDING_MESSAGE_ID will be processed as fast as possible
# to send 

class TestParams:
	def __init__(self):
		self.filepath = ""
		self.wait_time_after_publishing = 0.0
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
		self.robot_status = "Unknown Status"
		self.robot_pose = pm.Frame(pm.Rotation.Quaternion(0, 0, 0, 1), pm.Vector(0.0, 0.0, 0.0))
		self.list_of_tests = []
		self.current_test = None

		self.state = STATE_LOAD_MESSAGE # Initialize state to read_yaml_file
		self.test_index = -1 # Starts at invalid index

		self.robot_time = None
		self.update_robot_pose = False

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
			test_params.wait_time_after_publishing =  test_info['wait_time_after_publishing']

			if 'pause_at_step' in test_info:
				test_params.pause_at_step =  test_info['pause_at_step']
			if 'pause_at_time' in test_info:
				test_params.pause_at_time =  test_info['pause_at_time']
			if 'queued_message' in test_info:
				test_params.queued_message =  test_info['queued_message']

			try:
				if test_params.load_message():
					#self.list_of_tests.append(test_params)
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

	def read_yaml_file(self):
		return

	def robot_pose_callback(self, msg):
		# Update robot time
		self.robot_time = msg.header.stamp.to_sec()		

		# if update_robot_pose: 
			# update robot pose
			# updated_robot_pose = True
		
		return

	def robot_motion_status_callback(self, msg):
		# Check motion status
		# if pause is requested, do an interrupt and publish stop message
		# wait()
		# change state to read yaml

		return		
	def footstep_status_callback(self, msg):
		# Check footstep status
		# If pause is requested. Do an interrupt and publish stop message
		# wait()
		# change state to read_yaml

		return	

	# Data is written w.r.t Midfoot frame. These transformations will express position and orientation w.r.t World Frame 
	def transformSO3(self):
		return
	def transformSE3(self):
		return
	def transformFootsteps(self):
		return
	# -----------------------------------------------------------------------------------------------
	def output_status(self):
		print "  Running Test", (self.test_index + 1) , "out of", len(self.list_of_tests), "tests"

	def change_state_to(self, new_state):
		self.state = new_state
		print "  Changing state to ", STATE_INDEX_TO_NAME[self.state]

	def run(self):
		print "Test Suite state:", STATE_INDEX_TO_NAME[self.state]
		self.output_status()

		# Begin State Machine Logic Loop
		if self.state == STATE_LOAD_MESSAGE:
			# Check if we have any tests to run
			if (self.test_index < 0):
				print "  Warning: No tests to run."
				self.change_state_to(STATE_TERMINATE_PROGRAM)
			else:
				# We have Programs to run
				if self.test_index < len(self.list_of_tests):
					print('  Loading the message...')		
				else:
					self.change_state_to(STATE_TERMINATE_PROGRAM)


		elif (self.state == STATE_TERMINATE_PROGRAM):
			print "    Terminating program"
			return False

		# Pseudo Code:
		'''
			if state == read_yaml:
				if test_index < total_tests:
					read_yaml
				else:
					state = Done

				if (message_id > QUEABLE_STARTING_MESSAGE_ID) and (message_id < QUEABLE_ENDING_MESSAGE_ID)):
  				 	perform transforms using previous pose, publish, wait for 1 second then read the next file.
					change state to read_yaml.
					Read the file. and immediately publish the message
					state = publish_msg

				else:
					This is a regular message
					Read Message.
					Change State to update_robot_pose

			else if state == update_robot_pose:
				# wait for robot pose to be updated by a callback
				if updated_robot_pose:
					updated_robot_pose = False
					state = publish_msg

			else if state == publish_msg:
				# Transform message
				# Extract wait command
				if robot_stopped_moving or message_is_queuable:
				
				# Publish Message

				state = wait_for_execution

			else if state == wait_for_execution:
				# Wait for execution unless the message is queueable
				if not(message_is_queueable):
					wait()
				else:
					wait(1.0) # Wait to ensure that ros2 receives the message

				# If the message is queuable read the next yaml file
				if message_is_queueable or robot_stopped_moving:
					message_is_queuable = False
					state = read_yaml

			else if state = DONE:
				return state

		'''

		return True


def run_test(data):
    test_suite_obj = Test_Suite_State_Machine()
    # Set package path
    package_path = rospy.get_param('~TestSuitePackagePath', '')
    test_suite_obj.set_package_path(package_path)

    # Load data and run tests
    if test_suite_obj.load_data(data):

        # Advertise Publishers
        pubWhole = rospy.Publisher('/ihmc/valkyrie/humanoid_control/input/whole_body_trajectory', WholeBodyTrajectoryMessage, queue_size=10, latch=True)
        pubNeck = rospy.Publisher('/ihmc/valkyrie/humanoid_control/input/neck_trajectory', NeckTrajectoryMessage, queue_size=10, latch=True)
        pubFootSteps = rospy.Publisher('/ihmc/valkyrie/humanoid_control/input/footstep_data_list', FootstepDataListMessage, queue_size=10, latch=True)
        pubFootLoadBearing = rospy.Publisher('/ihmc/valkyrie/humanoid_control/input/foot_load_bearing', FootLoadBearingMessage, queue_size=10, latch=True)
        pubGoHome = rospy.Publisher('/ihmc/valkyrie/humanoid_control/input/go_home', GoHomeMessage, queue_size=10, latch=True)

        pubStopTrajectory = rospy.Publisher('/ihmc/valkyrie/humanoid_control/input/stop_all_trajectory', StopAllTrajectoryMessage, queue_size=10, latch=True)
        pubPauseWalking = rospy.Publisher('/ihmc/valkyrie/humanoid_control/input/pause_walking', PauseWalkingMessage, queue_size=10, latch=True)

        # Define Subscribers
        rospy.Subscriber("/ihmc_ros/valkyrie/output/robot_pose", Odometry, test_suite_obj.robot_pose_callback)
        rospy.Subscriber("/ihmc_ros/valkyrie/output/robot_motion_status", String, test_suite_obj.robot_motion_status_callback)
        rospy.Subscriber("/ihmc/valkyrie/humanoid_control/output/footstep_status", FootstepStatusMessage, test_suite_obj.footstep_status_callback)

        # Run the Loop
        while not rospy.is_shutdown():
        	if test_suite_obj.run():
        		time.sleep(0.5)
        	else:
        		break
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




