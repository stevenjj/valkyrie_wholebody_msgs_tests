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

# Define constants
ACCEPTED_TYPES = ["wholebody_msg", "neck_msg", "footstep_datalist_msg", "foot_load_bearing_msg", "go_home_msg"]
TEST_STATES = ["read_yaml_file", "update_robot_pose", "update_motion_status", "publish_msg", "wait_for_execution"]

QUEABLE_STARTING_MESSAGE_ID = 100 # message id which indicates this is the start of a queued message 
QUEABLE_ENDING_MESSAGE_ID = 200 # message id which indicates this is the last message part of the queue

class Test_Suite_State_Machine:
	def __init__(self):
		self.robot_status = "Unknown Status"
		self.robot_pose = pm.Frame(pm.Rotation.Quaternion(0, 0, 0, 1), pm.Vector(0.0, 0.0, 0.0))
		self.list_of_tests = []

		self.state = "read_yaml_file" # Initialize state to read_yaml_file
		self.test_index = 0 # Starts at 0

		self.current_data = None
		self.current_message = None

		self.robot_time = None

	def load_test_list(self, filename):
		return

	def read_yaml_file(self):
		return

	def robot_pose_callback(self):
		# update robot time
		# if update_robot_pose: 
			# update robot pose
			# updated_robot_pose = True
		return

	def robot_motion_status_callback(self):
		# Check motion status
		# if pause is requested, do an interrupt and publish stop message
		# wait()
		# change state to read yaml

		return		
	def footstep_status_callback(self):
		# Check footstep status
		# If pause is requested. Do an interrupt and publish stop message
		# wait()
		# change state to read_yaml

		return	

	# Data is written w.r.t Midfoot frame. 
	# Express position and orientation w.r.t World Frame 
	def transformSO3(self):
		return
	def transformSE3(self):
		return
	def transformFootsteps(self):
		return

	def output_status(self):
		print "Test Suite state:", self.state
		print "  test_index = ", self.test_index

	def run(self):
		# Begin State Machine Logic Loop
		'''
			if state == read_yaml:
				if test_index < total_tests:
					read_yaml
				else:
					state = Done

				if ((message_id > QUEABLE_STARTING_MESSAGE_ID) and (message_id < QUEABLE_ENDING_MESSAGE_ID)):
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

		return self.state



if __name__ == '__main__':
	rospy.init_node('ValkyrieTestSuite')

	test_suite_obj = Test_Suite_State_Machine()

	# Advertise Publishers
	pubWhole = rospy.Publisher('/ihmc/valkyrie/humanoid_control/input/whole_body_trajectory', WholeBodyTrajectoryMessage, queue_size=10, latch=True)
	pubNeck = rospy.Publisher('/ihmc/valkyrie/humanoid_control/input/neck_trajectory', NeckTrajectoryMessage, queue_size=10, latch=True)
	pubFootSteps = rospy.Publisher('/ihmc/valkyrie/humanoid_control/input/footstep_data_list', FootstepDataListMessage, queue_size=10, latch=True)
	pubFootLoadBearing = rospy.Publisher('/ihmc/valkyrie/humanoid_control/input/foot_load_bearing', FootLoadBearingMessage, queue_size=10, latch=True)

	pubStopTrajectory = rospy.Publisher('/ihmc/valkyrie/humanoid_control/input/stop_all_trajectory', StopAllTrajectoryMessage, queue_size=10, latch=True)
	pubPauseWalking = rospy.Publisher('/ihmc/valkyrie/humanoid_control/input/pause_walking', PauseWalkingMessage, queue_size=10, latch=True)

	# # Define Subscribers
	rospy.Subscriber("/ihmc_ros/valkyrie/output/robot_pose", Odometry, test_suite_obj.robot_pose_callback)
	rospy.Subscriber("/ihmc_ros/valkyrie/output/robot_motion_status", String, test_suite_obj.robot_motion_status_callback)
	rospy.Subscriber("/ihmc/valkyrie/humanoid_control/output/footstep_status", FootstepStatusMessage, test_suite_obj.footstep_status_callback)

	# # List of YAML files
	# # Read YAML file

	while not rospy.is_shutdown():
		print "state machine loop"
		test_suite_obj.run()
		time.sleep(0.5)
	print('Done')