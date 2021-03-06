Notes regarding the conversion from old IHMC messages to new IHMC ROS1 to ROS2 Messages:

1. unique_id is now sequence_id. It's best if these are set to non-zero values

2. The world frame ID is now 83766130

3. In general, all the required default values must be set explicitly when sending ROS1 messages to the to ROS1-to-ROS2 bridge. For instance:

	3a. All trajectories need to have 
	controller_msgs/QueueableMessage queueing_properties with

	queueing_properties.message_id != 0 # The neck trajectory doesn't accept zero message_id's. Setting this equal to a nonzero sequence id should be fine.
	queueing_properties.execution_mode = 0 # This must be set explicitly to a particular execution_mode

	3b. Selection and Weight matrices for SO3 and SE3 messages must be defined to the default values explicitly.
	# These matrices must be set to their default values explicitly

	controller_msgs/SelectionMatrix3DMessage angular_selection_matrix
	controller_msgs/SelectionMatrix3DMessage linear_selection_matrix
	controller_msgs/WeightMatrix3DMessage angular_weight_matrix
	controller_msgs/WeightMatrix3DMessage linear_weight_matrix

5. Currently, the API can only take 1 trajectory waypoint for all types of trajectories.

6. Each trajectory type is either a jointspace_trajectory, se3_trajectory, or an so3_trajectory, so the message fields for the joint_trajectory_messages and taskspace_trajectory_points are under this new message field.

7. Default value for weight message field is now -1.0 not .Nan

8. Topic names for publishing have been changed. Use:

/ihmc/valkyrie/humanoid_control/input/footstep_data_list
/ihmc/valkyrie/humanoid_control/input/neck_trajectory
/ihmc/valkyrie/humanoid_control/input/whole_body_trajectory
/ihmc/valkyrie/humanoid_control/input/stop_all_trajectory

It's best to check available controller options with `ros2 topic list`

9. The ROS1 publisher needs to having latching enabled. In python this is accomplished by setting the argument, latch=True at the publisher's initialization

10. spine trajectory is not in the new whole body controller messages

The checklist below was used to convert the yaml files to the correct message format:

# --------------------------------------------------------------
YAML file footstep message list conversion checklist:
	- change unique_id to sequence_id: 1 (ideally nonzero numbers)
	- for each footstep message, add default value for 
		touchdown_duration: -1
	- comment out execution_mode: 0 and previous_message_id: 0
	- rename predicted_contact_points to predicted_contact_points_2d
	- rename position_waypoints to custom_position_waypoints
	- add new default message fields:

trust_height_of_footsteps: False
are_footsteps_adjustable: False
offset_footsteps_with_execution_error: False
queueing_properties:
  sequence_id: 1
  execution_mode: 0
  message_id: 1
  previous_message_id: 0
  execution_delay_time: 0	


YAML file whole body message conversion checklist:
- Change unique_id: to sequence_id: 1 (nonzero numbers)
- Comment out execution_mode: 0 and previous_message_id: 0
- Change weight: .NaN to weight: -1.0
- Change translation: to position:
- Change rotation: to orientation:
- Delete spine_trajectory_message as it is no longer in the whole body message

For joints:
- Add jointspace_trajectory and indent joint_trajectory_messages
   - add sequence_id
- Change trajectory_points so it only contains 1 trajectory point 

For Chest:
- Add so3_trajectory and indent the lines right before the old execution_mode text (eg: taskspace_trajectory_points, frame_information, etc.)
  - add sequence_id
- (for active messages) Add default values of selection and weight matrices
- Remove taskspace_trajectory_points so it only contains 1 waypoint
- Change frame id -107 to new world frame id: 83766130
- add message fields under so3_trajectory: 
    selection_matrix:
      sequence_id: 1
      x_selected: true 
      y_selected: true 
      z_selected: true 
    weight_matrix:
      sequence_id: 1
      weight_frame_id: 0
      x_weight: -1
      y_weight: -1
      z_weight: -1    


For pelvis, left foot and right foot trajectory messages:

- Add se3_trajectory and indent lines right before the old execution_mode text
    - add sequence_id
- (for active messages) Add default values of selection and weight matrices
- Remove taskspace_trajectory_points so it only contains 1 waypoint
- Change frame id -107 to new world frame id: 83766130

- add message fields under se3_trajectory
    angular_selection_matrix:
      sequence_id: 1
      x_selected: true
      y_selected: true
      z_selected: true
    linear_selection_matrix:
      sequence_id: 1
      x_selected: true
      y_selected: true
      z_selected: true
    angular_weight_matrix:
      sequence_id: 1
      weight_frame_id: 0
      x_weight: -1
      y_weight: -1
      z_weight: -1
    linear_weight_matrix:
      sequence_id: 1
      weight_frame_id: 0
      x_weight: -1
      y_weight: -1
      z_weight: -1  



#------------------------------------------------------------------------------
Status of ROS1 -> ROS2 bridge message testing

Converted files and their status:
	- neck.yaml 
		- Status: Good
	- walk_forward.yaml 
		- Status: Good
	- whole_body_left_arm_front.yaml 
		- Status: Good. But only 1 waypoint per message
	- whole_body_right_arm_front.yaml 
		- Status: Good. Only 1 waypoint per message
	- whole_body_nominal.yaml 
		- Status: Good. Pelvis, torso and arms go back to the "home" position. Feet do not move. Only 1 waypoint per message
	- whole_body_left_foot_lift 
		- Status: Acceptable. We increased the space between the foot. Contains only 1 waypoint message. The left foot is raised, and she switches the CoP to the right foot. But, the robot_motion_status remains in the "IN_MOTION" state. She also does not put her foot back down. So no further messages can be sent, unless we renable the left foot contact with the FootLoadBearing Message.  )
	- whole_body_right_foot_lift.yaml (we increased the distance between the feet)
		- Status: Acceptable. Same as left foot lift
	- whole_body_left_arm_front_yaw_torso.yaml
		- Status: Good. 
	- whole_body_left_arm_front_yaw_torso.yaml
		- Status: Good
	- whole_body_right_arm_front_yaw_torso.yaml
		- Status: Good
	- whole_body_left_arm_low.yaml
		- Status: Good	
	- whole_body_right_arm_low.yaml
		- Status: Good
	- whole_body_left_arm_right_bend_torso.yaml
		- Status: Good
	- whole_body_right_arm_left_bend_torso.yaml
		- Status: Good
	- whole_body_left_arm_side.yaml
		- Status: Good
	- whole_body_right_arm_side.yaml
		- Status: Good
	- whole_body_T_pose.yaml
		- Status: Good
	- walk_forward_pause.launch
		- Status: Good
	- walk_in_place.yaml
		- Status: Good. New script which squares the feet to the mid frame orientation spaced at 0.125m.
	- walk_backward.yaml
		- Status: Good
	- load_left_foot.yaml
		- Status: Good
	- load_right_foot.yaml	
		- Status: Good
	- walk_turn_cw.yaml
		- Status: Good. We enforce a normalization step on the orientation after converting the frame into the world frame.
	- walk_turn_ccw.yaml
		- Status: Good. We enforce a normalization step on the orientation after converting the frame into the world frame.
	- whole_body_left_wrist 
		- Status: Good
	- whole_body_right_wrist 
		- Status: Good
	- whole_body_spine 	
		- Status: Untested for now, but no reason to not work.
	- whole_body_right_arm_side_pause
		Status: We cannot test this because the API only allows 1 trajectory message

Converted Python scripts
	neck.py 
		- Status: Good
	footstep.py 
		- Status: Good. 
	whole_body.py 
		- Status: Good. We have not confirmed StopAllTrajectories message due to current API limits.
	converted_load_foot.py
		- Status: Good. Loads the foot if it is unloaded.
	go_home.py
		- Status: Good. Sends a single go home message
	converted_spine.py
		- Status: Untested for now.


Tested ROS1 Messages
	WholeBodyTrajectoryMessage
		ArmTrajectoryMessage 
			- Status: Good. Left and right arm trajectories work reliably
		FootTrajectoryMessage  
			- Status: Good. Left foot works. Presumably Right foot also works
		PelvisTrajectoryMessage 
			- Status: Good. We only tried changing positions, but orientation seems to be correct. selection and weight matrices must be specified
		ChestTrajectoryMessage 
			- Status: Good. selection and weight matrices must be specified
		HeadTrajectoryMessage 
			- Status: Untested. All Fields are Empty
		HandTrajectoryMessage 
			- Status: Untested.	

	PauseWalkingMessage 
		- Status: Good. We tested with a sequence_id greater than the foot message
	NeckTrajectoryMessage 
		- Status: Good. message_id must not be 0
	PelvisHeightTrajectoryMessage 
		- Status: Good.
	FootLoadBearingMessage 
		- Status: Good. Works as intended. Left Foot can be loaded again once it is lifted up. But, it cannot be unloaded while being loaded and in contact with the ground


TO DO:
- DONE Retest right arm up trajectory message
- DONE Test footsteps with pause message 
- DONE Test left arm up trajectory
- DONE Test lift foot up 
	- Robot enters IN_MOTION state. She does not exit from this state and go back to STANDING 
- DONE Test whole body nominal 
- DONE Write foot load bearing script
-	DONE Finish conversion of wrist
- DONE	Write Go Home script. Try using the queue.
- DONE	Write python script which sends the entire test suite

- DONE Normalize quaternion before sending out

- DONE Test unit quaternion normalization for the footsteps. 
	DONE - Use the turn cw script.
	DONE - Uncomment lines after 2 steps are successful
- DONE Test Go Home suite
- DONE Test Neck suite
- DONE Test Turn cw suite
- DONE Test right arm front then go home suite
- DONE Test right foot lift then down suite
- DONE Test full suite
- DONE Write spine joint trajectory script to convert spine message
- Modify T-pose angles to reduce singularity shaking
- Test spine script
- Test right arm side pause script.
- Include spine script in full test suite
	- Partially done. need to test individual spine test first
