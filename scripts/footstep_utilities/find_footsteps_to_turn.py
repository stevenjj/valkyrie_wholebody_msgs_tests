import math
import numpy as np

# Mid Feet Frame: positive x-coordinate is up, positive y-coordinate is to the left.
# world Frame: positive x-coordinate is to the right, positive y-coordinate is up.
# Define rhe Rotation matrix which expresses a world-frame in mid-feet frame:
R_feet = np.array([[0, 1], \
		    	   [-1, 0]]) 


# Returns the footstep location in mid-feet frame
# Inputs:
#	r - distance from the midfeet location
#	theta - angle of a point in the unit circle following the right hand rule.
#			a theta of 0.0 has feet aligned with the y.   
def footstep_out(r, theta):
	xy_position = R_feet.dot(np.array([r*math.cos(theta), r*math.sin(theta)]))
	(qx, qy, qz, qw) = (0.0, 0.0, math.sin(theta/2.0), math.cos(theta/2.0))
	orientation = [qx, qy, qz, qw] 
	return (np.round(xy_position, decimals=4), orientation)


# Provides a list of angles for the orientation of each foot to reach a desired final angle
def angles_out(des_final_angle, max_angle_per_step, rads_tolerance = np.pi/180.0):
	# Compute number of full steps the robot can do
	factor = des_final_angle/max_angle_per_step
	num_full_steps = int(np.abs(factor))
	# Do full steps 
	angles = [(i+1)*max_angle_per_step*np.sign(des_final_angle) for i in range(num_full_steps)]

	# Check if we are off by more than the rads_tolerance
	final_step_delta_angle = np.fabs(des_final_angle) - (max_angle_per_step*num_full_steps)
	if final_step_delta_angle > rads_tolerance:
		# Do a final step if we are
		final_step_angle = des_final_angle 
		angles.append(final_step_angle)

	print "Final heading error is ", (np.fabs(des_final_angle) - np.fabs(angles[-1]))*180/np.pi, " degrees"

	return angles


# Gets a list of footstep poses for each foot to reach the desired heading
def get_pose_list(r, angles):
	left_foot_poses = []
	right_foot_poses = []

	for i in range(len(angles)):
		left_foot_poses.append(footstep_out(-r, angles[i]))
		right_foot_poses.append(footstep_out(r, angles[i]))

	return (left_foot_poses, right_foot_poses)

if __name__ == '__main__':
	# space between midfeet and foot
	r = np.fabs(0.175)
	# the maximum turning angle per step
	max_angle_per_step = np.fabs(np.pi/6.0) # must be < PI/6.0 to comply with IHMC controller limits
	# the desired final heading angle:
	des_final_angle = np.pi/2.0
	# Final heading tolerance of 1 degree
	rads_tolerance = np.pi/180.0

	print "Absolute foot distance from midfeet = ", r, " meters"
	print "Desired Final Heading Angle = ", des_final_angle, " radians"	
	print "Maximum Angle per step = ", max_angle_per_step, " radians"
	print "Final heading tolerance = ", rads_tolerance*180.0/np.pi, " degrees"	
	print " "
	angles =  angles_out(des_final_angle, max_angle_per_step, rads_tolerance)
	print "Angle waypoint list for each footstep: "
	print angles
	print " "
	# Get a list of footstep poses
	(left_foot_poses, right_foot_poses) = get_pose_list(r, angles)


	# Print in a human readable way
	for i in range(len(angles)):
		print "Footstep set ", i
		(xy_position, orientation) = right_foot_poses[i]
		print "Right Foot: xy_position: ", xy_position, "quat_ori:", orientation
		(xy_position, orientation) = left_foot_poses[i]		
		print "Left Foot: xy_position: ", xy_position, "quat_ori:", orientation





