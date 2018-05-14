# Valkyrie Hardware Testing Scripts
This package contains scripts for testing Valkyrie hardware behaviour.

These tests use the IHMC ROS API as it is used at the University Of Edinburgh. Failure to complete all of these test will result in loss of functionality of the robot. These tests should be run regulalry and especially before/after upgrades and maintenance visits.
All scripts are tested in the IHMC simulator (SCS).

This test suit has been designed and tested with the following versions of the software components:
- NASA API - Fievel
- IHMC Robotics Toolbox - 0.10.0
- OpenHumanoids - private repo 4ac67a0b

## Walking tests
The walking tests are manual. Each creates a foostep plan which will be executed on the robot. The operator has to visually confirm that the execution was correct. The behaviour to look out for is:
- Drift (how far off the target did the robot walk)
- Pelvis stability and oscillations (especially in single foot support)
- Foot stomping
- Falls

Each test will subscribe to the robot pose and offset the stored trajectory relative to the current pose (in case of drift). The test will only start if the robot is the `STANDING` state.
Each of the tests will also start a TF listener. Before running any of the tests, run the state publisher using:

```roslaunch valkyrie_testing_edi robot_state.launch```

Tests:
- Walking forward
  ```roslaunch valkyrie_testing_edi walk_forward.launch```
  The robot walks 4 short steps forwards relative where the robot is standing.
- Walking backward
  ```roslaunch valkyrie_testing_edi walk_backward.launch```
  The robot walks 4 short steps backwards relative where the robot is standing.
- Turn clockwise
  ```roslaunch valkyrie_testing_edi walk_turn_cw.launch```
  The robot turns clockwise 90 degrees around the position where it's standing.
- Turn counter-clockwise
  ```roslaunch valkyrie_testing_edi walk_turn_ccw.launch```
  The robot turns counter-clockwise 90 degrees around the position where it's standing.
- Pause a walking walking trajectory
  ```roslaunch valkyrie_testing_edi walk_forward_pause.launch```
  The robot is commanded to walk 4 short steps forward but the execution is pased after the first step. The second step should finish and the robot should stop executing only two steps.
- Nominal pose
  ```roslaunch valkyrie_testing_edi whole_body_nominal.launch```
  The robot will move into the prep pose (over 5s) and then move into the nominal pose (slight squat, arms on the side).
- Raise the left/right arm forwards
  ```roslaunch valkyrie_testing_edi whole_body_left_arm_front.launch```
  ```roslaunch valkyrie_testing_edi whole_body_right_arm_front.launch```
  The robot will move into the prep pose (over 5s) and then raise its left/right arm forward.
- Raise the left/right arm sideways
  ```roslaunch valkyrie_testing_edi whole_body_left_arm_side.launch```
  ```roslaunch valkyrie_testing_edi whole_body_right_arm_side.launch```
  The robot will move into the prep pose (over 5s) and then raise its left/right arm sideways.
- Raise the left/right arm across and bend over
  ```roslaunch valkyrie_testing_edi whole_body_left_arm_right_bend_torso.launch```
  ```roslaunch valkyrie_testing_edi whole_body_right_arm_left_bend_torso.launch```
  The robot will move into the prep pose (over 5s) and then move its left/right arm across towards the opposite side while banding the back.
- Raise the left/right arm forward and bend over
  ```roslaunch valkyrie_testing_edi whole_body_left_arm_front_yaw_torso.launch```
  ```roslaunch valkyrie_testing_edi whole_body_right_arm_front_yaw_torso.launch```
  The robot will move into the prep pose (over 5s) and then reach with its left/right arm forward while banding the back.
- Raise the left/right arm forward and squat
  ```roslaunch valkyrie_testing_edi whole_body_left_arm_low.launch```
  ```roslaunch valkyrie_testing_edi whole_body_right_arm_low.launch```
  The robot will move into the prep pose (over 5s) and then reach with its left/right arm forward while squatting.
- Move left/right wrist
  ```roslaunch valkyrie_testing_edi whole_body_left_wrist.launch```
  ```roslaunch valkyrie_testing_edi whole_body_right_wrist.launch```
  The robot will move into the prep pose (over 5s) and then left/right wrist along both axis close to their limits.
- T pose with both arms
  ```roslaunch valkyrie_testing_edi whole_body_T_pose.launch```
  The robot will move into the prep pose (over 5s) and then move into a T pose (bose arms raised on the sides).
- Start moving right arm sideways but stop the motion after 2s
  ```roslaunch valkyrie_testing_edi whole_body_right_arm_side_pause.launch```
  The robot will move into the prep pose (over 5s) and then raise its right arm sideways but stop the motion after 2s.
- Move the chest using the spine message (joint angles)
  ```roslaunch valkyrie_testing_edi whole_body_spine.launch```
  The robot will move into the prep pose (over 5s) and then move all three back joints using the spine message.
- Move the neck
  ```roslaunch valkyrie_testing_edi neck.launch```
  Turn the head left, forward and up (moving all three neck joints) then move the head back to zero configuration.
- Lift left/right foot
  ```roslaunch valkyrie_testing_edi whole_body_left_foot_lift.launch```
  ```roslaunch valkyrie_testing_edi whole_body_right_foot_lift.launch```
  Lift the left/right foot and then put it back on the ground.