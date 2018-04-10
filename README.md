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