cmake_minimum_required(VERSION 2.8.3)
project(valkyrie_testing_edi)


find_package(catkin REQUIRED )

catkin_package(
  CATKIN_DEPENDS geometry_msgs ihmc_msgs nav_msgs rospy_message_converter std_msgs tf_conversions yaml
)

#############
## Install ##
#############

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY data/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data)
FILE(GLOB python_scripts "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py")
catkin_install_python(PROGRAMS scripts/footsteps.py scripts/whole_body.py scripts/neck.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

