cmake_minimum_required(VERSION 2.8.3)
project(descartes_planning_server)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslib
  descartes_core
  descartes_moveit
  descartes_trajectory
  descartes_planner
  descartes_utilities
  trajectory_msgs
  tf
  tf_conversions
  eigen_conversions
  moveit_visual_tools
  nexon_msgs
  descartes_opw_model
)

find_package(opw_kinematics REQUIRED)

catkin_package(
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(${PROJECT_NAME}_server src/server.cpp)

target_link_libraries(${PROJECT_NAME}_server
 ${catkin_LIBRARIES}
)

set_target_properties(${PROJECT_NAME}_server
  PROPERTIES OUTPUT_NAME server
  PREFIX "")

#############
## Install ##
#############

# install(TARGETS ${PROJECT_NAME}_tutorial1
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
# )
