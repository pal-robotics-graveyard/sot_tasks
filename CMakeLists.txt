cmake_minimum_required(VERSION 2.8.3)
project(sot_tasks)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic-graph
  dynamic-graph-python
  jrl-mal
  jrl-mathtools
  roscpp
  sot-core
  sot-dyninv
  sot_cmake
  tf2
)

## Uncomment this if the package has a setup.py. This macro ensures
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
#
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
# modules and global scripts declared therein get installed
catkin_python_setup()




catkin_package(
  INCLUDE_DIRS include
  LIBRARIES task_velocity_damping task_joint_limit_clamping task_joint_weights
  CATKIN_DEPENDS dynamic-graph dynamic-graph-python jrl-mal jrl-mathtools roscpp sot-core sot-dyninv sot_cmake tf2
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories( include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(task_velocity_damping
   src/TaskVelocityDamping.cpp
   include/sot_tasks/TaskVelocityDamping.h
   include/sot_tasks/SignalHelper.h
)
target_link_libraries(task_velocity_damping ${CATKIN_LIBRARIES})

## Declare a cpp library
add_library(task_joint_limit_clamping
   src/TaskJointLimitClamping.cpp
   include/sot_tasks/TaskJointLimitClamping.h
)
target_link_libraries(task_joint_limit_clamping ${CATKIN_LIBRARIES})

## Declare a cpp library
add_library(task_joint_weights
   src/TaskJointWeights.cpp
   include/sot_tasks/TaskJointWeights.h
)
target_link_libraries(task_joint_weights ${CATKIN_LIBRARIES})

## Declare a cpp library
add_library(feature_grasping_point
   src/FeatureGraspingPoint.cpp
   include/sot_tasks/FeatureGraspingPoint.h
)
target_link_libraries(feature_grasping_point ${CATKIN_LIBRARIES})
add_subdirectory(src)
############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS feature_grasping_point task_velocity_damping task_joint_weights task_joint_limit_clamping
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

