#
#  Copyright (c) 2013, PAL Robotics, S.L. Author: Karsten Knese, Gennaro Raiola
#  Copyright 2010 CNRS
#

# LAAS cmake include for python 
include(${SOT_CMAKE_DIR}/python.cmake)
FINDPYTHON()
set(LIBRARY_OUTPUT_PATH ${CATKIN_DEVEL_PREFIX}/lib)
message(${LIBRARY_OUTPUT_PATH})
#set(WRAP_DIR ${LIBRARY_OUTPUT_PATH}/dynamic_graph/sot/tasks/${PROJECT_NAME})
#message("creating wrapper dir for ${PROJECT_NAME}: ${WRAP_DIR}")
message("python sitedir : ${PYTHON_SITELIB}")

dynamic_graph_python_module("${PROJECT_NAME}"
"" ${PROJECT_NAME}/wrap)

set(WRAPPERS task_velocity_damping task_joint_weights task_joint_limit_clamping feature_grasping_point)
foreach(TASK ${WRAPPERS})
#file(MAKE_DIRECTORY "${LIBRARY_OUTPUT_PATH}/dynamic_graph/sot/tasks/${PROJECT_NAME}")
dynamic_graph_python_module("${PROJECT_NAME}/${TASK}"
    ${TASK}
    ${PROJECT_NAME}/${TASK}/wrap
)
endforeach(TASK)

#PYTHON_INSTALL_ON_SITE("dynamic_graph/${PROJECT_NAME}" "meta_task_velocity_damping.py")
#PYTHON_INSTALL_ON_SITE("dynamic_graph/${PROJECT_NAME}" "meta_task_dynamic_velocity_damping.py")
#PYTHON_INSTALL_ON_SITE("dynamic_graph/${PROJECT_NAME}" "meta_task_opmod_velocity_damping.py")
