# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "hsr_planner: 4 messages, 1 services")

set(MSG_I_FLAGS "-Ihsr_planner:/home/hrl/catkin_ws/src/hsr_planner/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Inav_msgs:/opt/ros/melodic/share/nav_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(hsr_planner_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/hrl/catkin_ws/src/hsr_planner/msg/ObjectMessage.msg" NAME_WE)
add_custom_target(_hsr_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hsr_planner" "/home/hrl/catkin_ws/src/hsr_planner/msg/ObjectMessage.msg" "hsr_planner/CellMessage"
)

get_filename_component(_filename "/home/hrl/catkin_ws/src/hsr_planner/msg/ClutterPlannerServiceReq.msg" NAME_WE)
add_custom_target(_hsr_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hsr_planner" "/home/hrl/catkin_ws/src/hsr_planner/msg/ClutterPlannerServiceReq.msg" "nav_msgs/MapMetaData:hsr_planner/CellMessage:geometry_msgs/Pose:nav_msgs/OccupancyGrid:std_msgs/Header:hsr_planner/ObjectMessage:geometry_msgs/Point:geometry_msgs/PoseStamped:geometry_msgs/Quaternion"
)

get_filename_component(_filename "/home/hrl/catkin_ws/src/hsr_planner/msg/ClutterPlannerServiceResp.msg" NAME_WE)
add_custom_target(_hsr_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hsr_planner" "/home/hrl/catkin_ws/src/hsr_planner/msg/ClutterPlannerServiceResp.msg" "hsr_planner/CellMessage:geometry_msgs/Pose:std_msgs/Header:hsr_planner/ObjectMessage:geometry_msgs/Point:geometry_msgs/PoseStamped:geometry_msgs/Quaternion"
)

get_filename_component(_filename "/home/hrl/catkin_ws/src/hsr_planner/srv/ClutterPlannerService.srv" NAME_WE)
add_custom_target(_hsr_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hsr_planner" "/home/hrl/catkin_ws/src/hsr_planner/srv/ClutterPlannerService.srv" "nav_msgs/MapMetaData:hsr_planner/CellMessage:geometry_msgs/Pose:nav_msgs/OccupancyGrid:std_msgs/Header:hsr_planner/ObjectMessage:geometry_msgs/Point:geometry_msgs/PoseStamped:geometry_msgs/Quaternion"
)

get_filename_component(_filename "/home/hrl/catkin_ws/src/hsr_planner/msg/CellMessage.msg" NAME_WE)
add_custom_target(_hsr_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hsr_planner" "/home/hrl/catkin_ws/src/hsr_planner/msg/CellMessage.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(hsr_planner
  "/home/hrl/catkin_ws/src/hsr_planner/msg/ObjectMessage.msg"
  "${MSG_I_FLAGS}"
  "/home/hrl/catkin_ws/src/hsr_planner/msg/CellMessage.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hsr_planner
)
_generate_msg_cpp(hsr_planner
  "/home/hrl/catkin_ws/src/hsr_planner/msg/ClutterPlannerServiceReq.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/MapMetaData.msg;/home/hrl/catkin_ws/src/hsr_planner/msg/CellMessage.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/OccupancyGrid.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/hrl/catkin_ws/src/hsr_planner/msg/ObjectMessage.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hsr_planner
)
_generate_msg_cpp(hsr_planner
  "/home/hrl/catkin_ws/src/hsr_planner/msg/ClutterPlannerServiceResp.msg"
  "${MSG_I_FLAGS}"
  "/home/hrl/catkin_ws/src/hsr_planner/msg/CellMessage.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/hrl/catkin_ws/src/hsr_planner/msg/ObjectMessage.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hsr_planner
)
_generate_msg_cpp(hsr_planner
  "/home/hrl/catkin_ws/src/hsr_planner/msg/CellMessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hsr_planner
)

### Generating Services
_generate_srv_cpp(hsr_planner
  "/home/hrl/catkin_ws/src/hsr_planner/srv/ClutterPlannerService.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/MapMetaData.msg;/home/hrl/catkin_ws/src/hsr_planner/msg/CellMessage.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/OccupancyGrid.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/hrl/catkin_ws/src/hsr_planner/msg/ObjectMessage.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hsr_planner
)

### Generating Module File
_generate_module_cpp(hsr_planner
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hsr_planner
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(hsr_planner_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(hsr_planner_generate_messages hsr_planner_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hrl/catkin_ws/src/hsr_planner/msg/ObjectMessage.msg" NAME_WE)
add_dependencies(hsr_planner_generate_messages_cpp _hsr_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hrl/catkin_ws/src/hsr_planner/msg/ClutterPlannerServiceReq.msg" NAME_WE)
add_dependencies(hsr_planner_generate_messages_cpp _hsr_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hrl/catkin_ws/src/hsr_planner/msg/ClutterPlannerServiceResp.msg" NAME_WE)
add_dependencies(hsr_planner_generate_messages_cpp _hsr_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hrl/catkin_ws/src/hsr_planner/srv/ClutterPlannerService.srv" NAME_WE)
add_dependencies(hsr_planner_generate_messages_cpp _hsr_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hrl/catkin_ws/src/hsr_planner/msg/CellMessage.msg" NAME_WE)
add_dependencies(hsr_planner_generate_messages_cpp _hsr_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hsr_planner_gencpp)
add_dependencies(hsr_planner_gencpp hsr_planner_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hsr_planner_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(hsr_planner
  "/home/hrl/catkin_ws/src/hsr_planner/msg/ObjectMessage.msg"
  "${MSG_I_FLAGS}"
  "/home/hrl/catkin_ws/src/hsr_planner/msg/CellMessage.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hsr_planner
)
_generate_msg_eus(hsr_planner
  "/home/hrl/catkin_ws/src/hsr_planner/msg/ClutterPlannerServiceReq.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/MapMetaData.msg;/home/hrl/catkin_ws/src/hsr_planner/msg/CellMessage.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/OccupancyGrid.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/hrl/catkin_ws/src/hsr_planner/msg/ObjectMessage.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hsr_planner
)
_generate_msg_eus(hsr_planner
  "/home/hrl/catkin_ws/src/hsr_planner/msg/ClutterPlannerServiceResp.msg"
  "${MSG_I_FLAGS}"
  "/home/hrl/catkin_ws/src/hsr_planner/msg/CellMessage.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/hrl/catkin_ws/src/hsr_planner/msg/ObjectMessage.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hsr_planner
)
_generate_msg_eus(hsr_planner
  "/home/hrl/catkin_ws/src/hsr_planner/msg/CellMessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hsr_planner
)

### Generating Services
_generate_srv_eus(hsr_planner
  "/home/hrl/catkin_ws/src/hsr_planner/srv/ClutterPlannerService.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/MapMetaData.msg;/home/hrl/catkin_ws/src/hsr_planner/msg/CellMessage.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/OccupancyGrid.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/hrl/catkin_ws/src/hsr_planner/msg/ObjectMessage.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hsr_planner
)

### Generating Module File
_generate_module_eus(hsr_planner
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hsr_planner
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(hsr_planner_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(hsr_planner_generate_messages hsr_planner_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hrl/catkin_ws/src/hsr_planner/msg/ObjectMessage.msg" NAME_WE)
add_dependencies(hsr_planner_generate_messages_eus _hsr_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hrl/catkin_ws/src/hsr_planner/msg/ClutterPlannerServiceReq.msg" NAME_WE)
add_dependencies(hsr_planner_generate_messages_eus _hsr_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hrl/catkin_ws/src/hsr_planner/msg/ClutterPlannerServiceResp.msg" NAME_WE)
add_dependencies(hsr_planner_generate_messages_eus _hsr_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hrl/catkin_ws/src/hsr_planner/srv/ClutterPlannerService.srv" NAME_WE)
add_dependencies(hsr_planner_generate_messages_eus _hsr_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hrl/catkin_ws/src/hsr_planner/msg/CellMessage.msg" NAME_WE)
add_dependencies(hsr_planner_generate_messages_eus _hsr_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hsr_planner_geneus)
add_dependencies(hsr_planner_geneus hsr_planner_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hsr_planner_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(hsr_planner
  "/home/hrl/catkin_ws/src/hsr_planner/msg/ObjectMessage.msg"
  "${MSG_I_FLAGS}"
  "/home/hrl/catkin_ws/src/hsr_planner/msg/CellMessage.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hsr_planner
)
_generate_msg_lisp(hsr_planner
  "/home/hrl/catkin_ws/src/hsr_planner/msg/ClutterPlannerServiceReq.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/MapMetaData.msg;/home/hrl/catkin_ws/src/hsr_planner/msg/CellMessage.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/OccupancyGrid.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/hrl/catkin_ws/src/hsr_planner/msg/ObjectMessage.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hsr_planner
)
_generate_msg_lisp(hsr_planner
  "/home/hrl/catkin_ws/src/hsr_planner/msg/ClutterPlannerServiceResp.msg"
  "${MSG_I_FLAGS}"
  "/home/hrl/catkin_ws/src/hsr_planner/msg/CellMessage.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/hrl/catkin_ws/src/hsr_planner/msg/ObjectMessage.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hsr_planner
)
_generate_msg_lisp(hsr_planner
  "/home/hrl/catkin_ws/src/hsr_planner/msg/CellMessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hsr_planner
)

### Generating Services
_generate_srv_lisp(hsr_planner
  "/home/hrl/catkin_ws/src/hsr_planner/srv/ClutterPlannerService.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/MapMetaData.msg;/home/hrl/catkin_ws/src/hsr_planner/msg/CellMessage.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/OccupancyGrid.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/hrl/catkin_ws/src/hsr_planner/msg/ObjectMessage.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hsr_planner
)

### Generating Module File
_generate_module_lisp(hsr_planner
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hsr_planner
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(hsr_planner_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(hsr_planner_generate_messages hsr_planner_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hrl/catkin_ws/src/hsr_planner/msg/ObjectMessage.msg" NAME_WE)
add_dependencies(hsr_planner_generate_messages_lisp _hsr_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hrl/catkin_ws/src/hsr_planner/msg/ClutterPlannerServiceReq.msg" NAME_WE)
add_dependencies(hsr_planner_generate_messages_lisp _hsr_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hrl/catkin_ws/src/hsr_planner/msg/ClutterPlannerServiceResp.msg" NAME_WE)
add_dependencies(hsr_planner_generate_messages_lisp _hsr_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hrl/catkin_ws/src/hsr_planner/srv/ClutterPlannerService.srv" NAME_WE)
add_dependencies(hsr_planner_generate_messages_lisp _hsr_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hrl/catkin_ws/src/hsr_planner/msg/CellMessage.msg" NAME_WE)
add_dependencies(hsr_planner_generate_messages_lisp _hsr_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hsr_planner_genlisp)
add_dependencies(hsr_planner_genlisp hsr_planner_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hsr_planner_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(hsr_planner
  "/home/hrl/catkin_ws/src/hsr_planner/msg/ObjectMessage.msg"
  "${MSG_I_FLAGS}"
  "/home/hrl/catkin_ws/src/hsr_planner/msg/CellMessage.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hsr_planner
)
_generate_msg_nodejs(hsr_planner
  "/home/hrl/catkin_ws/src/hsr_planner/msg/ClutterPlannerServiceReq.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/MapMetaData.msg;/home/hrl/catkin_ws/src/hsr_planner/msg/CellMessage.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/OccupancyGrid.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/hrl/catkin_ws/src/hsr_planner/msg/ObjectMessage.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hsr_planner
)
_generate_msg_nodejs(hsr_planner
  "/home/hrl/catkin_ws/src/hsr_planner/msg/ClutterPlannerServiceResp.msg"
  "${MSG_I_FLAGS}"
  "/home/hrl/catkin_ws/src/hsr_planner/msg/CellMessage.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/hrl/catkin_ws/src/hsr_planner/msg/ObjectMessage.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hsr_planner
)
_generate_msg_nodejs(hsr_planner
  "/home/hrl/catkin_ws/src/hsr_planner/msg/CellMessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hsr_planner
)

### Generating Services
_generate_srv_nodejs(hsr_planner
  "/home/hrl/catkin_ws/src/hsr_planner/srv/ClutterPlannerService.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/MapMetaData.msg;/home/hrl/catkin_ws/src/hsr_planner/msg/CellMessage.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/OccupancyGrid.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/hrl/catkin_ws/src/hsr_planner/msg/ObjectMessage.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hsr_planner
)

### Generating Module File
_generate_module_nodejs(hsr_planner
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hsr_planner
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(hsr_planner_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(hsr_planner_generate_messages hsr_planner_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hrl/catkin_ws/src/hsr_planner/msg/ObjectMessage.msg" NAME_WE)
add_dependencies(hsr_planner_generate_messages_nodejs _hsr_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hrl/catkin_ws/src/hsr_planner/msg/ClutterPlannerServiceReq.msg" NAME_WE)
add_dependencies(hsr_planner_generate_messages_nodejs _hsr_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hrl/catkin_ws/src/hsr_planner/msg/ClutterPlannerServiceResp.msg" NAME_WE)
add_dependencies(hsr_planner_generate_messages_nodejs _hsr_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hrl/catkin_ws/src/hsr_planner/srv/ClutterPlannerService.srv" NAME_WE)
add_dependencies(hsr_planner_generate_messages_nodejs _hsr_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hrl/catkin_ws/src/hsr_planner/msg/CellMessage.msg" NAME_WE)
add_dependencies(hsr_planner_generate_messages_nodejs _hsr_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hsr_planner_gennodejs)
add_dependencies(hsr_planner_gennodejs hsr_planner_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hsr_planner_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(hsr_planner
  "/home/hrl/catkin_ws/src/hsr_planner/msg/ObjectMessage.msg"
  "${MSG_I_FLAGS}"
  "/home/hrl/catkin_ws/src/hsr_planner/msg/CellMessage.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hsr_planner
)
_generate_msg_py(hsr_planner
  "/home/hrl/catkin_ws/src/hsr_planner/msg/ClutterPlannerServiceReq.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/MapMetaData.msg;/home/hrl/catkin_ws/src/hsr_planner/msg/CellMessage.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/OccupancyGrid.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/hrl/catkin_ws/src/hsr_planner/msg/ObjectMessage.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hsr_planner
)
_generate_msg_py(hsr_planner
  "/home/hrl/catkin_ws/src/hsr_planner/msg/ClutterPlannerServiceResp.msg"
  "${MSG_I_FLAGS}"
  "/home/hrl/catkin_ws/src/hsr_planner/msg/CellMessage.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/hrl/catkin_ws/src/hsr_planner/msg/ObjectMessage.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hsr_planner
)
_generate_msg_py(hsr_planner
  "/home/hrl/catkin_ws/src/hsr_planner/msg/CellMessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hsr_planner
)

### Generating Services
_generate_srv_py(hsr_planner
  "/home/hrl/catkin_ws/src/hsr_planner/srv/ClutterPlannerService.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/MapMetaData.msg;/home/hrl/catkin_ws/src/hsr_planner/msg/CellMessage.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/OccupancyGrid.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/hrl/catkin_ws/src/hsr_planner/msg/ObjectMessage.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hsr_planner
)

### Generating Module File
_generate_module_py(hsr_planner
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hsr_planner
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(hsr_planner_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(hsr_planner_generate_messages hsr_planner_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hrl/catkin_ws/src/hsr_planner/msg/ObjectMessage.msg" NAME_WE)
add_dependencies(hsr_planner_generate_messages_py _hsr_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hrl/catkin_ws/src/hsr_planner/msg/ClutterPlannerServiceReq.msg" NAME_WE)
add_dependencies(hsr_planner_generate_messages_py _hsr_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hrl/catkin_ws/src/hsr_planner/msg/ClutterPlannerServiceResp.msg" NAME_WE)
add_dependencies(hsr_planner_generate_messages_py _hsr_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hrl/catkin_ws/src/hsr_planner/srv/ClutterPlannerService.srv" NAME_WE)
add_dependencies(hsr_planner_generate_messages_py _hsr_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hrl/catkin_ws/src/hsr_planner/msg/CellMessage.msg" NAME_WE)
add_dependencies(hsr_planner_generate_messages_py _hsr_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hsr_planner_genpy)
add_dependencies(hsr_planner_genpy hsr_planner_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hsr_planner_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hsr_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hsr_planner
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(hsr_planner_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(hsr_planner_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(hsr_planner_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(hsr_planner_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hsr_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hsr_planner
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(hsr_planner_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(hsr_planner_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(hsr_planner_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(hsr_planner_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hsr_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hsr_planner
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(hsr_planner_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(hsr_planner_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(hsr_planner_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(hsr_planner_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hsr_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hsr_planner
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(hsr_planner_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(hsr_planner_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(hsr_planner_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(hsr_planner_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hsr_planner)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hsr_planner\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hsr_planner
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(hsr_planner_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(hsr_planner_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(hsr_planner_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(hsr_planner_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
