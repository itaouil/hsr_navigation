cmake_minimum_required(VERSION 2.8.3)
project(hsr_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  tf2_ros
  std_msgs
  nav_msgs
  cv_bridge
  pluginlib
  costmap_2d
  sensor_msgs
  geometry_msgs
  actionlib_msgs
  eigen_conversions
  dwa_local_planner
  message_generation
)

#find_package(OpenCV REQUIRED)
set(OpenCV_LIBS opencv_core opencv_imgproc opencv_calib3d opencv_video opencv_features2d opencv_ml opencv_highgui)


################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
    CellMessage.msg
    ObjectMessage.msg
    PlannerServiceReq.msg
    PlannerServiceResp.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  PlannerService.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs
    nav_msgs
    geometry_msgs
    actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    config
    include
  LIBRARIES hsr_navigation
  CATKIN_DEPENDS 
    roscpp 
    message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  config
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(control src/control.cpp)
target_link_libraries(control ${catkin_LIBRARIES})

add_executable(planner src/planner.cpp)
target_link_libraries(planner ${catkin_LIBRARIES})

add_executable(navigation src/navigation.cpp)
target_link_libraries(navigation ${catkin_LIBRARIES})

add_executable(perception src/perception.cpp)
target_link_libraries(perception ${catkin_LIBRARIES} ${OpenCV_LIBS})