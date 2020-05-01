#ifndef PARAMETERS_HPP_
#define PARAMETERS_HPP_

#include <iostream>

// General parameters
const bool DEBUG(true);

// Navigation parameters
const std::string PLANNER_REQ("/hsr_navigation/srvReq");
const std::string DWA_VELOCITIES("/hsrb/command_velocity");
const std::string GLOBAL_COSTMAP("/hsr_navigation/global_costmap/costmap");

// Perception parameters
const std::string FRAME_ID("head_rgbd_sensor_rgb_frame");
const std::string CAMERA_INFO("/hsrb/head_rgbd_sensor/rgb/camera_info");
const std::string RGB_DATA("/hsrb/head_rgbd_sensor/rgb/image_rect_color");
const std::string DEPTH_DATA("/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw");

#endif
