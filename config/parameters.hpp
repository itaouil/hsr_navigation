#ifndef PARAMETERS_HPP_
#define PARAMETERS_HPP_

#include <iostream>

// General parameters
const bool DEBUG(true);

// Planner parameters
const bool BACKUPPLAN(false);
const float ROBOTRADIUS(0.215);
const unsigned int OBJMOVCOST(2);
const float INFLATIONRADIUS(0.8);
const unsigned int MOVINGCOST(11);
const unsigned int ENTERCOST(100);
const unsigned int CHANGEOBJECTCOST(250);

// Navigation parameters
const std::string PLANNER_REQ("/hsr_navigation/srvReq");
const std::string GLOBAL_COSTMAP("/hsr_navigation/global_costmap/costmap");

// Perception parameters
const std::string FRAME_ID("head_rgbd_sensor_rgb_frame");
const std::string CAMERA_INFO("/hsrb/head_rgbd_sensor/rgb/camera_info");
const std::string RGB_DATA("/hsrb/head_rgbd_sensor/rgb/image_rect_color");
const std::string DEPTH_DATA("/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw");

// Control parameters
const double DISTANCE(0.473);
const std::string ODOMETRY("/hsrb/odom");
const std::string DWA_VELOCITIES("/hsrb/command_velocity");

#endif
