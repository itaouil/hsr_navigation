#ifndef NAVIGATION_PARAMS_HPP_
#define NAVIGATION_PARAMS_HPP_

#include <iostream>

// Publisher/Subscribers
const std::string CP_PUB_TOPIC("/hsr_navigation/srvReq");
const std::string DWA_PUB_TOPIC("/hsrb/command_velocity");
const std::string GC_SUB_TOPIC("/hsr_navigation/global_costmap/costmap");
const std::string MODEL_SUB_TOPIC("/hsrb/head_rgbd_sensor/rgb/camera_info");
const std::string RGB_SUB_TOPIC("/hsrb/head_rgbd_sensor/rgb/image_rect_color");
const std::string DEPTH_SUB_TOPIC("/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw");

// General
const std::string FRAME_ID("head_rgbd_sensor_rgb_frame");

#endif
