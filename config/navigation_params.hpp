#ifndef NAVIGATION_PARAMS_HPP_
#define NAVIGATION_PARAMS_HPP_

#include <iostream>

const std::string CP_PUB_TOPIC("/hsr_planner/srvReq");
const std::string DWA_PUB_TOPIC("/hsrb/command_velocity");
const std::string GC_SUB_TOPIC("/hsr_planner/global_costmap/costmap");
const std::string RGB_SUB_TOPIC("/hsrb/head_rgbd_sensor/rgb/image_rect_color");
const std::string DEPTH_SUB_TOPIC("/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw");

#endif
