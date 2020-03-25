#ifndef PLANNER_HPP_
#define PLANNER_HPP_

// General imports
#include <iostream>
#include <boost/thread/thread.hpp>

// ROS imports
#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <dwa_local_planner/dwa_planner_ros.h>
#include <hsr_planner/ClutterPlannerService.h>

class Planner
{
public:
    // Contructor (explicit)
    explicit Planner();

    // Destructor (virtual)
    virtual ~Planner();

private:
    /**
     * Class methods
     */

    // ROS services
    void loadStaticMap();
    void requestClutterPlan();
    
    // ROS publishers
    void publishServicePlan(const hsr_planner::ClutterPlannerService &);

    // General
    void initialize();
    void populatePlannerRequest(hsr_planner::ClutterPlannerService &);
    void dwaTrajectoryControl(const hsr_planner::ClutterPlannerService &);

    /**
     * Class members
     */

    // ROS members
    ros::Publisher m_pub;
    ros::Publisher m_velPub;
    ros::NodeHandle m_nodeHandle;
    dwa_local_planner::DWAPlannerROS m_dp;
    nav_msgs::OccupancyGrid m_occupacyGrid;
    costmap_2d::Costmap2DROS* m_costMap = nullptr;
};

#endif // PLANNER_HPP_
