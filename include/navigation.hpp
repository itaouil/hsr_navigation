#ifndef NAVIGATION_HPP_
#define NAVIGATION_HPP_

// General imports
#include <mutex>
#include <random>
#include <math.h>
#include <iostream>

// ROS msg/srv
#include <nav_msgs/Path.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <hsr_navigation/PlannerService.h>
#include <hsr_navigation/PlannerServiceReq.h>

// ROS general
#include "ros/ros.h"
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>
#include <dwa_local_planner/dwa_planner_ros.h>

// Config file with paramters
#include "parameters.hpp"

// Custom classes
#include "control.hpp"
#include "perception.hpp"

class Navigation
{
public:
    // Contructor (explicit)
    explicit Navigation(tf2_ros::Buffer&, tf2_ros::TransformListener&);

    // Destructor (virtual)
    virtual ~Navigation();

private:
    /**
     * Class methods
     */

    void initialize();
    void requestPlan();
    void loadStaticMap();
    void checkGlobalPath(const nav_msgs::OccupancyGrid);
    void populatePlannerRequest(hsr_navigation::PlannerService &);

    /**
     * Class members
     */

    // General members
    std::mutex m_mtx;
    Control *m_control = nullptr;
    Perception *m_perception = nullptr;

    // ROS members
    ros::NodeHandle m_nh;
    ros::Publisher m_velPub;
    ros::Publisher m_srvPub;
    ros::Subscriber m_costSub;
    tf2_ros::Buffer &m_buffer;
    tf2_ros::TransformListener &m_tf;
    nav_msgs::OccupancyGrid m_updatedMap;
    dwa_local_planner::DWAPlannerROS m_dp;
    nav_msgs::OccupancyGrid m_occupacyGrid;
    costmap_2d::Costmap2D *m_globalCostmap = nullptr;
    std::vector<geometry_msgs::PoseStamped> m_globalPath;
    costmap_2d::Costmap2DROS* m_localCostmapROS = nullptr;
    costmap_2d::Costmap2DROS* m_globalCostmapROS = nullptr;
};

#endif // NAVIGATION_HPP_
