#ifndef CONTROL_HPP_
#define CONTROL_HPP_

// General imports
#include <mutex>
#include <math.h>
#include <iostream>

// ROS msg/srv
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>

// ROS general
#include "ros/ros.h"
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <dwa_local_planner/dwa_planner_ros.h>

// Config file with paramters
#include "parameters.hpp"

class Control
{
public:
    // Contructor (explicit)
    explicit Control(tf2_ros::Buffer &, costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS*);

    // Destructor (virtual)
    virtual ~Control();

    /**
     * Public methods
     */
    void setNewPlan();
    bool actionInCourse();
    void handlePlan(const hsr_navigation::PlannerService&);

private:
    /**
     * Private methods
     */

    void initialize();
    void loadStaticMap();
    void actionControl(const hsr_navigation::PlannerService&);
    void dwaControl(const std::vector<geometry_msgs::PoseStamped>&);

    /**
     * Private members
     */

    // General members
    std::mutex m_mtx;
    bool m_action = false;
    bool m_newPlan = false;

    // ROS members
    ros::Publisher m_velPub;
    tf2_ros::Buffer &m_buffer;
    dwa_local_planner::DWAPlannerROS m_dp;
    costmap_2d::Costmap2DROS* m_localCostmapROS = nullptr;
    costmap_2d::Costmap2DROS* m_globalCostmapROS = nullptr;
};

#endif // CONTROL_HPP_
