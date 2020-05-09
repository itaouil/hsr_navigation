#ifndef CONTROL_HPP_
#define CONTROL_HPP_

// General imports
#include <mutex>
#include <math.h>
#include <iostream>

// ROS msg/srv
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <hsr_navigation/PlannerService.h>

// ROS general
#include "ros/ros.h"
#include <costmap_2d/costmap_2d.h>
#include <tf/transform_datatypes.h>
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
    bool initialized();
    bool actionInCourse();
    void handlePlan(const hsr_navigation::PlannerService&);

private:
    /**
     * Private methods
     */
    void push();
    void grasp();
    void initialize();
    void loadStaticMap();
    void rotate(const unsigned int);
    double getRadians(const unsigned int);
    void checkOdometry(const nav_msgs::Odometry);
    void dwaControl(const std::vector<geometry_msgs::PoseStamped>&);
    void actionControl(const std::vector<geometry_msgs::PoseStamped>&);
    unsigned int getIndex(const std::vector<geometry_msgs::PoseStamped> &);

    /**
     * Private members
     */

    // General members
    bool m_push = false;
    bool m_rotate = false;
    double m_previousX = 0;
    double m_previousY = 0;
    bool m_action = false;
    bool m_firstRun = true;
    bool m_newPlan = false;
    double m_totalDistance = 0;
    bool m_initialized = false;

    // ROS members
    ros::NodeHandle m_nh;
    ros::Publisher m_velPub;
    ros::Subscriber m_odomSub;
    tf2_ros::Buffer &m_buffer;
    dwa_local_planner::DWAPlannerROS m_dp;
    costmap_2d::Costmap2DROS* m_localCostmapROS = nullptr;
    costmap_2d::Costmap2DROS* m_globalCostmapROS = nullptr;
};

#endif // CONTROL_HPP_
