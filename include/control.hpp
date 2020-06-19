#ifndef CONTROL_HPP_
#define CONTROL_HPP_

// General imports
#include <mutex>
#include <math.h>
#include <iostream>
#include <Python.h>

// ROS msg/srv
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <hsr_navigation/CellMessage.h>
#include <geometry_msgs/PoseStamped.h>
#include <hsr_navigation/ObjectMessage.h>
#include <hsr_navigation/ActionService.h>
#include <hsr_navigation/PlannerService.h>
#include <geometry_msgs/TransformStamped.h>

// ROS general
#include "ros/ros.h"
#include <costmap_2d/costmap_2d.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>
#include <dwa_local_planner/dwa_planner_ros.h>

// Config file with paramters
#include "parameters.hpp"

class Control
{
public:
    // Contructor (explicit)
    explicit Control(tf2_ros::Buffer &, 
                     tf2_ros::TransformListener&, 
                     costmap_2d::Costmap2DROS*, 
                     costmap_2d::Costmap2DROS*);

    // Destructor (virtual)
    virtual ~Control();

    /**
     * Public methods
     */
    void stopControl();
    bool initialized();
    bool postActionPlan();
    bool actionInCourse();
    void setObjectFrame(const double, const double);
    void handlePlan(const hsr_navigation::PlannerService&);

private:
    /**
     * Private methods
     */
    void initialize();
    void loadStaticMap();
    void performAction(const std::string&);
    void setOdometry(const nav_msgs::Odometry);
    void dwaControl(const std::vector<geometry_msgs::PoseStamped>&);
    float distance(geometry_msgs::PoseStamped, hsr_navigation::CellMessage);
    void actionControl(const std::vector<geometry_msgs::PoseStamped>&,
                       const std::vector<hsr_navigation::ObjectMessage>&);
    unsigned int getIndex(const std::vector<geometry_msgs::PoseStamped> &);

    /**
     * Private members
     */

    // General members
    bool m_pushAction = false;
    bool m_kickAction = false;
    bool m_graspAction = false;
    bool m_stopControl = false;
    bool m_initialized = false;
    bool m_publishFrame = false;
    bool m_postActionPlan = false;

    // ROS members
    ros::Rate m_rate{10};
    ros::NodeHandle m_nh;
    ros::Publisher m_velPub;
    tf::Transform m_transform;
    ros::Subscriber m_odomSub;
    tf2_ros::Buffer &m_buffer;
    tf2_ros::TransformListener &m_tf;
    dwa_local_planner::DWAPlannerROS m_dp;
    tf::TransformBroadcaster m_broadcaster;
    costmap_2d::Costmap2DROS* m_localCostmapROS = nullptr;
    costmap_2d::Costmap2DROS* m_globalCostmapROS = nullptr;
};

#endif // CONTROL_HPP_
