#ifndef PLANNER_HPP_
#define PLANNER_HPP_

// General imports
#include <mutex>
#include <random>
#include <iostream>
#include <boost/thread/thread.hpp>

// ROS imports
#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Twist.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <dwa_local_planner/dwa_planner_ros.h>
#include <hsr_planner/ClutterPlannerService.h>
#include <hsr_planner/ClutterPlannerServiceReq.h>

class Planner
{
public:
    // Contructor (explicit)
    explicit Planner(tf2_ros::Buffer&, tf2_ros::TransformListener&);

    // Destructor (virtual)
    virtual ~Planner();

private:
    /**
     * Class methods
     */

    // ROS services
    void loadStaticMap();
    void requestClutterPlan(const bool &);

    // General
    void initialize();
    void checkGlobalPath(const nav_msgs::OccupancyGrid);
    void dwaTrajectoryControl(const hsr_planner::ClutterPlannerService &);
    void populatePlannerRequest(hsr_planner::ClutterPlannerService &, const bool &);

    /**
     * Class members
     */

    // General members
    std::mutex m_mtx;
    bool m_debug = true;
    bool m_replan = false;
    std::default_random_engine m_re;
    std::uniform_real_distribution<float> m_x_unif{1, 5.5f};
    std::uniform_real_distribution<float> m_y_unif{-0.23f, 0.9f};

    // ROS members
    ros::Subscriber m_sub;
    ros::Publisher m_velPub;
    ros::Publisher m_srvPub;
    tf2_ros::Buffer &m_buffer;
    ros::NodeHandle m_nodeHandle;
    tf2_ros::TransformListener &m_tf;
    nav_msgs::OccupancyGrid m_updatedMap;
    dwa_local_planner::DWAPlannerROS m_dp;
    nav_msgs::OccupancyGrid m_occupacyGrid;
    costmap_2d::Costmap2DROS* m_localCostmap = nullptr;
    costmap_2d::Costmap2DROS* m_globalCostmap = nullptr;
    std::vector<geometry_msgs::PoseStamped> m_globalPath;
};

#endif // PLANNER_HPP_
