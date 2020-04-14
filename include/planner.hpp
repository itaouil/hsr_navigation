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
    void populatePlannerRequest(hsr_planner::ClutterPlannerService &, const bool &);
    void dwaTrajectoryControl(const hsr_planner::ClutterPlannerService &);

    /**
     * Class members
     */

    // Setters
    inline void setGlobalCostmap(const nav_msgs::OccupancyGrid p_globalCostmap)
    {
        m_mtx.lock();
        m_globalCostmap = p_globalCostmap;
        m_mtx.unlock();
    }

    // General members
    std::mutex m_mtx;
    std::default_random_engine m_re;
    std::uniform_real_distribution<float> m_x_unif{1, 5.5f};
    std::uniform_real_distribution<float> m_y_unif{-0.23f, 0.9f};

    // ROS members
    ros::Publisher m_sub;
    ros::Publisher m_velPub;
    tf2_ros::Buffer &m_buffer;
    ros::NodeHandle m_nodeHandle;
    tf2_ros::TransformListener &m_tf;
    dwa_local_planner::DWAPlannerROS m_dp;
    nav_msgs::OccupancyGrid m_occupacyGrid;
    nav_msgs::OccupancyGrid m_globalCostmap;
    costmap_2d::Costmap2DROS* m_local = nullptr;
    costmap_2d::Costmap2DROS* m_global = nullptr;
};

#endif // PLANNER_HPP_
