#ifndef PLANNER_HPP_
#define PLANNER_HPP_

// ROS imports
#include "ros/ros.h"
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
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

    // General
    void populatePlannerRequest(hsr_planner::ClutterPlannerService &);

    /**
     * Class members
     */

    // ROS members
    ros::NodeHandle m_nodeHandle;
    nav_msgs::OccupancyGrid m_occupacyGrid;
};

#endif // PLANNER_HPP_
