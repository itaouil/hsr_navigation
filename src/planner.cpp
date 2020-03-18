// Header files
#include "planner.hpp"
#include "clutter_planner.hpp"

// General imports
#include <iostream>

#include <hsr_planner/ClutterPlannerService.h>

/**
 * Default constructor.
 */
Planner::Planner()
{
    // Load static map
    loadStaticMap();
}

/** 
 * Destructor.
 */
Planner::~Planner()
{
}

/**
 * Load static map used
 * as input to the Clutter
 * 
 */
void Planner::loadStaticMap()
{
    // Create map service client
    ros::ServiceClient l_map_service_client;
    l_map_service_client = m_nodeHandle.serviceClient<nav_msgs::GetMap>("map");

    // Service response
    nav_msgs::GetMap l_srv_map;

    // Load map if service is
    // successful, otherwise abort
    if (l_map_service_client.call(l_srv_map))
    {
        ROS_INFO("Map service called successfully");
        const nav_msgs::OccupancyGrid &m_occupacyGrid(l_srv_map.response.map);
    }
    else
    {
        ROS_ERROR("Failed to call map service");
        return;
    }
}

/**
 * Requests a clutter plan service.
 */
void Planner::requestClutterPlan()
{
    // Create clutter planner client
    ros::ServiceClient l_clutter_planner_client;
    l_clutter_planner_client = m_nodeHandle.serviceClient
}

int main(int argc, char **argv)
{
    // Create ROS node
    ros::init(argc, argv, "planner");
    ROS_INFO("Created Planner node...");

    // Create Planner instance
    Planner planner = Planner();

    // Spin ROS
    ros::spin();

    return 0;
}