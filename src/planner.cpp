// Header files
#include "planner.hpp"
#include "clutter_planner.hpp"

// General imports
#include <iostream>

/**
 * Default constructor.
 */
Planner::Planner()
{
    // Load static map
    loadStaticMap();

    // Request clutter planning
    requestClutterPlan();
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
 */
void Planner::loadStaticMap()
{
    // Create map service client
    ros::ServiceClient l_map_service_client;
    l_map_service_client = m_nodeHandle.serviceClient<nav_msgs::GetMap>("static_map");

    // Service request
    nav_msgs::GetMap l_srv_map;

    // Load map if service is
    // successful, otherwise abort
    if (l_map_service_client.call(l_srv_map))
    {
        ROS_INFO("Map service called successfully");
        m_occupacyGrid = l_srv_map.response.map;
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
    ros::ServiceClient l_client;
    l_client = m_nodeHandle.serviceClient<hsr_planner::ClutterPlannerService>("clutter_planner_service");

    // Service request
    hsr_planner::ClutterPlannerService l_service;
    populatePlannerRequest(l_service);

    // Call service
    if (l_client.call(l_service))
    {
        ROS_INFO("Clutter planner succesful response...");
    }
    else
    {
        ROS_ERROR("Failed to call service clutter_planner");
        return;
    }
}

/**
 * Populate clutter planner request.
 */
void Planner::populatePlannerRequest(hsr_planner::ClutterPlannerService &p_service)
{
    // Start pose
    geometry_msgs::PoseStamped l_start;
    l_start.header.frame_id = "map";
	l_start.pose.position.x = 1.0;
	l_start.pose.position.y = 7.5;
	l_start.pose.orientation.w = 1.0;

    // Goal pose
    geometry_msgs::PoseStamped l_goal;
    l_goal.header.frame_id = "map";
	l_goal.pose.position.x = 1.0;
	l_goal.pose.position.y = 8.0;
	l_goal.pose.orientation.w = 0.5;

    // Objects (no object for the moment)
    std::vector<hsr_planner::ObjectMessage> l_objects(0);

    // Populate request parameter by reference
	p_service.request.start = l_start;
    p_service.request.goal = l_goal;
	p_service.request.obstacles_in = l_objects;
    p_service.request.grid = m_occupacyGrid;
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