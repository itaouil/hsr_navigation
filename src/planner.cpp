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
    // Create path publisher
    m_pub = m_nodeHandle.advertise<nav_msgs::Path>("/base_local_path", 1);

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
        ROS_INFO("Planner service called successfully");

        // Publish service plan on RVIZ
        publishServicePlan(l_service);
    }
    else
    {
        ROS_ERROR("Failed to call service clutter_planner");
        return;
    }
}

/**
 * Publishes the service plan response.
 */
void Planner::publishServicePlan(const hsr_planner::ClutterPlannerService &p_service)
{
    // Create navigation message
    nav_msgs::Path l_navPath;

    // Populate path
    l_navPath.header.stamp = ros::Time::now();;
    l_navPath.header.frame_id = "map";
    l_navPath.poses = p_service.response.path;

    m_pub.publish(l_navPath);
}

/**
 * Populate clutter planner request.
 */
void Planner::populatePlannerRequest(hsr_planner::ClutterPlannerService &p_service)
{
    // Start pose
    geometry_msgs::PoseStamped l_start;
    l_start.header.frame_id = "map";
	l_start.pose.position.x = 0.02;
	l_start.pose.position.y = 0.06;
	l_start.pose.orientation.w = 1.0;

    // Goal pose
    geometry_msgs::PoseStamped l_goal;
    l_goal.header.frame_id = "map";
	l_goal.pose.position.x = 6.15;
	l_goal.pose.position.y = 7;
	l_goal.pose.orientation.w = 0.00185;

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
