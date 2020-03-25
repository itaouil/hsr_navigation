// Header files
#include "planner.hpp"
#include "clutter_planner.hpp"

/**
 * Default constructor.
 */
Planner::Planner()
{
    // Initialize members
    initialize();

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
 * Initialize members
 */
void Planner::initialize()
{
    // Create path publisher
    m_pub = m_nodeHandle.advertise<nav_msgs::Path>("/base_local_path", 1);

    // Create velocity publisher
    m_velPub = m_nodeHandle.advertise<geometry_msgs::Twist>("/hsrb/command_velocity", 1);
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
        //publishServicePlan(l_service);

        // Send velocity cmds to the robot
        dwaTrajectoryControl(l_service);
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
    // // Get global pose
    // geometry_msgs::PoseStamped l_global_pose;
    // m_costMap->getRobotPose(l_global_pose);

    // Start pose
    geometry_msgs::PoseStamped l_start;
    l_start.header.frame_id = "map";
    l_start.pose.position.x = 0;
	l_start.pose.position.y = 0;
    l_start.pose.position.z = 0;
    l_start.pose.orientation.x = 0;
    l_start.pose.orientation.y = 0;
    l_start.pose.orientation.z = 0;
	l_start.pose.orientation.w = 0;
	// l_start.pose.position.x = l_global_pose.pose.position.x;
	// l_start.pose.position.y = l_global_pose.pose.position.y;
    // l_start.pose.position.z = l_global_pose.pose.position.z;
    // l_start.pose.orientation.x = l_global_pose.pose.orientation.x;
    // l_start.pose.orientation.y = l_global_pose.pose.orientation.y;
    // l_start.pose.orientation.z = l_global_pose.pose.orientation.z;
	// l_start.pose.orientation.w = l_global_pose.pose.orientation.w;

    // Goal pose
    geometry_msgs::PoseStamped l_goal;
    l_goal.header.frame_id = "map";
    l_goal.pose.position.x = 0.439;
	l_goal.pose.position.y = 0.274;
    l_goal.pose.position.z = 0;
    l_goal.pose.orientation.x = 0;
    l_goal.pose.orientation.y = 0;
    l_goal.pose.orientation.z = 0;
	l_goal.pose.orientation.w = 0;

    // Objects (no object for the moment)
    std::vector<hsr_planner::ObjectMessage> l_objects(0);

    // Populate request parameter by reference
	p_service.request.start = l_start;
    p_service.request.goal = l_goal;
	p_service.request.obstacles_in = l_objects;
    p_service.request.grid = m_occupacyGrid;
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
 * Compute velocity commands to
 * be sent to the robot in order
 * to reach the goal.
 */
void Planner::dwaTrajectoryControl(const hsr_planner::ClutterPlannerService &p_service)
{
    // TF2 objects
    tf2_ros::Buffer l_buffer(ros::Duration(10));
    tf2_ros::TransformListener l_tf(l_buffer);

    // Create costmap
    costmap_2d::Costmap2DROS l_costmap("local_costmap", l_buffer);

    // Check that planner is initialized
    if (!m_dp.isInitialized())
    {
        // Initialize dwa local planner
        m_dp.initialize("hsr_dwa_planner", &l_buffer, &l_costmap);

        ROS_INFO("DWA has been initialized successfully...");
    }

    // Set global plan for local planner
    if (m_dp.setPlan(p_service.response.path))
    {
        ROS_INFO("DWA set plan: SUCCESS");
    }
    else
    {
        ROS_ERROR("DWA set plan: FAILED");
    }
    
    // Create twist messag to be
    // populate by the local planner
    geometry_msgs::Twist l_cmd_vel;
    // l_cmd_vel.linear.x = 0;
    // l_cmd_vel.linear.y = 0;
    // l_cmd_vel.linear.z = 0;
    // l_cmd_vel.angular.x = 0;
    // l_cmd_vel.angular.y = 0;
    // l_cmd_vel.angular.z = 0;

    // Get robot pose in the map
    geometry_msgs::PoseStamped l_global_pose;
    l_costmap.getRobotPose(l_global_pose);

    // Keep sending commands
    // until goal is reached
    while (!m_dp.isGoalReached())
    {
        // Compute velocity commands
        if (m_dp.computeVelocityCommands(l_cmd_vel))
        {
            ROS_INFO("DWA compute cmd_vel: SUCCESS");
        }
        else
        {
            ROS_ERROR("DWA compute cmd_vel: FAILED");
        }

        // Send commands
        m_velPub.publish(l_cmd_vel);
    }
}

int main(int argc, char **argv)
{
    // Create ROS node
    ros::init(argc, argv, "planner");
    ROS_INFO("Created Planner node...");

    // Create Planner instance
    Planner planner;

    // Spin ROS
    ros::spin();

    return 0;
}