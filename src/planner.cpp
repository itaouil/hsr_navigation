// Header files
#include "planner.hpp"
#include "clutter_planner.hpp"

/**
 * Default constructor.
 */
Planner::Planner(tf2_ros::Buffer &p_buffer, tf2_ros::TransformListener &p_tf):
    m_buffer(p_buffer), m_tf(p_tf)
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
    if (m_local)
        delete m_local;
    
    if (m_global)
        delete m_global;
}

/**
 * Initialize members
 */
void Planner::initialize()
{
    // Create path publisher
    m_pub = m_nodeHandle.advertise<nav_msgs::Path>("/base_local_path", 1000);

    // Create velocity publisher
    m_velPub = m_nodeHandle.advertise<geometry_msgs::Twist>("/hsrb/command_velocity", 1000);

    // Initialize costmaps (global and local)
    m_local = new costmap_2d::Costmap2DROS("local_costmap", m_buffer);
    m_global = new costmap_2d::Costmap2DROS("global_costmap", m_buffer);

    // Start costmaps
    //m_local->start();
    //m_global->start();
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

    // Load map
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
        publishServicePlan(l_service);

        // Send velocity cmds to the robot
        dwaTrajectoryControl(l_service);
    }
    else
    {
        ROS_ERROR("Failed to call service clutter_planner");
    }
}

/**
 * Populate clutter planner request.
 */
void Planner::populatePlannerRequest(hsr_planner::ClutterPlannerService &p_service)
{
    // // Get global pose
    geometry_msgs::PoseStamped l_global_pose;
    geometry_msgs::PoseStamped l_global_pose_local;
    m_global->getRobotPose(l_global_pose);
    m_local->getRobotPose(l_global_pose_local);
    ROS_INFO_STREAM(l_global_pose_local);

    // Start pose
    geometry_msgs::PoseStamped l_start;
    l_start.header.frame_id = "map";
	l_start.pose.position.x = l_global_pose.pose.position.x;
	l_start.pose.position.y = l_global_pose.pose.position.y;
    l_start.pose.position.z = l_global_pose.pose.position.z;
    l_start.pose.orientation.x = l_global_pose.pose.orientation.x;
    l_start.pose.orientation.y = l_global_pose.pose.orientation.y;
    l_start.pose.orientation.z = l_global_pose.pose.orientation.z;
	l_start.pose.orientation.w = l_global_pose.pose.orientation.w;

    // Goal pose
    geometry_msgs::PoseStamped l_goal;
    l_goal.header.frame_id = "map";
    l_goal.pose.position.x = m_x_unif(m_re);
	l_goal.pose.position.y = m_y_unif(m_re);
    l_goal.pose.position.z = 0;
    l_goal.pose.orientation.x = 0;
    l_goal.pose.orientation.y = 0;
    l_goal.pose.orientation.z = 0;
	l_goal.pose.orientation.w = 0;

    ROS_INFO_STREAM(l_start);
    ROS_INFO_STREAM(l_goal);

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
    // Check that planner is initialized
    if (!m_dp.isInitialized())
    {
        // Initialize dwa local planner
        m_dp.initialize("hsr_dwa_planner", &m_buffer, m_local);

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

    // Get robot pose in the map
    geometry_msgs::PoseStamped l_global_pose;
    m_global->getRobotPose(l_global_pose);
    ROS_INFO_STREAM(l_global_pose);

    // Keep sending commands
    // until goal is reached
    while (!m_dp.isGoalReached())
    {
        // Update costmaps
        //m_local->updateMap();
        //m_global->updateMap();

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
        ROS_INFO_STREAM(l_cmd_vel);
        m_velPub.publish(l_cmd_vel);
    }

    ROS_INFO("Goal was reached. New pose will be set now...");
    requestClutterPlan();
}

int main(int argc, char **argv)
{
    // Create ROS node
    ros::init(argc, argv, "planner");
    ROS_INFO("Created Planner node...");

    // TF2 objects
    tf2_ros::Buffer l_buffer(ros::Duration(10));
    tf2_ros::TransformListener l_tf(l_buffer);

    // Create Planner instance
    Planner planner(l_buffer, l_tf);

    // Spin ROS
    ros::spin();

    return 0;
}
