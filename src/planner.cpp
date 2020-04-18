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
    requestClutterPlan(true);
}

/**
 * Destructor.
 */
Planner::~Planner()
{
    if (m_localCostmap)
        delete m_localCostmap;
    
    if (m_global)
        delete m_global;
}

/**
 * Initialize members
 */
void Planner::initialize()
{
    // Subscriber to global costmap
    m_sub = m_nodeHandle.subscribe<nav_msgs::OccupancyGrid>("/hsr_planner/global_costmap/costmap", 
                                   1000,
                                   &Planner::setGlobalCostmap,
                                   this);

    // Velocity publisher (DWA)
    m_velPub = m_nodeHandle.advertise<geometry_msgs::Twist>("/hsrb/command_velocity", 1000);

    // Service response publisher
    m_srvPub = m_nodeHandle.advertise<hsr_planner::ClutterPlannerServiceResp>("/hsr_planner/srvResponse", 1000);

    // Initialize costmaps (global and local)
    m_localCostmap = new costmap_2d::Costmap2DROS("local_costmap", m_buffer);
    m_globalCostmap = new costmap_2d::Costmap2DROS("global_costmap", m_buffer);
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
void Planner::requestClutterPlan(const bool &p_useStaticMap)
{
    // Create clutter planner client
    ros::ServiceClient l_client;
    l_client = m_nodeHandle.serviceClient<hsr_planner::ClutterPlannerService>("clutter_planner_service");

    // Service request
    hsr_planner::ClutterPlannerService l_service;
    populatePlannerRequest(l_service, p_useStaticMap);

    // Call service
    if (l_client.call(l_service))
    {
        ROS_INFO("Planner service called successfully");

        // Publish service response
        m_srvPub.publish(l_service.response);

        // Store computed path
        m_globalPath = l_service.response.path;

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
void Planner::populatePlannerRequest(hsr_planner::ClutterPlannerService &p_service,
                                     const bool &p_useStaticMap)
{
    // // Get global pose
    geometry_msgs::PoseStamped l_global_pose;
    m_global->getRobotPose(l_global_pose);

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
    // l_goal.pose.position.x = m_x_unif(m_re);
	// l_goal.pose.position.y = m_y_unif(m_re);
    l_goal.pose.position.x = 6.17;
	l_goal.pose.position.y = 2.36;
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

    if (p_useStaticMap) 
    {
        ROS_INFO("Using Static Map for planning");
        p_service.request.grid = m_occupacyGrid;
    }
    else
    {
        ROS_INFO("Using Global Costmap for planning");
        ROS_INFO_STREAM(m_updatedMap.header);
        ROS_INFO_STREAM(m_updatedMap.info);
        p_service.request.grid = m_updatedMap;
    }
}

/**
 * Check whether the computed
 * global plan is collision free
 * based on the updated global costmap.
 */
void Planner::checkGlobalPath(const nav_msgs::OccupancyGrid &p_globalCostmap)
{
    // Get global costmap
    Costmap2D *l_globalCostmap = m_globalCostmap->getCostmap();

    // Map coordinates
    unsigned int l_mx;
    unsigned int l_my;

    // Check if global path is free
    for (auto poseStamped: m_globalPath)
    {
        // World coordinates
        double l_wx = poseStamped.pose.position.x
        double l_wy = poseStamped.pose.position.y

        // Cast from world to map
        if (!l_globalCostmap->worldToMap(l_wx, l_wy, l_mx, l_my))
        {
            if (m_debug) 
            {
                ROS_ERROR("Conversion error: from world coordinates to map");
            }
        }

        // Get cost
        ROS_INFO("Cell cost: " << l_globalCostmap.getCost(l_mx, l_my));
    }
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
        m_dp.initialize("hsr_dwa_planner", &m_buffer, m_localCostmap);

        ROS_INFO("DWA has been successfully initialized...");
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
    
    // Twist msg to be populated
    // by the local planner
    geometry_msgs::Twist l_cmd_vel;

    // Planner status
    bool l_completionStatus = true;

    // Keep sending commands
    // until goal is reached
    while (!m_replan && !m_dp.isGoalReached())
    {
        // Compute local velocities
        if (!m_dp.computeVelocityCommands(l_cmd_vel))
        {
            if (m_debug)
            {
                ROS_ERROR("DWA velocities computation failed.");
            }
        }

        // Send commands
        if (m_debug)
        {
            ROS_INFO_STREAM(l_cmd_vel);
        }
        m_velPub.publish(l_cmd_vel);

        ros::spinOnce();
    }

    if (m_dp.isGoalReached())
    {
        ROS_INFO("Goal successfully reached...");
    }
    else if (m_replan)
    {
        ROS_INFO("Obstacle found on path. Re-planning now...");
        requestClutterPlan(false);
    }
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
