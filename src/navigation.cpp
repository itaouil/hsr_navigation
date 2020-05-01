// Header files
#include "navigation.hpp"
#include "clutter_planner.hpp"

/**
 * Default constructor.
 */
Navigation::Navigation(tf2_ros::Buffer &p_buffer, tf2_ros::TransformListener &p_tf):
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
Navigation::~Navigation()
{
    if (m_localCostmapROS)
        delete m_localCostmapROS;
    
    if (m_globalCostmapROS)
        delete m_globalCostmapROS;
}

/**
 * Initialize members
 */
void Navigation::initialize()
{
    // Perception instance
    m_perception = new Perception();

    // DWA planner velocity publisher
    m_velPub = m_nh.advertise<geometry_msgs::Twist>(DWA_VELOCITIES, 1);

    // Clutter planner request publisher
    m_srvPub = m_nh.advertise<hsr_navigation::ClutterPlannerServiceReq>(PLANNER_REQ, 1);

    // Subscriber to global costmap
    m_costSub = m_nh.subscribe<nav_msgs::OccupancyGrid>(GLOBAL_COSTMAP, 
                                                        1,
                                                        &Navigation::checkGlobalPath,
                                                        this);
    
    // Initialize costmaps (global and local)
    m_localCostmapROS = new costmap_2d::Costmap2DROS("local_costmap", m_buffer);
    m_globalCostmapROS = new costmap_2d::Costmap2DROS("global_costmap", m_buffer);

    // Initialize dwa local planner
    m_dp.initialize("hsr_dwa_planner", &m_buffer, m_localCostmapROS);
}

/**
 * Load static map used
 * as input to the Clutter
 */
void Navigation::loadStaticMap()
{
    // Create map service client
    ros::ServiceClient l_map_service_client;
    l_map_service_client = m_nh.serviceClient<nav_msgs::GetMap>("static_map");

    // Service request
    nav_msgs::GetMap l_srv_map;

    // Load map
    if (l_map_service_client.call(l_srv_map))
    {
        m_occupacyGrid = l_srv_map.response.map;

        if (DEBUG)
        {
            ROS_INFO("Map service called successfully");
        }
    }
    else
    {
        if (DEBUG)
        {
            ROS_ERROR("Failed to call map service");
        }
    }
}

/**
 * Requests a clutter plan service.
 */
void Navigation::requestClutterPlan()
{
    // Create service client
    ros::ServiceClient l_client;
    l_client = m_nh.serviceClient<hsr_navigation::ClutterPlannerService>("clutter_planner_service");

    // Populate request
    hsr_navigation::ClutterPlannerService l_service;
    populatePlannerRequest(l_service);

    // Publish request
    m_srvPub.publish(l_service.request);

    // Call service
    if (l_client.call(l_service))
    {
        // Store computed path
        m_globalPath = l_service.response.path;
        
        if (DEBUG)
        {
            ROS_INFO("Planner service called successfully");
        }

        // Send velocity cmds to the robot
        dwaTrajectoryControl(l_service);
    }
    else
    {
        if (DEBUG)
        {
            ROS_ERROR("Failed to call planner service...");
        }     
    }
}

/**
 * Populate clutter planner request.
 */
void Navigation::populatePlannerRequest(hsr_navigation::ClutterPlannerService &p_service)
{
    // Get global pose
    geometry_msgs::PoseStamped l_global_pose;
    m_globalCostmapROS->getRobotPose(l_global_pose);

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

    // Populate request parameter by reference
	p_service.request.start = l_start;
    p_service.request.goal = l_goal;
	p_service.request.obstacles_in = m_perception->getObstacles(m_tf, m_globalCostmap);
    p_service.request.grid = m_occupacyGrid;
}

/**
 * Check whether the computed
 * global plan is collision free
 * based on the updated global costmap.
 */
void Navigation::checkGlobalPath(const nav_msgs::OccupancyGrid p_globalCostmap)
{
    // Update global costmap
    m_globalCostmap = m_globalCostmapROS->getCostmap();

    // Only check for obstacle if
    // action was not commanded by
    // new clutter planner plan
    if (!m_action)
    {
        // Map coordinates
        int l_mx;
        int l_my;

        // Check if global path is free
        for (auto poseStamped: m_globalPath)
        {
            // World coordinates
            double l_wx = poseStamped.pose.position.x;
            double l_wy = poseStamped.pose.position.y;

            // Cast from world to map
            m_globalCostmap->worldToMapEnforceBounds(l_wx, l_wy, l_mx, l_my);

            // Get cost (convert to int from unsigned char)
            int l_cellCost = (int) m_globalCostmap->getCost(l_mx, l_my);

            // Log cost
            if (l_cellCost > 253)
            {
                ROS_INFO("Obstacle detected on the path.");
                m_replan = true;
                break;
            }
        }

        // Update map for replan
        m_updatedMap = p_globalCostmap;
    }
}

/**
 * Compute velocity commands to
 * be sent to the robot in order
 * to reach the goal.
 */
void Navigation::dwaTrajectoryControl(const hsr_navigation::ClutterPlannerService &p_service)
{
    // Check if plan consists of
    // manipulating the environment
    // or not
    if (!p_service.response.obstacles_out.empty())
    {
        //TODO: set intermediate point
        //TODO: call manipulation action
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

    // Keep sending commands
    // until goal is reached
    while (!m_replan && !m_dp.isGoalReached())
    {
        // Compute local velocities
        if (!m_dp.computeVelocityCommands(l_cmd_vel))
        {
            if (DEBUG)
            {
                ROS_ERROR("DWA velocities computation failed.");
            }
        }

        // Send commands
        if (DEBUG)
        {
            //ROS_INFO_STREAM(l_cmd_vel);
        }
        m_velPub.publish(l_cmd_vel);

        ros::spinOnce();
    }

    if (m_dp.isGoalReached())
    {
        ROS_INFO("GOAL REACHED :)");
    }
    else
    {
        ROS_INFO("STOPPING DWA FOR REPLANNING...");
    }
}

int main(int argc, char **argv)
{
    // Create ROS node
    ros::init(argc, argv, "navigation");
    ROS_INFO("Created navigation node...");

    // TF2 objects
    tf2_ros::Buffer l_buffer(ros::Duration(10));
    tf2_ros::TransformListener l_tf(l_buffer);

    // Create Navigation instance
    Navigation navigation(l_buffer, l_tf);

    // Spin ROS
    ros::spin();

    return 0;
}
