// Header files
#include "planner.hpp"
#include "navigation.hpp"

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
    requestPlan();
}

/**
 * Destructor.
 */
Navigation::~Navigation()
{
    if (m_control)
        m_control.reset();

    if (m_perception)
        m_perception.reset();

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
    // Clutter planner request publisher
    m_srvPub = m_nh.advertise<hsr_navigation::PlannerServiceReq>(PLANNER_REQ, 1);

    // Static map publisher
    m_mapPub = m_nh.advertise<nav_msgs::OccupancyGrid>(STATIC_MAP, 1);

    // Subscriber to global costmap
    m_costSub = m_nh.subscribe<nav_msgs::OccupancyGrid>(GLOBAL_COSTMAP, 
                                                        1,
                                                        &Navigation::checkGlobalPath,
                                                        this);
    
    // Initialize costmaps (global and local)
    m_localCostmapROS = new costmap_2d::Costmap2DROS("local_costmap", m_buffer);
    m_globalCostmapROS = new costmap_2d::Costmap2DROS("global_costmap", m_buffer);

    // Perception instance
    m_perception.reset(new Perception(m_buffer, m_tf));

    // Control instance
    m_control.reset(new Control(m_buffer, m_localCostmapROS, m_globalCostmapROS));

    // Wait for members initialization
    ros::Rate l_rate(10);
    while (ros::ok)
    {
        if (m_perception->initialized() && m_control->initialized())
            break;

        ros::spinOnce();
        l_rate.sleep();
    }
    
    ROS_INFO("Navigation: Initialized Correctly.");
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

        if (DEBUGNAVIGATION)
        {
            ROS_INFO("Map service called successfully");
        }
    }
    else
    {
        if (DEBUGNAVIGATION)
        {
            ROS_ERROR("Failed to call map service");
        }
    }
}

/**
 * Requests a clutter plan service.
 */
void Navigation::requestPlan()
{
    if (DEBUGNAVIGATION)
    {
        ROS_INFO("Navigation: requestPlan called.");
    }

    // Create service client
    ros::ServiceClient l_client;
    l_client = m_nh.serviceClient<hsr_navigation::PlannerService>("planner_service");

    // Populate request
    hsr_navigation::PlannerService l_service;
    populatePlannerRequest(l_service);

    // Publish request
    m_srvPub.publish(l_service.request);

    // Call service
    if (l_client.call(l_service))
    {
        // Store computed path
        m_globalPath = l_service.response.path;
        
        if (DEBUGNAVIGATION)
        {
            ROS_INFO("Planner service called successfully");
        }

        // Send velocity cmds to the robot
        m_control->handlePlan(l_service);
    }
    else
    {
        if (DEBUGNAVIGATION)
        {
            ROS_ERROR("Failed to call planner service...");
        }     
    }
}

/**
 * Populate clutter planner request.
 */
void Navigation::populatePlannerRequest(hsr_navigation::PlannerService &p_service)
{
    if (DEBUGNAVIGATION)
    {
        ROS_INFO("Navigation: populating planner request.");
    }

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
    l_goal.pose.position.x = 1.97;
	l_goal.pose.position.y = 2.47;
    l_goal.pose.position.z = 0;
    l_goal.pose.orientation.x = 0;
    l_goal.pose.orientation.y = 0;
    l_goal.pose.orientation.z = 0;
	l_goal.pose.orientation.w = 0;

    // Populate request parameter by reference
	p_service.request.start = l_start;
    p_service.request.goal = l_goal;
	p_service.request.obstacles_in = m_perception->getObstacles(m_globalCostmap);
    p_service.request.grid = m_occupacyGrid;

    // Publish static map used
    m_mapPub.publish(m_occupacyGrid);

    // Log
    if (DEBUGNAVIGATION)
    {
        ROS_INFO("Planner request populated successfully");
    }
}

/**
 * Check whether the computed
 * global plan is collision free
 * based on the updated global costmap.
 */
void Navigation::checkGlobalPath(const nav_msgs::OccupancyGrid p_globalCostmap)
{
    // Replan flag
    bool replan = false;

    // Update map (globacl costmap)
    m_updatedMap = p_globalCostmap;

    // Update global costmap
    m_globalCostmap = m_globalCostmapROS->getCostmap();

    // Only check for obstacle if
    // action was not commanded by
    // new clutter planner plan
    if (!m_control->actionInCourse())
    {
        if (DEBUGNAVIGATION)
        {
            ROS_INFO("action is false so I am checking...");
        }

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
            if (l_cellCost > 252)
            {
                if (DEBUGNAVIGATION)
                {
                    ROS_INFO("Obstacle detected on the path.");
                    std::cout << "GCM wx: " << l_wx << std::endl;
                    std::cout << "GCM wy: " << l_wy << std::endl;
                    std::cout << "GCM x: " << l_mx << std::endl;
                    std::cout << "GCM y: " << l_my << std::endl;
                }

                // Set replan flag
                replan = true;

                break;
            }
        }

        // Handle replan
        if (replan || m_control->postActionPlan())
        {
            // Stop control
            m_control->stopControl();

            // Replan
            requestPlan();
        }
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
