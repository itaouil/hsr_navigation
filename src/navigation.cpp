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
    requestClutterPlan(true);
}

/**
 * Destructor.
 */
Navigation::~Navigation()
{
    if (m_localCostmap)
        delete m_localCostmap;
    
    if (m_globalCostmap)
        delete m_globalCostmap;
}

/**
 * Initialize members
 */
void Navigation::initialize()
{
    // DWA planner velocity publisher
    m_velPub = m_nh.advertise<geometry_msgs::Twist>(DWA_PUB_TOPIC, 1);

    // Clutter planner request publisher
    m_srvPub = m_nh.advertise<hsr_navigation::ClutterPlannerServiceReq>(CP_PUB_TOPIC, 1);

    // Subscriber to global costmap
    m_costSub = m_nh.subscribe<nav_msgs::OccupancyGrid>(GC_SUB_TOPIC, 
                                   1,
                                   &Navigation::checkGlobalPath,
                                   this);

    // Synchronize RGB and Depth subscribers
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(m_nh, RGB_SUB_TOPIC, 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(m_nh, DEPTH_SUB_TOPIC, 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(rgb_sub, 
                                                                                   depth_sub, 
                                                                                   10);
    sync.registerCallback(boost::bind(&Navigation::perceptionCallback,
                                      this, 
                                      _1, 
                                      _2));

    // Initialize costmaps (global and local)
    m_localCostmap = new costmap_2d::Costmap2DROS("local_costmap", m_buffer);
    m_globalCostmap = new costmap_2d::Costmap2DROS("global_costmap", m_buffer);

    // Initialize dwa local planner
    m_dp.initialize("hsr_dwa_planner", &m_buffer, m_localCostmap);
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
void Navigation::requestClutterPlan(const bool &p_useStaticMap)
{
    // Create clutter planner client
    ros::ServiceClient l_client;
    l_client = m_nh.serviceClient<hsr_navigation::ClutterPlannerService>("clutter_planner_service");

    // Service request
    hsr_navigation::ClutterPlannerService l_service;
    populatePlannerRequest(l_service, p_useStaticMap);

    // Publish service request
    m_srvPub.publish(l_service.request);

    // Call service
    if (l_client.call(l_service))
    {
        ROS_INFO("Planner service called successfully");

        // Store computed path
        m_globalPath = l_service.response.path;

        // Send velocity cmds to the robot
        dwaTrajectoryControl(l_service);
    }
    else
    {
        ROS_ERROR("Failed to call service clutter_planner");

        if (m_debug)
        {
            // Get global costmap
            costmap_2d::Costmap2D *l_globalCostmap = m_globalCostmap->getCostmap();

            // Map coordinates
            int l_mx;
            int l_my;

            // Robot global pose
            geometry_msgs::PoseStamped l_global_pose;
            m_globalCostmap->getRobotPose(l_global_pose);

            // World coordinates
            double l_wx = l_global_pose.pose.position.x;
            double l_wy = l_global_pose.pose.position.y;

            // Cast from world to map
            l_globalCostmap->worldToMapEnforceBounds(l_wx, l_wy, l_mx, l_my);

            // Get cost (convert to int from unsigned char)
            int l_cellCost = (int) l_globalCostmap->getCost(l_mx, l_my);

            std::cout << "Robot Pose Cost " << l_cellCost << std::endl;
        }
    }
}

/**
 * Populate clutter planner request.
 */
void Navigation::populatePlannerRequest(hsr_navigation::ClutterPlannerService &p_service,
                                     const bool &p_useStaticMap)
{
    // // Get global pose
    geometry_msgs::PoseStamped l_global_pose;
    m_globalCostmap->getRobotPose(l_global_pose);

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
    std::vector<hsr_navigation::ObjectMessage> l_objects(0);

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
        p_service.request.grid = m_updatedMap;
    }
}

/**
 * Check whether the computed
 * global plan is collision free
 * based on the updated global costmap.
 */
void Navigation::checkGlobalPath(const nav_msgs::OccupancyGrid p_globalCostmap)
{
    // Get global costmap
    costmap_2d::Costmap2D *l_globalCostmap = m_globalCostmap->getCostmap();

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
        l_globalCostmap->worldToMapEnforceBounds(l_wx, l_wy, l_mx, l_my);

        // Get cost (convert to int from unsigned char)
        int l_cellCost = (int) l_globalCostmap->getCost(l_mx, l_my);

        // Log cost
        if (l_cellCost > 253)
        {
            ROS_INFO("Obstacle detected on the path. Starting perception...");
            std::cout << l_cellCost << std::endl;
            m_replan = true;
            break;
        }
    }

    // Update map for replan
    m_updatedMap = p_globalCostmap;
}

void Navigation::perceptionCallback(const sensor_msgs::ImageConstPtr& p_rgb, 
                                    const sensor_msgs::ImageConstPtr& p_depth)
{
    // Convert ROS image to OpenCV Mat
    try
    {
      m_cvPtr = cv_bridge::toCvCopy(p_rgb, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    // ConverT BGR to HSV
    cv::Mat l_hsv;
    cv::cvtColor(m_cvPtr->image, l_hsv, cv::COLOR_BGR2HSV);

    // Red color masks
    cv::Mat l_mask1;
    cv::Mat l_mask2;

    // Creating masks to detect the upper and lower red color.
    cv::inRange(l_hsv, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), l_mask1);
    cv::inRange(l_hsv, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), l_mask2);

    // Generate final mask
    l_mask1 = l_mask1 + l_mask2;

    cv::Mat l_kernel = cv::Mat::ones(3,3, CV_32F);
    cv::morphologyEx(l_mask1, l_mask1, cv::MORPH_OPEN, l_kernel);
    cv::morphologyEx(l_mask1, l_mask1, cv::MORPH_DILATE, l_kernel);

    // creating an inverted mask to segment out the cloth from the frame
    cv::bitwise_not(l_mask1, l_mask2);
    cv::Mat l_output;

    // Segmenting the cloth out of the frame using bitwise and with the inverted mask
    bitwise_and(m_cvPtr->image, m_cvPtr->image, l_output, l_mask1);

    // Update GUI Window
    cv::imshow("Original Image", m_cvPtr->image);
    cv::imshow("Mask Image", l_output);
    cv::waitKey(3);
}

/**
 * Compute velocity commands to
 * be sent to the robot in order
 * to reach the goal.
 */
void Navigation::dwaTrajectoryControl(const hsr_navigation::ClutterPlannerService &p_service)
{
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
            if (m_debug)
            {
                ROS_ERROR("DWA velocities computation failed.");
            }
        }

        // Send commands
        if (m_debug)
        {
            //ROS_INFO_STREAM(l_cmd_vel);
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
        ROS_INFO("Quitting DWA control for replanning...");

        // Reset replan flag
        m_replan = false;

        // Request new plan
        requestClutterPlan(false);
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
