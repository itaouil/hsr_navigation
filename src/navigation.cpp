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
    // Initialize costmaps (global and local)
    m_localCostmapROS = new costmap_2d::Costmap2DROS("local_costmap", m_buffer);
    m_globalCostmapROS = new costmap_2d::Costmap2DROS("global_costmap", m_buffer);

    // Initialize dwa local planner
    m_dp.initialize("hsr_dwa_planner", &m_buffer, m_localCostmapROS);

    // DWA planner velocity publisher
    m_velPub = m_nh.advertise<geometry_msgs::Twist>(DWA_PUB_TOPIC, 1);

    // Clutter planner request publisher
    m_srvPub = m_nh.advertise<hsr_navigation::ClutterPlannerServiceReq>(CP_PUB_TOPIC, 1);

    // Subscriber to global costmap
    m_costSub = m_nh.subscribe<nav_msgs::OccupancyGrid>(GC_SUB_TOPIC, 
                                   1,
                                   &Navigation::checkGlobalPath,
                                   this);

    // Synchronize RGB and Depth
    message_filters::Subscriber<sensor_msgs::Image> rgbSub(m_nh, RGB_SUB_TOPIC, 1);
    message_filters::Subscriber<sensor_msgs::Image> depthSub(m_nh, DEPTH_SUB_TOPIC, 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> camInfo(m_nh, MODEL_SUB_TOPIC, 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image, 
                                      sensor_msgs::Image, 
                                      sensor_msgs::CamerInfo> sync(rgbSub, 
                                                                   depthSub,
                                                                   camInfo,
                                                                   10);
    sync.registerCallback(boost::bind(&Navigation::perceptionCallback,
                                      this,
                                      _1,
                                      _2,
                                      -3));
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
            // Map coordinates
            int l_mx;
            int l_my;

            // Robot global pose
            geometry_msgs::PoseStamped l_global_pose;
            m_globalCostmapROS->getRobotPose(l_global_pose);

            // World coordinates
            double l_wx = l_global_pose.pose.position.x;
            double l_wy = l_global_pose.pose.position.y;

            // Cast from world to map
            m_globalCostmap->worldToMapEnforceBounds(l_wx, l_wy, l_mx, l_my);

            // Get cost (convert to int from unsigned char)
            int l_cellCost = (int) m_globalCostmap->getCost(l_mx, l_my);

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

void Navigation::perceptionCallback(const sensor_msgs::ImageConstPtr& p_rgb, 
                                    const sensor_msgs::ImageConstPtr& p_depth
                                    const sensor_msgs::CameraInfoConstPtr& p_camInfo)
{
    // Initialise camera model
    if (!m_modelInitialised)
    {
        m_phcm.fromCameraInfo(p_camInfo);
        m_modelInitialised = true;
    }

    // Perform computations only
    // if obstacle was detected
    // with laser scans
    if (m_replan)
    {
        // Convert ROS image to OpenCV Mat
        try
        {
            m_rgbPtr = cv_bridge::toCvCopy(p_rgb, sensor_msgs::image_encodings::BGR8);
            m_depthPtr = cv_bridge::toCvCopy(p_depth, sensor_msgs::image_encodings::TYPE_32FC1);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }

        // Convert BGR to HSV
        cv::Mat l_hsv;
        cv::cvtColor(m_rgbPtr->image, l_hsv, cv::COLOR_BGR2HSV);

        // Red color masks
        cv::Mat l_mask1;
        cv::Mat l_mask2;
        cv::inRange(l_hsv, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), l_mask1);
        cv::inRange(l_hsv, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), l_mask2);

        // Generate final mask
        l_mask = l_mask1 + l_mask2;

        // Get locations of red pixels 
        std::vector<cv::Point2d> l_locations;
        cv::findNonZero(l_mask1, l_locations);

        // Populate object message
        hsr_navigation::ObjectMessage l_objMsg;
        populateObjectMessage(l_objMsg, l_locations, m_depthPtr);
    }
}

/**
 * Create ObjectMessage instance
 * to be sent to the clutter planner
 * for re-planning.
 */
void Navigation::populateObjectMessage(hsr_navigation::ObjectMessage &p_objMsg,
                                       const std::vector<cv::Point2d> &p_locations,
                                       const cv_bridge::CvImagePtr &p_depthImage)
{
    // Convert 2d pixels in 3d points
    std::vector<geometry_msgs::PointStamped> l_3dPoints;
    for (auto l_point: p_locations)
    {   
        // Access depth value
        const double l_depth = p_depthImage->image.at<float>(l_point.y, l_point.x);

        // Check if depth is valide and not NaN
        if (!isnan(l_depth))
        {
            std::cout<< "Depth value is: " << l_depth << std::endl;
            
            // Ray point with correct depth
            cv::Point3d l_3dPoint = m_phcm.projectPixelTo3dRay(l_point);
            l_3dPoint.z = l_depth;

            // Create point stamped object
            geometry_msgs::PointStamped l_pointStamped;
            l_pointStamped.header.stamp = ros::Time::now();
            l_pointStamped.header.frame_id = FRAME_ID;
            l_pointStamped.point.x = l_3dPoint.x;
            l_pointStamped.point.y = l_3dPoint.y;
            l_pointStamped.point.z = l_3dPoint.z;

            // Populate vector
            l_3dPoints.push_back(l_pointStamped);
        }
        else
        {
            std::cout<< "Depth value is NaN..." << std::endl;
        }
    }

    // RGB-D frame to map frame
    std::vector<hsr_navigation::CellMessage> l_cellMessages;
    for (auto l_stampedIn: l_3dPoints)
    {
        try
        {
            // Map coordinates
            int l_mx;
            int l_my;

            // RGBD to map
            geometry_msgs::PointStamped l_stampedOut;
            m_tf.transformPoint("map", l_stampedIn, l_stampedOut);

            // world to map conversion
            l_wx = l_stampedOut.point.x;
            l_wy = l_stampedOut.point.y;
            m_globalCostmap->worldToMapEnforceBounds(l_wx, l_wy, l_mx, l_my);

            // Populate cell message vector
            hsr_navigation::CellMessage l_cellMessage;
            l_cellMessage.mx = l_mx;
            l_cellMessage.my = l_my;

            // Populate vector
            l_cellMessages.push_back(l_cellMessage);
        }
        catch (tf::TransformException &e)
        {
            ROS_ERROR("%s", e.what());
            ros::Duration(1.0).sleep();
            continue;
        }
    }

    // Compute mean 2d point
    cv::Mat l_mean_;
    cv::reduce(p_locations, l_mean_, 01, CV_REDUCE_AVG);
    cv::Point2f l_mean(l_mean_.at<float>(0,0), l_mean_.at<float>(0,1));

    // Compute mean 2d point map space
    int l_mx;
    int l_my;
    m_globalCostmap->worldToMapEnforceBounds(l_mean.x, l_mean.y, l_mx, l_my);

    // Populage object message
    p_objMsg.uid = 1
    p_objMsg.object_class = 1;
    p_objMsg.center_cell.mx = l_mx;
    p_objMsg.center_cell.my = l_my;
    p_objMsg.center_wx = (int) l_mean.x;
    p_objMsg.center_wy = (int) l_mean.y;
    p_objMsg.cell_vector = l_cellMessages;

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
        ROS_INFO("GOAL REACHED :)...");
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
