// Header files
#include "perception.hpp"

// Namespaces
using namespace sensor_msgs;
using namespace message_filters;

typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;

/**
 * Default constructor.
 */
Perception::Perception(tf2_ros::Buffer &p_buffer, tf2_ros::TransformListener &p_tf):
    m_buffer(p_buffer), m_tf(p_tf)
{
    // Initialize members
    initialize();
}

/**
 * Destructor.
 */
Perception::~Perception()
{
}

/**
 * Initialize members
 */
void Perception::initialize()
{
    // Head trajectory publisher
    m_trajPub = m_nh.advertise<trajectory_msgs::JointTrajectory>(HEAD_CONTROL, 10);

    // Camera info subscriber
    m_camInfo = m_nh.subscribe<sensor_msgs::CameraInfo>(CAMERA_INFO, 
                                                       1,
                                                       &Perception::setCameraInfo,
                                                       this);

    // Synchronize rgb and depth data
    m_rgbSub.subscribe(m_nh, RGB_DATA, 1);
    m_depthSub.subscribe(m_nh, DEPTH_DATA, 1);
    m_sync.reset(new Sync(MySyncPolicy(5), m_rgbSub, m_depthSub));
    m_sync->registerCallback(boost::bind(&Perception::setRGBD, this, _1, _2));
}

/**
 * Sets the camera model.
 */
void Perception::setCameraInfo(sensor_msgs::CameraInfo p_camInfo)
{
    // Initialise camera model
    if (!m_modelInitialized)
    {
        m_phcm.fromCameraInfo(p_camInfo);
        m_modelInitialized = true;

        if (DEBUGPERCEPTION)
        {
            ROS_INFO("Camera model set correctly.");
        }
    }
}

/**
 * Sets RGB-D data for a later usage.
 */
void Perception::setRGBD(const sensor_msgs::ImageConstPtr& p_rgb, 
                         const sensor_msgs::ImageConstPtr& p_depth)
{
    if (m_modelInitialized)
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

        // Confirm initialization
        m_initialized = true;
    }    
}

/**
 * Main function that handles
 * the logic to create the obstacle
 * message for the new plan
 */
std::vector<hsr_navigation::ObjectMessage> Perception::getObstacles(costmap_2d::Costmap2D *p_gcm)
{
    // Make head look down
    if (!m_firstTime)
    {
        lookDown();
    }

    // Sleep for two seconds to
    // avoid problems with illumination
    // changes while head is moving
    ros::Duration(2).sleep();

    if (DEBUGPERCEPTION)
    {
        ROS_INFO("Perception: getObstacles called from Navigation");
    }

    // Object messag holder
    std::vector<hsr_navigation::ObjectMessage> l_objects{0};

    // If perception not yet
    // initialized return empty
    // array
    if (!m_initialized)
    {
        return l_objects;
    }

    // Convert the RGB image
    // to an HSV color space
    cv::Mat l_hsv;
    cv::cvtColor(m_rgbPtr->image, l_hsv, cv::COLOR_BGR2HSV);

    if (DEBUGPERCEPTION)
    {
        ROS_INFO("Perception: converted BGR to HSV");
    }

    // Define push action mask
    cv::Mat l_pushMask;
    cv::Mat l_pushHSV = l_hsv.clone();
    cv::inRange(l_pushHSV, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 0), l_pushMask);

    // Define grasp action mask
    cv::Mat l_graspMask;
    cv::Mat l_graspHSV = l_hsv.clone();
    cv::inRange(l_graspHSV, cv::Scalar(23,168,0), cv::Scalar(29,193,255), l_graspMask);

    // Define grasp action mask 2
    cv::Mat l_graspMask2;
    cv::Mat l_graspHSV2 = l_hsv.clone();
    cv::inRange(l_graspHSV2, cv::Scalar(0,255,0), cv::Scalar(0,255,255), l_graspMask2);

    // Define kick action mask
    cv::Mat l_kickMask;
    cv::Mat l_kickHSV = l_hsv.clone();
    cv::inRange(l_kickHSV, cv::Scalar(0, 0, 130), cv::Scalar(180, 0, 255), l_kickMask);

    cv::imshow("Push Mask", l_pushMask);
    cv::imshow("Grasp Mask", l_graspMask);
    cv::imshow("Grasp Mask 2", l_graspMask2);
    cv::imshow("Kick Mask", l_kickMask);

    cv::waitKey(0);

    // // Get pushable object pixel locations
    std::vector<cv::Point> l_pushPixelsLocation;
    cv::findNonZero(l_pushMask, l_pushPixelsLocation);

    // Get graspable object pixel locations
    std::vector<cv::Point> l_graspPixelsLocation;
    cv::findNonZero(l_graspMask, l_graspPixelsLocation);

    // Get graspable object2 pixel locations
    std::vector<cv::Point> l_graspPixelsLocation2;
    cv::findNonZero(l_graspMask2, l_graspPixelsLocation2);

    // Get kick object pixel locations
    std::vector<cv::Point> l_kickPixelsLocation;
    cv::findNonZero(l_kickMask, l_kickPixelsLocation);

    // // Populate push object
    std::cout << "Number of push pixels: " << l_pushPixelsLocation.size() << std::endl;
    if (l_pushPixelsLocation.size() > 3500)
    {
        if (DEBUGPERCEPTION)
        {
            ROS_INFO("Push pixels detected.");
        }

        populateObjectMessage(3, p_gcm, l_pushPixelsLocation, l_objects);
    }

    // Populate grasp object
    std::cout << "Number of grasp pixels: " << l_graspPixelsLocation.size() << std::endl;
    if (l_graspPixelsLocation.size() > 400 && l_graspPixelsLocation.size() < 4600)
    {
        if (DEBUGPERCEPTION)
        {
            ROS_INFO("Grasp pixels detected.");
        }

        populateObjectMessage(5, p_gcm, l_graspPixelsLocation, l_objects);
    }

    // Populate grasp2 object
    std::cout << "Number of grasp2 pixels: " << l_graspPixelsLocation2.size() << std::endl;
    if (l_graspPixelsLocation2.size() > 400 && l_graspPixelsLocation2.size() < 4600)
    {
        if (DEBUGPERCEPTION)
        {
            ROS_INFO("Grasp2 pixels detected.");
        }

        populateObjectMessage(5, p_gcm, l_graspPixelsLocation2, l_objects);
    }

    // Populate kick object
    std::cout << "Number of kick pixels: " << l_kickPixelsLocation.size() << std::endl;
    if (l_kickPixelsLocation.size() > 300 && l_kickPixelsLocation.size() < 4800)
    {
        if (DEBUGPERCEPTION)
        {
            ROS_INFO("Kick pixels detected.");
        }

        populateObjectMessage(1, p_gcm, l_kickPixelsLocation, l_objects);
    }

    // Reset m_uid
    m_uid = 1;

    // Set flag
    m_firstTime = false;

    return l_objects;
}

/**
 * Create ObjectMessage instance
 * to be used for re-planning.
 */
void Perception::populateObjectMessage(const unsigned int p_action,
                                       costmap_2d::Costmap2D *p_gcm,
                                       const std::vector<cv::Point> &p_locations,
                                       std::vector<hsr_navigation::ObjectMessage> &p_objs)
{
    if (DEBUGPERCEPTION)
    {
        ROS_INFO("Perception: populate object message called.");
    }

    // Convert 2d pixels in 3d points
    std::vector<geometry_msgs::PointStamped> l_3dPoints;
    for (auto l_point: p_locations)
    {
        // Access depth value
        double l_depth = m_depthPtr->image.at<float>(l_point.y, l_point.x);

        // Check if depth is valid
        if (!isnan(l_depth))
        {   
            // Convert depth to meters
            l_depth *= 0.001;

            // Compute world coordinates for
            // every pixel location in 2D
            cv::Point3d l_3dPoint = m_phcm.projectPixelTo3dRay(l_point);
            l_3dPoint *= l_depth;

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

    if (DEBUGPERCEPTION)
    {
        ROS_INFO("Perception: computed 3D points.");
    }

    // Overal mean centre point
    double l_meanX = 0.0;
    double l_meanY = 0.0;
    
    std::vector<hsr_navigation::CellMessage> l_cellMessages;

    // RGB-D to map frame
    for (auto l_3dPointRGBDFrame: l_3dPoints)
    {
        try
        {            
            // RGB-D to map transformation
            geometry_msgs::PointStamped l_3dPointMapFrame;
            transformPoint(FRAME_ID, l_3dPointMapFrame, l_3dPointRGBDFrame);

            // Aggregate means
            l_meanX += l_3dPointMapFrame.point.x;
            l_meanY += l_3dPointMapFrame.point.y;

            // World coord. to map coord. conversion
            int l_mx;
            int l_my;
            p_gcm->worldToMapEnforceBounds(l_3dPointMapFrame.point.x, 
                                           l_3dPointMapFrame.point.y, 
                                           l_mx, 
                                           l_my);

            // Populate cell message vector
            hsr_navigation::CellMessage l_cellMessage;
            l_cellMessage.mx = l_mx;
            l_cellMessage.my = l_my;

            // Populate vector
            l_cellMessages.push_back(l_cellMessage);
        }
        catch (tf2::TransformException &e)
        {
            ROS_ERROR("%s", e.what());
            ros::Duration(1.0).sleep();
            continue;
        }
    }

    if (DEBUGPERCEPTION)
    {
        ROS_INFO("Perception: computed cell messages.");
    }

    // Compute final mean point
    l_meanX /= l_cellMessages.size();
    l_meanY /= l_cellMessages.size();

    // Get map coordinates of the mean
    int l_mx;
    int l_my;
    p_gcm->worldToMapEnforceBounds(l_meanX, 
                                   l_meanY, 
                                   l_mx, 
                                   l_my);

    // Create object message
    hsr_navigation::ObjectMessage l_obj;
    l_obj.uid = m_uid;
    l_obj.object_class = p_action;
    l_obj.center_cell.mx = l_mx;
    l_obj.center_cell.my = l_my;
    l_obj.center_wx = l_meanX;
    l_obj.center_wy = l_meanY;
    l_obj.cell_vector = l_cellMessages;

    if (DEBUGPERCEPTION)
    {
        ROS_INFO("Perception: computed object message.");
        std::cout<< "Mean mx: " << l_mx << std::endl;
        std::cout<< "Mean my: " << l_my << std::endl;
        std::cout<< "Mean wx: " << l_meanX << std::endl;
        std::cout<< "Mean wy: " << l_meanY << std::endl;
    }

    // Update object message vector
    p_objs.push_back(l_obj);

    // Increase m_uid
    m_uid += 1;
}

/**
 * Confirms that the perception
 * was initialized correctly
 */
bool Perception::initialized()
{
    return m_initialized;
}

/**
 * Transforms a point from a
 * given frame ID to map frame
 * using transform matrices
 */
void Perception::transformPoint(const std::string &p_frameID,
                                geometry_msgs::PointStamped &p_output,
                                const geometry_msgs::PointStamped &p_input)
{
    // Create transformer
    geometry_msgs::TransformStamped l_transformer;

    // Set transformer
    l_transformer = m_buffer.lookupTransform("map", 
                                             p_frameID,
                                             ros::Time(0),
                                             ros::Duration(1.0));

    // Perform transformation
    tf2::doTransform(p_input, p_output, l_transformer);
}

/**
 * Sends commands to the head to
 * look down in order to check for
 * obstacles in the path.
 */
void Perception::lookDown()
{
    // Wait for publisher connection
    while (m_trajPub.getNumSubscribers() == 0) {
        ros::Duration(0.1).sleep();
    }

    // Flags
    bool l_running = false;

    // Make sure controller is running
    ros::ServiceClient l_client = m_nh.serviceClient<controller_manager_msgs::ListControllers>(CONTROLLER);
    
    // Initialize controllers holder
    controller_manager_msgs::ListControllers m_listControllers;

    while (l_running == false) {
        ros::Duration(0.1).sleep();
        if (l_client.call(m_listControllers)) {
            for (unsigned int i = 0; i < m_listControllers.response.controller.size(); i++) {
                controller_manager_msgs::ControllerState c = m_listControllers.response.controller[i];
                if (c.name == "head_trajectory_controller" && c.state == "running") {
                    l_running = true;
                }
            }
        }
    }

    // Fill ROS message
    trajectory_msgs::JointTrajectory l_traj;
    l_traj.joint_names.push_back("head_pan_joint");
    l_traj.joint_names.push_back("head_tilt_joint");
    l_traj.points.resize(1);
    l_traj.points[0].positions.resize(2);
    l_traj.points[0].positions[0] = 0.0;
    l_traj.points[0].positions[1] = -0.7;
    l_traj.points[0].velocities.resize(2);
    for (size_t i = 0; i < 2; ++i) {
        l_traj.points[0].velocities[i] = 0.0;
    }
    l_traj.points[0].time_from_start = ros::Duration(3.0);

    // Publish ROS message
    m_trajPub.publish(l_traj);

    ros::spinOnce();
    ros::Duration(3.0).sleep();

    if (DEBUGPERCEPTION)
    {
        ROS_INFO("Perception: head control trajectory sent.");
    }
}
