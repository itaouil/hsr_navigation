// Header files
#include "perception.hpp"

/**
 * Default constructor.
 */
Perception::Perception()
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
    // Camera info subscriber
    m_camInfo = m_nh.subscribe<sensor_msgs::CameraInfo>(CAMERA_INFO, 
                                                        1,
                                                        &Perception::setCameraInfo,
                                                        this);

    // Synchronize rgb and depth data
    message_filters::Subscriber<sensor_msgs::Image> rgbSub(m_nh, RGB_DATA, 1);
    message_filters::Subscriber<sensor_msgs::Image> depthSub(m_nh, DEPTH_DATA, 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image, 
                                      sensor_msgs::Image> sync(rgbSub, 
                                                               depthSub,
                                                               10);
    sync.registerCallback(boost::bind(&Perception::setRGBD,
                                      this,
                                      _1,
                                      _2));
}

/**
 * Sets the camera model.
 */
void Perception::setCameraInfo(const sensor_msgs::CameraInfoConstPtr& p_camInfo)
{
    // Initialise camera model
    if (!m_modelInitialized)
    {
        m_phcm.fromCameraInfo(p_camInfo);
        m_modelInitialized = true;

        if (DEBUG)
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

            if (DEBUG)
            {
                ROS_INFO("RGB-D data set correctly.");
            }
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }    
}

/**
 * Main function that handles
 * the logic to create the obstacle
 * message for the new plan
 */
std::vector<hsr_navigation::ObjectMessage> Perception::getObstacles(costmap_2d::Costmap2D *p_gcm,
                                                                    tf2_ros::TransformListener &p_tf)
{
    // Object messag holder
    std::vector<hsr_navigation::ObjectMessage> l_objects{0};

    // Convert the RGB image
    // to an HSV color space
    cv::Mat l_hsv;
    cv::cvtColor(m_rgbPtr->image, l_hsv, cv::COLOR_BGR2HSV);

    // Define red color mask
    cv::Mat l_mask1;
    cv::Mat l_mask2;
    cv::inRange(l_hsv, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), l_mask1);
    cv::inRange(l_hsv, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), l_mask2);
    cv::Mat l_mask = l_mask1 + l_mask2;

    // Get red pixel location in the matrix
    std::vector<cv::Point2d> l_locations;
    cv::findNonZero(l_mask1, l_locations);

    if (l_locations.size())
    {
        if (DEBUG)
        {
            ROS_INFO("Red pixels detected.");
        }

        populateObjectMessage(l_objects);
    }
    
    return l_objects
}

/**
 * Create ObjectMessage instance
 * to be used for re-planning.
 */
void Navigation::populateObjectMessage(costmap_2d::Costmap2D *p_gcm,
                                       tf2_ros::TransformListener &p_tf,
                                       const std::vector<cv::Point2d> &p_locations,
                                       std::vector<hsr_navigation::ObjectMessage> &p_objs)
{
    // Convert 2d pixels in 3d points
    std::vector<geometry_msgs::PointStamped> l_3dPoints;
    for (auto l_point: p_locations)
    {
        // Access depth value
        const double l_depth = m_depthPtr->image.at<float>(l_point.y, l_point.x);

        // Check if depth is valid
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

    // RGB-D to map frame
    std::vector<hsr_navigation::CellMessage> l_cellMessages;
    for (auto l_3dPointRGBFrame: l_3dPoints)
    {
        try
        {
            // Map coordinates
            int l_mx;
            int l_my;

            // RGBD to map
            geometry_msgs::PointStamped l_3dPointMapFrame;
            m_tf.transformPoint("map", l_3dPointRGBFrame, l_3dPointMapFrame);

            // world to map conversion
            m_globalCostmap->worldToMapEnforceBounds(l_3dPointMapFrame.point.x, 
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

    // Create object message
    hsr_navigation::ObjectMessage l_obj;
    l_obj.uid = 1
    l_obj.object_class = 1;
    l_obj.center_cell.mx = l_mx;
    l_obj.center_cell.my = l_my;
    l_obj.center_wx = l_mean.x;
    l_obj.center_wy = l_mean.y;
    l_obj.cell_vector = l_cellMessages;

    if (DEBUG)
    {
        ROS_INFO(l_obj);
    }

    // Update object message vector
    p_objs.push_back(l_obj);
}