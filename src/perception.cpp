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
                //ROS_INFO("RGB-D data set correctly.");
            }
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
    if (DEBUG)
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

    if (DEBUG)
    {
        ROS_INFO("Perception: converted BGR to HSV");
    }

    // Define red color mask
    cv::Mat l_mask1;
    cv::Mat l_mask2;
    cv::inRange(l_hsv, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), l_mask1);
    cv::inRange(l_hsv, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), l_mask2);
    cv::Mat l_mask = l_mask1 + l_mask2;

    //cv::imshow("Mask", l_mask1);
    //cv::waitKey(0);

    if (DEBUG)
    {
        ROS_INFO("Perception: created final mask");
    }

    // Get red pixel location in the matrix
    std::vector<cv::Point> l_locations;
    cv::findNonZero(l_mask, l_locations);

    if (DEBUG)
    {
        ROS_INFO("Perception: found pixel locations");
    }

    if (l_locations.size())
    {
        if (DEBUG)
        {
            ROS_INFO("Red pixels detected.");
        }

        populateObjectMessage(p_gcm, l_locations, l_objects);
    }
    
    if (DEBUG)
    {
        ROS_INFO("Perception: Obstacles computed correctly");
    }

    return l_objects;
}

/**
 * Create ObjectMessage instance
 * to be used for re-planning.
 */
void Perception::populateObjectMessage(costmap_2d::Costmap2D *p_gcm,
                                       const std::vector<cv::Point> &p_locations,
                                       std::vector<hsr_navigation::ObjectMessage> &p_objs)
{
    if (DEBUG)
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

    if (DEBUG)
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

    if (DEBUG)
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
    l_obj.uid = 1;
    l_obj.object_class = 1;
    l_obj.center_cell.mx = l_mx;
    l_obj.center_cell.my = l_my;
    l_obj.center_wx = l_meanX;
    l_obj.center_wy = l_meanY;
    l_obj.cell_vector = l_cellMessages;

    if (DEBUG)
    {
        ROS_INFO("Perception: computed object message.");
        std::cout<< "Mean mx: " << l_mx << std::endl;
        std::cout<< "Mean my: " << l_my << std::endl;
        std::cout<< "Mean wx: " << l_meanX << std::endl;
        std::cout<< "Mean wy: " << l_meanY << std::endl;
    }

    // Update object message vector
    p_objs.push_back(l_obj);
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
