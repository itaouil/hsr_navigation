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
    // // Make head look down
    if (!m_firstTime)
    {
        lookDown();
    }

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

    // Create service client
    ros::ServiceClient l_client;
    l_client = m_nh.serviceClient<hsr_navigation::DetectionService>("detection_service");

    // Populate request
    hsr_navigation::DetectionService l_service;
    l_service.request.image = *(cv_bridge::CvImage(std_msgs::Header(), "bgr8", m_rgbPtr->image).toImageMsg());

    // Call service
    if (l_client.call(l_service))
    { 
        // Store labels and points
        m_labels = l_service.response.labels.data;
        m_points2d = l_service.response.points.data;

        if (DEBUGPERCEPTION)
        {
            ROS_INFO("Detection service called successfully");
        }
    }
    else
    {
        if (DEBUGPERCEPTION)
        {
            ROS_ERROR("Failed to call detection service...");
        }     
    }

    // Process object pixels (received from detection module)
    for (int x = 0; x < m_labels.size(); x++)
    {
        // Initialize action
        int l_action = -1;

        // Choose action based on label
        if (m_labels[x] == "grasp")
        {
            l_action = 5;
        }
        else if (m_labels[x] == "kick")
        {
            l_action = 1;
        }
        else
        {
            l_action = 3;
        }

        std::cout << "Now processing: " << m_labels[x] << std::endl;
        std::cout << "Number of pixels to it: " << m_points2d[x].data.size() << std::endl;

        // Process object pixels
        populateObjectMessage(l_action, p_gcm, m_points2d[x].data, l_objects);
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
                                       const std::vector<hsr_navigation::Point> &p_locations,
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
            cv::Point2d l_2dPoint(l_point.x, l_point.y);
            cv::Point3d l_3dPoint = m_phcm.projectPixelTo3dRay(l_2dPoint);
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

            // World coord. to map coord. conversion
            int l_mx;
            int l_my;
            p_gcm->worldToMapEnforceBounds(l_3dPointMapFrame.point.x, 
                                           l_3dPointMapFrame.point.y, 
                                           l_mx, 
                                           l_my);

            // Populate vector with new
            // cell only if not already
            // present in the geometry set
            // in order to avoid planner issues
            // due to overlapping cells
            std::pair<int, int> l_cellPair = std::make_pair(l_mx, l_my);
            if (!(m_geometry.find(l_cellPair) != m_geometry.end()))
            {
                // Add new element to set
                m_geometry.insert(l_cellPair);

                // Create cell message
                hsr_navigation::CellMessage l_cellMessage;
                l_cellMessage.mx = l_mx;
                l_cellMessage.my = l_my;

                // Populate cell message vector
                l_cellMessages.push_back(l_cellMessage);

                // Aggregate means
                l_meanX += l_3dPointMapFrame.point.x;
                l_meanY += l_3dPointMapFrame.point.y;
            }
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
    std::cout << "Size: " << l_cellMessages.size();
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
    l_traj.points[0].positions[1] = -0.5;
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

    // // Wait for head trajectory
    // // movement to be finished
    // // with a timeour of 2 seconds
    // ros::Time start_time = ros::Time::now();
    // ros::Duration timeout(3.0);
    // while(ros::Time::now() - start_time < timeout) {
    //     ros::spinOnce();
    // }
}