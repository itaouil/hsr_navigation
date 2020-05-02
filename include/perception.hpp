#ifndef PERCEPTION_HPP_
#define PERCEPTION_HPP_

// General imports
#include <mutex>
#include <math.h>
#include <iostream>
#include <boost/thread/thread.hpp>

// OpenCV imports
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// ROS msg/srv
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <hsr_navigation/CellMessage.h>
#include <hsr_navigation/ObjectMessage.h>

// ROS general
#include "ros/ros.h"
#include <costmap_2d/costmap_2d.h>
#include <message_filters/subscriber.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/time_synchronizer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/sync_policies/approximate_time.h>

// Perception parameters
#include "parameters.hpp"

// Abbreviation
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
typedef message_filters::Synchronizer<MySyncPolicy> Sync;

class Perception
{
public:
    // Contructor (explicit)
    explicit Perception();

    // Destructor (virtual)
    virtual ~Perception();

    /**
     * Class methods
     */
    bool initialized();
    std::vector<hsr_navigation::ObjectMessage> getObstacles(costmap_2d::Costmap2D*);

private:
    /**
     * Class methods
     */

    void initialize();

    void setCameraInfo(sensor_msgs::CameraInfo);

    void setRGBD(const sensor_msgs::ImageConstPtr&, const sensor_msgs::ImageConstPtr&);

    void populateObjectMessage(costmap_2d::Costmap2D*,
                               const std::vector<cv::Point2d>&,
                               std::vector<hsr_navigation::ObjectMessage>&);

    /**
     * Class members
     */

    // General members
    std::mutex m_mtx;
    bool m_debug = true;
    bool m_initialized = false;
    bool m_modelInitialized = false;

    // OpenCV members
    cv_bridge::CvImagePtr m_rgbPtr;
    cv_bridge::CvImagePtr m_depthPtr;
    image_geometry::PinholeCameraModel m_phcm;

    // ROS members
    ros::NodeHandle m_nh;
    ros::Subscriber m_camInfo;
    message_filters::Subscriber<sensor_msgs::Image> m_rgbSub;
    message_filters::Subscriber<sensor_msgs::Image> m_depthSub;
    boost::shared_ptr<Sync> m_sync;
};

#endif // PERCEPTION_HPP_
