#ifndef PERCEPTION_HPP_
#define PERCEPTION_HPP_

// General imports
#include <set>
#include <mutex>
#include <math.h>
#include <utility>
#include <iostream>
#include <boost/thread/thread.hpp>

// OpenCV imports
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// ROS msg/srv
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <hsr_navigation/Point.h>
#include <hsr_navigation/Points.h>
#include <hsr_navigation/Labels.h>
#include <sensor_msgs/CameraInfo.h>
#include <hsr_navigation/Points2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <hsr_navigation/CellMessage.h>
#include <hsr_navigation/ObjectMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <hsr_navigation/DetectionService.h>

// HSR imports
#include <trajectory_msgs/JointTrajectory.h>
#include <controller_manager_msgs/ControllerState.h>
#include <controller_manager_msgs/ListControllers.h>

// ROS general
#include "ros/ros.h"
#include <costmap_2d/costmap_2d.h>
#include <message_filters/subscriber.h>
#include <tf2_ros/transform_listener.h>
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
    explicit Perception(tf2_ros::Buffer&, tf2_ros::TransformListener&);

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

    void lookDown();
    void initialize();
    void setCameraInfo(sensor_msgs::CameraInfo);
    bool pointWithinOffset(const unsigned int, const geometry_msgs::PointStamped&);
    void setRGBD(const sensor_msgs::ImageConstPtr&, const sensor_msgs::ImageConstPtr&);
    void transformPoint(const std::string&, 
                        geometry_msgs::PointStamped&, 
                        const geometry_msgs::PointStamped&);
    void populateObjectMessage(const unsigned int,
                               costmap_2d::Costmap2D*,
                               const std::vector<hsr_navigation::Point>&,
                               std::vector<hsr_navigation::ObjectMessage>&);

    /**
     * Class members
     */

    // General members
    std::mutex m_mtx;
    unsigned int m_uid = 1;
    bool m_firstTime = true;
    bool m_initialized = false;
    bool m_modelInitialized = false;
    std::vector<std::string> m_labels;
    std::set<std::pair<int, int>> m_geometry;
    std::vector<hsr_navigation::Points> m_points2d;

    // OpenCV members
    cv_bridge::CvImagePtr m_rgbPtr;
    cv_bridge::CvImagePtr m_depthPtr;
    image_geometry::PinholeCameraModel m_phcm;

    // ROS members
    ros::NodeHandle m_nh;
    ros::Publisher m_trajPub;
    ros::Subscriber m_camInfo;
    tf2_ros::Buffer &m_buffer;
    boost::shared_ptr<Sync> m_sync;
    tf2_ros::TransformListener &m_tf;
    message_filters::Subscriber<sensor_msgs::Image> m_rgbSub;
    message_filters::Subscriber<sensor_msgs::Image> m_depthSub;
};

#endif // PERCEPTION_HPP_
