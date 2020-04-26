#ifndef NAVIGATION_HPP_
#define NAVIGATION_HPP_

// General imports
#include <mutex>
#include <random>
#include <math.h>
#include <iostream>
#include <boost/thread/thread.hpp>

// OpenCV imports
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// ROS msg/srv
#include <nav_msgs/Path.h>
#include <std_msgs/Header.h>
#include <nav_msgs/GetMap.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <hsr_navigation/CellMessage.h>
#include <hsr_navigation/ObjectMessage.h>
#include <hsr_navigation/ClutterPlannerService.h>
#include <hsr_navigation/ClutterPlannerServiceReq.h>

// ROS general
#include "ros/ros.h"
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <message_filters/subscriber.h>
#include <tf2_ros/transform_listener.h>
#include <message_filters/time_synchronizer.h>
#include <dwa_local_planner/dwa_planner_ros.h>
#include <image_geometry/PinholeCameraModel.h>

// Config file with paramters
#include "navigation_params.hpp"

class Navigation
{
public:
    // Contructor (explicit)
    explicit Navigation(tf2_ros::Buffer&, tf2_ros::TransformListener&);

    // Destructor (virtual)
    virtual ~Navigation();

private:
    /**
     * Class methods
     */

    // ROS services
    void loadStaticMap();
    void requestClutterPlan(const bool &);

    // ROS callbacks
    void checkGlobalPath(const nav_msgs::OccupancyGrid);
    void perceptionCallback(const sensor_msgs::ImageConstPtr&, 
                            const sensor_msgs::ImageConstPtr&, 
                            const sensor_msgs::CameraInfoConstPtr&);

    // General
    void initialize();
    void dwaTrajectoryControl(const hsr_navigation::ClutterPlannerService &);
    void populatePlannerRequest(hsr_navigation::ClutterPlannerService &);
    void updateObjectMessage(const cv_bridge::CvImagePtr&, const std::vector<cv::Point2d>&);

    /**
     * Class members
     */

    // General members
    std::mutex m_mtx;
    bool m_debug = true;
    bool m_replan = false;
    bool m_action = false;
    bool m_modelInitialised = false;
    std::default_random_engine m_re;
    std::uniform_real_distribution<float> m_x_unif{1, 5.5f};
    std::uniform_real_distribution<float> m_y_unif{-0.23f, 0.9f};

    // OpenCV members
    cv_bridge::CvImagePtr m_rgbPtr;
    cv_bridge::CvImagePtr m_depthPtr;
    image_geometry::PinholeCameraModel m_phcm;

    // ROS members
    ros::NodeHandle m_nh;
    ros::Publisher m_velPub;
    ros::Publisher m_srvPub;
    ros::Subscriber m_costSub;
    tf2_ros::Buffer &m_buffer;
    tf2_ros::TransformListener &m_tf;
    nav_msgs::OccupancyGrid m_updatedMap;
    dwa_local_planner::DWAPlannerROS m_dp;
    nav_msgs::OccupancyGrid m_occupacyGrid;
    costmap_2d::Costmap2D *m_globalCostmap = nullptr;
    std::vector<geometry_msgs::PoseStamped> m_globalPath;
    costmap_2d::Costmap2DROS* m_localCostmapROS = nullptr;
    costmap_2d::Costmap2DROS* m_globalCostmapROS = nullptr;
    std::vector<hsr_navigation::ObjectMessage> m_objects{0};
};

#endif // NAVIGATION_HPP_
