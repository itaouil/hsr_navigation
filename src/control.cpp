// Header files
#include "control.hpp"

/**
 * Default constructor.
 */
Control::Control(tf2_ros::Buffer &p_buf, 
                 tf2_ros::TransformListener &p_tf,
                 costmap_2d::Costmap2DROS* p_lc, 
                 costmap_2d::Costmap2DROS* p_gc):
    m_buffer(p_buf), m_tf(p_tf), m_localCostmapROS(p_lc), m_globalCostmapROS(p_gc)
{
    // Initialize members
    initialize();
}

/**
 * Destructor.
 */
Control::~Control()
{
}

/**
 * Initialize members
 */
void Control::initialize()
{
    // DWA planner velocity publisher
    m_velPub = m_nh.advertise<geometry_msgs::Twist>(DWA_VELOCITIES, 1);

    // Subscriber to global costmap
    m_odomSub = m_nh.subscribe<nav_msgs::Odometry>(ODOMETRY, 
                                                   1,
                                                   &Control::setOdometry,
                                                   this);

    // Initialize dwa local planner
    m_dp.initialize("hsr_dwa_planner", &m_buffer, m_localCostmapROS);

    // Confirm initialization
    m_initialized = true;
}

/**
 * Main logic of the control class
 * that decides which 
 */
void Control::handlePlan(const hsr_navigation::PlannerService &p_service)
{
    // Choose simple control or action
    if (p_service.response.obstacles_out.empty())
    {
        if (DEBUGCONTROL)
        {
            ROS_INFO("Control: path is empty -- starting simple DWA control.");
        }

        dwaControl(p_service.response.path);
    }
    else
    {
        if (DEBUGCONTROL)
        {
            ROS_INFO("Control: path is obstructed -- starting action control.");
        }

        actionControl(p_service.response.path, p_service.response.obstacles_out);
    }
}

/**
 * DWA local planner to reach
 * the goal without the need to
 * perform any sort of action.
 */
void Control::dwaControl(const std::vector<geometry_msgs::PoseStamped> &p_path)
{
    // Reset flags
    m_stopControl = false;
    m_postActionPlan = false;

    // Set global plan for local planner
    if (m_dp.setPlan(p_path))
    {
        if (DEBUGCONTROL)
        {
            ROS_INFO("Control: DWA set plan succeeded.");
        }
    }
    else
    {
        if (DEBUGCONTROL)
        {
            ROS_ERROR("Control: DWA set plan failed.");
        }
    }
    
    // Velocity command
    geometry_msgs::Twist l_cmd_vel;

    // Control loop
    while (!m_stopControl && !m_dp.isGoalReached())
    {
        // Compute local velocities
        if (!m_dp.computeVelocityCommands(l_cmd_vel))
        {
            if (DEBUGCONTROL)
            {
                ROS_ERROR("Control: DWA velocities computation failed.");
            }
        }

        // Send commands
        if (DEBUGCONTROL)
        {
            //ROS_INFO_STREAM(l_cmd_vel);
        }
        m_velPub.publish(l_cmd_vel);

        // Keep spinning
        ros::spinOnce();
    }

    if (m_dp.isGoalReached())
    {
        if (DEBUGCONTROL)
        {
            ROS_INFO("Control: Goal reached :)");
        }

        // If action control then
        // perform action for removal
        if (m_pushAction || m_graspAction || m_kickAction)
        {
            // Check which action to perform
            if (m_pushAction)
            {
                if (DEBUGCONTROL)
                {
                    ROS_INFO("Starting push control.");
                }

                // Call push routine
                performAction("push");

                // Reset flag
                m_pushAction = false;
            }
            else if (m_graspAction)
            {   
                if (DEBUGCONTROL)
                {
                    ROS_INFO("Starting grasp control.");
                }

                // Call grasp routine
                performAction("grasp");

                // Reset related flags
                m_graspAction = false;
                m_publishFrame = false;
            }
            else
            {
                if (DEBUGCONTROL)
                {
                    ROS_INFO("Starting kick control.");
                }

                // Call kick routine
                performAction("kick");

                // Reset related flags
                m_kickAction = false;
                m_publishFrame = false;
            }
            
            // Request new plan
            m_postActionPlan = true;
        }
    }
    else
    {
        if (DEBUGCONTROL)
        {
            ROS_INFO_STREAM("Control: stopping control for replanning ");
        }
    }
}

/**
 * Action control that 
 */
void Control::actionControl(const std::vector<geometry_msgs::PoseStamped> &p_path,
                            const std::vector<hsr_navigation::ObjectMessage> &p_obstacles)
{
    if (DEBUGCONTROL)
    {
        ROS_INFO("Control: action control started.");

        std::cout << "Number of elements received: " << p_obstacles.size() << std::endl;

        for (int x = 0; x < p_obstacles.size(); x++)
        {
            std::cout << "Element: " << x << ". Object class: " << unsigned(p_obstacles[x].object_class) << std::endl;
        }
    }

    // Get which object is to be manipulated
    unsigned int l_objIdx = p_obstacles.size() - 1;
    // unsigned int l_objIdx = 0;

    // Set action to be performed
    // based on the obstacles uid 
    // received by planner
    if (unsigned(p_obstacles[l_objIdx].object_class) == 3)
    {
        // Set action as push
        // for DWA control check
        m_pushAction = true;
    }
    else if (unsigned(p_obstacles[l_objIdx].object_class) == 5)
    {
        // Set action as grasp
        // for DWA control check
        m_graspAction = true;

        // Extract x and y pose
        // of the object in map
        // coordinate system
        double l_mx = p_obstacles[0].center_wx;
        double l_my = p_obstacles[0].center_wy;

        // Set object frame
        setObjectFrame(l_mx, l_my);

        // Start publishing frame
        // of the object to be grasped
        m_publishFrame = true;
    }
    else
    {
        // Set action as push
        // for DWA control check
        m_kickAction = true;

        // Extract x and y pose
        // of the object in map
        // coordinate system
        double l_mx = p_obstacles[0].center_wx;
        double l_my = p_obstacles[0].center_wy;

        // Set object frame
        setObjectFrame(l_mx, l_my);

        // Start publishing frame
        // of the object to be kicked
        m_publishFrame = true;
    }

    // Compute at which index the
    // intermediate point for the
    // initial part of the path is
    unsigned int l_idx = getIndex(p_path);

    // Extract intermediate path
    std::vector<geometry_msgs::PoseStamped> l_intermediatePath(p_path.begin(), 
                                                               p_path.begin() + l_idx - 2);

    // Log
    if (DEBUGCONTROL)
    {
        std::cout << "Path: " << p_path.size() << std::endl;
        std::cout << "Intermediate: " << l_intermediatePath.size() << std::endl;
    }

    // Set intermediate path
    dwaControl(l_intermediatePath);

    // if (!p_obstacles.empty())
    // {
    //     // Get which obstacle to manipulate
    //     float l_minDistance = 100000000;
    //     unsigned int l_objIdx = -1;
    //     for (int x = 0; x < p_obstacles.size(); x++)
    //     {
    //         // Compute distance
    //         float l_distance = distance(p_path[l_idx], p_obstacles[x].center_cell);

    //         // Log
    //         std::cout << "Object: " << unsigned(p_obstacles[x].object_class) << ". Distance: " << l_distance << std::endl; 

    //         // Get min distance
    //         if (l_distance < l_minDistance)
    //         {
    //             l_objIdx = x;
    //             l_minDistance = l_distance;
    //         }
    //     }

    //     std::cout << "Object ID to be manipulated is: " << l_objIdx << std::endl;
    //     std::cout << "Min distance is: " << l_minDistance << std::endl;    
    // }
}

/**
 * Robotic arm action to
 * either grasp of kick object
 */
void Control::performAction(const std::string &p_action)
{
    // Create service client
    ros::ServiceClient l_client;
    l_client = m_nh.serviceClient<hsr_navigation::ActionService>("action_service");

    // Populate action request
    hsr_navigation::ActionService l_service;
    l_service.request.action = p_action;

    // Call action service
    if (l_client.call(l_service))
    {   
        if (DEBUGCONTROL)
        {
            ROS_INFO("Action Service: service called succesfully.");
        }

        if (l_service.response.output)
        {
            ROS_INFO("Action Service: action performed.");
        }
        else
        {
            ROS_INFO("Action Service: failed to perform action.");
        }
    }
    else
    {
        if (DEBUGCONTROL)
        {
            ROS_INFO("Action Service: failed to call service.");
        }     
    }
}

/**
 * Get intermediat path index
 */
unsigned int Control::getIndex(const std::vector<geometry_msgs::PoseStamped> &p_path)
{
    // Costmap2D
    costmap_2d::Costmap2D *l_globalCostmap = m_globalCostmapROS->getCostmap();

    // Temporary holders
    int l_mx;
    int l_my;

    // Index of obstructed cell
    unsigned int l_idx = 0;

    // Find first obstructed cell
    for (auto poseStamped: p_path)
    {
        // World coordinates
        double l_wx = poseStamped.pose.position.x;
        double l_wy = poseStamped.pose.position.y;

        // Cast from world to map
        l_globalCostmap->worldToMapEnforceBounds(l_wx, l_wy, l_mx, l_my);

        // Get cost (convert to int from unsigned char)
        int l_cellCost = (int) l_globalCostmap->getCost(l_mx, l_my);
        std::cout << "Cost: " << l_cellCost << "at :" << l_mx << " " << l_my << std::endl;

        // Log cost
        if (l_cellCost > 250)
        {
            if (DEBUGCONTROL)
            {
                ROS_INFO_STREAM("Control: Obstructed cell found at idx: " << l_idx);
                ROS_INFO_STREAM("Control: l_wx: " << l_wx);
                ROS_INFO_STREAM("Control: l_wy: " << l_wy);
            }

            break;
        }
        else
        {
            // Increase counter
            l_idx += 1;
        }   
    }

    return l_idx;
}

/**
 * Set odometry message
 */
void Control::setOdometry(const nav_msgs::Odometry p_data)
{
    // Publish tf frame
    // of object to be grasped
    if (m_publishFrame)
    {
        m_broadcaster.sendTransform(tf::StampedTransform(m_transform, 
                                                         ros::Time::now(), 
                                                         "map", 
                                                         OBJECT_FRAME));
    }
}

/**
 * Sets object frame transform
 * for object manipulation tasks
 */
void Control::setObjectFrame(const double p_mx, const double p_my)
{
    // Set obstacle frame transform origin
    m_transform.setOrigin(tf::Vector3(p_mx, p_my, 0.0));

    // Get transformation from palm link
    // to map coordinate frame
    geometry_msgs::TransformStamped l_out;
    l_out = m_buffer.lookupTransform("hand_palm_link", 
                                     "map",
                                     ros::Time(0),
                                     ros::Duration(1.0));

    // Set quaternion
    tf::Quaternion l_q(
        l_out.transform.rotation.x,
        l_out.transform.rotation.y,
        l_out.transform.rotation.z,
        l_out.transform.rotation.w
    );
    
    // Get RPY in radians
    double l_roll, l_pitch, l_yaw;
    tf::Matrix3x3 l_m(l_q);
    l_m.getRPY(l_roll, l_pitch, l_yaw);

    std::cout << "Roll: " << l_roll << " Pitch: " << l_pitch << " Yaw: " << l_yaw << std::endl;
    std::cout << "Mean point x: " << p_mx << " Mean  point y: " << p_my << std::endl;

    // Set new quaternion to
    // match hand_palm_link
    // yaw offset during manipulation
    l_q.setRPY(l_roll, l_pitch, l_yaw);

    // Set transform rotation
    m_transform.setRotation(l_q);
}

/**
 * Compute euclidean distance
 * to find which object on the
 * path is to be manipulated
 */
float Control::distance(geometry_msgs::PoseStamped p_pose, hsr_navigation::CellMessage p_omcenter)
{
    float l_x = p_pose.pose.position.x - p_omcenter.mx;
    float l_y = p_pose.pose.position.y - p_omcenter.my;

    float l_distance = sqrt(pow(l_x, 2) + pow(l_y, 2));

    return l_distance;
}

/**
 * Let control know about a
 * new plan that has been set
 */
void Control::stopControl()
{
    m_stopControl = true;
}

/**
 * Method used by the navigation
 * class to avoid replanning while
 * action/manipulation is in course
 */
bool Control::actionInCourse()
{
    return (m_pushAction || m_graspAction || m_kickAction);
}

/**
 * Confirms that control was
 * initialized successfully
 */
bool Control::initialized()
{
    return m_initialized;
}

/**
 * Confirms to navigation logic
 * that control requested a post
 * action re-planning after manipulation
 */
bool Control::postActionPlan()
{
    return m_postActionPlan;
}