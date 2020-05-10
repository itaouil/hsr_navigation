// Header files
#include "control.hpp"

/**
 * Default constructor.
 */
Control::Control(tf2_ros::Buffer &p_buf, costmap_2d::Costmap2DROS* p_lc, costmap_2d::Costmap2DROS* p_gc): 
    m_buffer(p_buf), m_localCostmapROS(p_lc), m_globalCostmapROS(p_gc)
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

        actionControl(p_service.response.path);
    }
}

/**
 * DWA local planner to reach
 * the goal without the need to
 * perform any sort of action.
 */
void Control::dwaControl(const std::vector<geometry_msgs::PoseStamped> &p_path)
{
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
    while (!m_newPlan && !m_dp.isGoalReached())
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
        if (m_action)
        {
            push();

            // Reset flag
            m_action = false;
        }
    }
    else
    {
        if (DEBUGCONTROL)
        {
            ROS_INFO_STREAM("Control: stopping control for replanning ");
        }
    }

    // Reset replan flag
    m_newPlan = false;
}

/**
 * Action control that 
 */
void Control::actionControl(const std::vector<geometry_msgs::PoseStamped> &p_path)
{
    if (DEBUGCONTROL)
    {
        ROS_INFO("Control: action control started.");
    }

    // Set action flag to avoid
    // replanning by navigation
    m_action = true;

    // Get intermiate pose index
    unsigned int l_idx = getIndex(p_path);

    // Extract intermediate path
    std::vector<geometry_msgs::PoseStamped> l_intermediatePath(p_path.begin(), 
                                                               p_path.begin() + l_idx - 6);

    // Send robot to intermediate path
    std::cout << "Path: " << p_path.size() << std::endl;
    std::cout << "Intermediate: " << l_intermediatePath.size() << std::endl;

    // Set intermediate path
    dwaControl(l_intermediatePath);    
}

/**
 * Push action to remove obstacle
 */
void Control::push()
{
    if (DEBUGCONTROL)
    {
        ROS_INFO("Control: starting pushing action.");
    }

    // Pause costmaps
    // m_localCostmapROS->pause();
    // m_globalCostmapROS->pause();

    // Rotate by 180 degrees
    rotate(90);

    // Allow odometry computations
    m_push = true;

    // Populate velocity command
    geometry_msgs::Twist l_cmd_vel;
    l_cmd_vel.linear.x = -0.2;

    // Push until distance is
    // above a certain threshold
    while (m_totalDistance < DISTANCE)
    {
        // Apply linear velocity
        ROS_INFO("Control: applying push velocity.");
        m_velPub.publish(l_cmd_vel);

        // Keep spinning
        ros::spinOnce();   
    }

    // Back up from obstacle
    // Populate velocity command
    l_cmd_vel.linear.x = 0.2;

    // Push until distance is
    // above a certain threshold
    while (m_totalDistance < DISTANCE)
    {
        // Apply linear velocity
        ROS_INFO("Control: backing up.");
        m_velPub.publish(l_cmd_vel);

        // Keep spinning
        ros::spinOnce();   
    }

    // Rotate by -180 degrees
    rotate(-90);

    // Restart costmaps
    // m_localCostmapROS->start();
    // m_globalCostmapROS->start();

    // Clear variables
    m_push = false;
    m_previousX = 0;
    m_previousY = 0;
    m_firstRun = true;
    m_totalDistance = 0;
}

/**
 * Grasp action to remove
 * obstacle from the path
 */
void Control::grasp()
{
    //TODO: implement grasping action
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
        //std::cout << "Cost: " << l_cellCost << "at :" << l_mx << " " << l_my << std::endl;

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
    m_odometry = p_data;

    // Only compute travelled
    // distance if control is
    // in push action mode
    if (m_push)
    {
        // Store initial pose
        if(m_firstRun)
        {
            m_previousX = p_data.pose.pose.position.x;
            m_previousY = p_data.pose.pose.position.y;
            m_firstRun = false;
        }

        // Get current pose
        double l_x = p_data.pose.pose.position.x;
        double l_y = p_data.pose.pose.position.y;

        // Add increment to total distance
        m_totalDistance += sqrt(pow(l_x - m_previousX, 2) + pow(l_y - m_previousY, 2));
        
        std::cout << "Total distance travelled so far is: " << m_totalDistance << std::endl;

        // Update previous point
        m_previousX = p_data.pose.pose.position.x;
        m_previousY = p_data.pose.pose.position.y;
    }
}

/**
 * Let control know about a
 * new plan that has been set
 */
void Control::setNewPlan()
{
    m_newPlan = true;
}

/**
 * Method used by the navigation
 * class to avoid replanning while
 * action/manipulation is in course
 */
bool Control::actionInCourse()
{
    return m_action;
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
 * Rotate robot by 180 degrees
 * for push action
 */
void Control::rotate(const unsigned int p_degrees)
{
    // Rotational difference
    double l_diff = 10000000;

    // Target rotation
    double l_targetRad = p_degrees * (M_PI / 180);

    while (l_diff > 0.01)
    {
        // Create quaternion
        double l_quatX = m_odometry.pose.pose.orientation.x;
        double l_quatY = m_odometry.pose.pose.orientation.y;
        double l_quatZ = m_odometry.pose.pose.orientation.z;
        double l_quatW = m_odometry.pose.pose.orientation.w;
        tf::Quaternion l_quat(l_quatX, l_quatY, l_quatZ, l_quatW);

        // Creat matrix from quaternion
        tf::Matrix3x3 l_m(l_quat);

        // Extract axis angle representation
        double l_roll, l_pitch, l_yaw;
        l_m.getRPY(l_roll, l_pitch, l_yaw);

        // Compute rotation difference
        l_diff = l_targetRad - l_yaw;

        // Populate velocity command
        geometry_msgs::Twist l_cmd_vel;
        l_cmd_vel.angular.z = 0.2 * (l_diff);

        // Publish rotation velocity
        m_velPub.publish(l_cmd_vel);

        // Log
        std::cout << "Target: " << l_targetRad << " Current: " << l_yaw << std::endl;

        // Keep receiving data
        ros::spinOnce();   
    }
}
