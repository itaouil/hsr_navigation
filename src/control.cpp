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
        if (m_action)
        {
            // Check if push action
            if (m_pushAction)
            {
                push();
            }
            
            // Check if grasp action
            if (m_graspAction)
            {
                grasp();
            }
            
            // Reset action flags
            m_action = false;
            m_pushAction = false;
            m_graspAction = false;

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
    }

    // Set action flag to avoid
    // replanning by navigation
    m_action = true;

    // Set action to be performed
    // based on the obstacles received
    // by the planner
    if (p_obstacles[0].object_class == 1)
    {
        m_pushAction = true;
    }
    else
    {
        m_graspAction = true;
    }

    // Get intermiate pose index
    unsigned int l_idx = getIndex(p_path);

    // Extract intermediate path
    std::vector<geometry_msgs::PoseStamped> l_intermediatePath(p_path.begin(), 
                                                               p_path.begin() + l_idx - 2);

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

    // Rotate
    rotate(180);

    // Allow push
    m_push = true;

    // Populate velocity command
    geometry_msgs::Twist l_cmd_vel;
    l_cmd_vel.linear.x = -0.2;

    // Push
    while (ros::ok)
    {
        if (m_totalDistance > DISTANCE)
        {
            clear();
            break;
        }
        else
        {
            // Apply linear velocity
            ROS_INFO("Control: pushing.");
            m_velPub.publish(l_cmd_vel);

            // Keep spinning
            ros::spinOnce();
            m_rate.sleep();
        }
    }

    // Allow backtracking
    m_push = true;

    // Back up from obstacle
    // Populate velocity command
    l_cmd_vel.linear.x = 0.2;

    // Backtrack
    while (ros::ok)
    {
        if (m_totalDistance > DISTANCE)
        {
            clear();
            break;
        }
        else
        {
            // Apply linear velocity
            ROS_INFO("Control: backtracking.");
            m_velPub.publish(l_cmd_vel);

            // Keep spinning
            ros::spinOnce();
            m_rate.sleep();
        }
    }

    // Rotate by 180 degrees
    rotate(180);

    // Clear variables
    clear();
}

/**
 * Grasp action to remove
 * obstacle from the path
 */
void Control::grasp()
{
    std::cout << "I SHOULD START GRASPIIIIIIIING RIGHT NOW....." << std::endl;
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
 * Rotate robot by 180 degrees
 * for push action
 */
void Control::rotate(const unsigned int p_degrees)
{
    // Converting from angles to radians
    double l_relativeAngle = p_degrees * (M_PI / 180);

    // Velocity command
    geometry_msgs::Twist l_cmd_vel;
    l_cmd_vel.linear.x = 0;
    l_cmd_vel.linear.y = 0;
    l_cmd_vel.linear.z = 0;
    l_cmd_vel.angular.x = 0;
    l_cmd_vel.angular.y = 0;
    l_cmd_vel.angular.z = 0.4;

    // Distance computations
    double l_currentAngle = 0;
    double l_t0 = ros::Time::now().toSec();

    while(l_currentAngle < l_relativeAngle)
    {
        m_velPub.publish(l_cmd_vel);
        double l_t1 = ros::Time::now().toSec();
        l_currentAngle = 0.4 * (l_t1 - l_t0);
    }

    // Stop rotation
    l_cmd_vel.angular.z = 0;
    m_velPub.publish(l_cmd_vel);
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
 * Confirms to navigation logic
 * that control requested a post
 * action re-planning after manipulation
 */
bool Control::postActionPlan()
{
    return m_postActionPlan;
}

/**
 * Clear push variables
 * for the sequential push
 * actions
 */
void Control::clear()
{
    m_push = false;
    m_previousX = 0;
    m_previousY = 0;
    m_firstRun = true;
    m_totalDistance = 0;
}
