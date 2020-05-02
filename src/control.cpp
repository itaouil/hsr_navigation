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
        if (DEBUG)
        {
            ROS_INFO("Control: path is empty -- starting simple DWA control.");
        }

        dwaControl(p_service.response.path);
    }
    else
    {
        if (DEBUG)
        {
            ROS_INFO("Control: path is obstructed -- starting action control.");
            ROS_INFO_STREAM(p_service.response.obstacles_out);
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
        if (DEBUG)
        {
            ROS_INFO("Control: DWA set plan succeeded.");
        }
    }
    else
    {
        if (DEBUG)
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
            if (DEBUG)
            {
                ROS_ERROR("Control: DWA velocities computation failed.");
            }
        }

        // Send commands
        if (DEBUG)
        {
            //ROS_INFO_STREAM(l_cmd_vel);
        }
        m_velPub.publish(l_cmd_vel);

        ros::spinOnce();
    }

    if (m_dp.isGoalReached())
    {
        if (DEBUG)
        {
            ROS_INFO("Control: Goal reached :)");
        }
    }
    else
    {
        if (DEBUG)
        {
            ROS_INFO("Control: stopping control for replanning");
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
    if (DEBUG)
    {
        ROS_INFO("Control: action control started.");
    }

    // Set action flag to avoid
    // replanning by navigation
    // logic
    m_action = true;

    // Costmap2D
    costmap_2d::Costmap2D *l_globalCostmap = m_globalCostmapROS->getCostmap();

    // Find intermediate point path
    // Map coordinates
    int l_mx;
    int l_my;

    // Index of obstructed cell
    unsigned int idx = 0;

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

        // Log cost
        if (l_cellCost > 253)
        {
            if (DEBUG)
            {
                ROS_INFO_STREAM("Control: Obstructed cell found at idx: " << idx);
            }

            // Increase counter
            idx += 1;

            break;
        }

        // Increase counter
        idx += 1;
    }

    // Extract intermediate path
    std::vector<geometry_msgs::PoseStamped> l_intermediatePath(p_path.begin(), 
                                                               p_path.begin() + idx - 2);

    // Send robot to intermediate path
    dwaControl(l_intermediatePath);

    if (DEBUG)
    {
        ROS_INFO("Control: Intermediate path reached.");
    }

    // Resume orignal plan
    dwaControl(p_path);

    // Reset action flag
    m_action = false;
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
