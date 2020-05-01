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
}

/**
 * Main logic of the control class
 * that decides which 
 */
void Control::handlePlan(const hsr_navigation::PlannerService &p_service)
{
    if (p_service.response.obstacles_out.empty())
    {
        dwaControl(p_service.response.path);
    }
    else
    {
        actionControl();
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
            ROS_INFO("DWA set plan: SUCCESS");
        }
    }
    else
    {
        if (DEBUG)
        {
            ROS_ERROR("DWA set plan: FAILED");
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
                ROS_ERROR("DWA velocities computation failed.");
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
            ROS_INFO("GOAL REACHED :)");
        }
    }
    else
    {
        if (DEBUG)
        {
            ROS_INFO("STOPPING DWA FOR REPLANNING...");
        }
    }
}

/**
 * Action control that 
 */
void Control::actionControl(const std::vector<geometry_msgs::PoseStamped> &p_path)
{
    // Set action flag to avoid
    // replanning by navigation
    // logic
    m_action = true;

    //TODO: compute intermediate point path

    //TODO: reach intermediate point

    //TODO: start action (push/manipulation)

    //TODO: resume original path
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