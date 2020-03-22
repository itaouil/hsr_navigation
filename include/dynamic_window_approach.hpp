#ifndef DWA_HPP_
#define DWA_HPP_

// Parameters
#include "dwa_parameters.hpp"

class DWA
{
public:
    // Contructor (explicit)
    explicit DWA();

    // Destructor (virtual)
    virtual ~DWA();

private:
    /**
     * Class methods
     */

    // Robot parameters for DWA
    const float m_dt = DT;
    const float m_maxYaw = MAX_YAW;
    const float m_maxSpeed = MAX_SPEED;
    const float m_minSpeed = MIN_SPEED;
    const float m_goalCost = GOAL_COST;
    const float m_speedCost = SPEED_COST;
    const float m_robotRadius = ROBOT_RADIUS;
    const float m_predictTime = PREDICT_TIME;
    const float m_obstacleCost = OBSTACLE_COST;
    const float m_yawResolution = YAW_RESOLUTION;
    const float m_speedResolution = SPEED_RESOLUTION;
    const float m_maxAcceleration = MAX_ACCELERATION;
    const float m_maxYawAcceleration = MAX_YAW_ACCELERATION;

    /**
     * Class members
     */
};

#endif // DWA_HPP_
