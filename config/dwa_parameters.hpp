#ifndef DWA_PARAMETERS_HPP_
#define DWA_PARAMETERS_HPP_

#include <math.h>

// Robot velocities parameters
float MAX_SPEED = 1.0f; // [m/s]
float MIN_SPEED = -0.5f; // [m/s]
float MAX_YAW = 40.0f * M_PI / 180.0f; // [rad/s]

// Robot acceleration parameters
float MAX_ACCELERATION = 0.2f; // [m/ss]
float MAX_YAW_ACCELERATION = 40.0f * M_PI / 180.0f; // [r/ss]

// Robot parameters resolutions
float SPEED_RESOLUTION = 0.01; // [m/s]
float YAW_RESOLUTION = 0.1F * M_PI / 180.0f; // [rad/s]

// Time parameters (delta t)
float DT = 0.1f; // [s] time tick for motion prediction
float PREDICT_TIME = 3.0f; // [s]

// Heuristic parameters
float GOAL_COST = 0.15f;
float SPEED_COST = 1.0f;
float OBSTACLE_COST = 1.0f;

// Robot radius
float ROBOT_RADIUS = 1.0f; // [m]

#endif
