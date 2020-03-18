// Header files
#include "planner.hpp"
#include "clutter_planner.hpp"

// General imports
#include <iostream>

Planner::Planner()
{
}

Planner::~Planner() 
{
}

void Planner::dwaPlanning()
{
}

void Planner::aStarPlanning()
{
}

void Planner::loadOccupancyMap()
{
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner");
    ros::NodeHandle n;

    return 0;
}