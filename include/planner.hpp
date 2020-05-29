// Copyright (c) 2016, Peter Regier
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the University of Freiburg nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef PLANNER_HPP__
#define PLANNER_HPP__

// General imports
#include <list>
#include <queue>
#include <fstream>
#include <unordered_map>
#include <unordered_set>

// ROS imports
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <ros/service_server.h>
#include <geometry_msgs/Pose.h>
#include <costmap_2d/cost_values.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <hsr_navigation/ObjectMessage.h>
#include <hsr_navigation/PlannerService.h>

// Parameters
#include "parameters.hpp"

enum ObjectClass
{
  BACKGROUND,
  BALL,
  BOOKS,
  BOXES,
  CARS,
  DOLL,
  STAFFED,
  BLOCKS
};

struct Cell
{
  unsigned int mx_;
  unsigned int my_;
  Cell() : mx_(-1), my_(-1)
  {
  }
  Cell(unsigned int x, unsigned int y) : mx_(x), my_(y)
  {
  }
  Cell(const Cell &other) : mx_(other.mx_), my_(other.my_){};
  bool operator==(const Cell &other) const
  {
    return mx_ == other.mx_ && my_ == other.my_;
  }
  bool operator!=(const Cell &other) const
  {
    return !(*this == other);
  }
};

struct PriorityQueue
{
public:
  typedef std::pair<Cell, double> Element;
  struct ElementCompare
  {
    bool operator()(const Element a, const Element b) const
    {
      return a.second > b.second;
    }
  };
  std::priority_queue<Element, std::vector<Element>, ElementCompare> elements;

  inline bool empty() const
  {
    return elements.empty();
  }

  inline void put(const Cell &item, double priority)
  {
    elements.push(std::make_pair(item, priority));
  }

  inline Cell get()
  {
    Cell best_item = elements.top().first;
    elements.pop();
    return best_item;
  }
};

struct InflationCell
{
public:
  int mx_, my_;
  unsigned char cost;

  InflationCell() : mx_(0), my_(0), cost(0)
  {
  }
  InflationCell(int mx, int my, unsigned char cost) : mx_(mx), my_(my), cost(cost)
  {
  }
};

class Planner
{
public:
  explicit Planner();
  virtual ~Planner();

  bool PlannerSrv(hsr_navigation::PlannerService::Request &,
                  hsr_navigation::PlannerService::Response &);

  void inflateStaticMap();

  void createPartialObjectMap();

  void getCellInflationVector();

  void getObjectInflationCells(std::vector<hsr_navigation::ObjectMessage> &,
                               std::vector<std::set<std::pair<unsigned int, unsigned int>>> &);

  void createObjectMap(std::vector<hsr_navigation::ObjectMessage> &,
                       std::vector<std::set<std::pair<unsigned int, unsigned int>>> &);

  inline unsigned char computeInflationCost(double distance) const
  {
    unsigned char cost = 0;
    if (distance <= m_cellRobotRadius)
      cost = costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    else
    {
      double euclidean_distance = distance * m_resolution;
      double factor = exp(-1.0 * 10 * (euclidean_distance - m_robotRadius));
      cost = (unsigned char)((costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
    }
    return cost;
  }

  bool makePlan(geometry_msgs::PoseStamped &,
                geometry_msgs::PoseStamped &,
                std::vector<geometry_msgs::PoseStamped> &);

  void reconstructPath(const Cell &,
                       const Cell &,
                       std::unordered_map<Cell, Cell> &,
                       std::vector<Cell> &);

  bool worldToMap(double &,
                  double &,
                  unsigned int &,
                  unsigned int &);

  void mapToWorld(unsigned int &,
                  unsigned int &,
                  double &,
                  double &);

  void cellPathToOutput(std::vector<Cell> &,
                        std::vector<geometry_msgs::PoseStamped> &);

  double cost_multplier(const Cell &,
                        const Cell &) const;

  inline double heuristic(const Cell &,
                          const Cell &);

  void createOccSemGrid(const hsr_navigation::PlannerService::Request &,
                        std::vector<hsr_navigation::ObjectMessage> &,
                        std::vector<std::set<std::pair<unsigned int, unsigned int>>> &);

  void constraintAStar(Cell &,
                       Cell &,
                       std::unordered_map<Cell, Cell> &,
                       std::unordered_map<Cell, double> &,
                       std::vector<Cell> &);

  void getConstraintNeighbors(const Cell &,
                              std::vector<Cell> &);

  void publishPlan(const std::vector<geometry_msgs::PoseStamped> &);

  void pointToNext(std::vector<geometry_msgs::PoseStamped> &,
                   int &);

  // General members
  // cv::Mat m_colorImg;
  bool m_backUpPlan;
  double m_originX = -1;
  double m_originY = -1;
  double m_resolution = -1;
  double m_robotRadius = -1;
  unsigned int m_width = -1;
  unsigned int m_height = -1;
  //double m_inflationRadius;
  unsigned char m_enterCost;
  unsigned char m_movingCost;
  unsigned char m_objMovCost;
  unsigned int m_cellRobotRadius;
  unsigned char m_changeObjectCost;
  unsigned int m_cellInflationRadius = -1;
  unsigned int extra_inflation;

  std::vector<InflationCell> m_cellInflationCost;
  std::vector<InflationCell> m_cellLethalInflation;
  std::set<long long unsigned int> m_objectsOnPath;
  std::vector<std::vector<unsigned char>> m_staticMap;
  std::vector<std::vector<unsigned int>> m_objectClassCostMap;
  std::vector<std::vector<std::list<long long unsigned int>>> m_objectMap;
  std::vector<std::set<std::pair<unsigned int, unsigned int>>> m_objectsInflationCells;

  // ROS members
  ros::NodeHandle m_n;
  ros::Publisher m_gridPub;
  ros::Publisher m_planPub;
  ros::ServiceServer m_server;
  nav_msgs::OccupancyGrid m_objectGrid;
};

namespace std
{
template <>
class hash<Cell>
{
public:
  size_t operator()(const Cell &cell) const
  {
    return cell.mx_ * 1000000 + cell.my_;
  }
};
}

#endif /* Planner_HPP_ */
