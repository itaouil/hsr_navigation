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

// Class header
#include "planner.hpp"

/**
 * Constructor.
 *
 * Initalizes members, and
 * creates two custom publishers.
 */
Planner::Planner()
{
	// Initialize members
	m_enterCost = ENTERCOST;
	m_movingCost = MOVINGCOST;
	m_objMovCost = OBJMOVCOST;
	m_backUpPlan = BACKUPPLAN;
	m_robotRadius = ROBOTRADIUS;
	//m_inflationRadius = INFLATIONRADIUS;
	m_changeObjectCost = CHANGEOBJECTCOST;
        extra_inflation = EXTRAINFLATION;
        
	// Create publishers
	m_planPub = m_n.advertise<nav_msgs::Path>("relaxed_constraint_plan", 1);
	m_gridPub = m_n.advertise<nav_msgs::OccupancyGrid>("server_grid", 1, true);
}

/**
 * Virtual destructor.
 */
Planner::~Planner()
{
}

/**
 * The function publishes a path
 * as a nav_msgs/Path using a publisher.
 */
void Planner::publishPlan(const std::vector<geometry_msgs::PoseStamped> &path)
{
	// Create nav_msgs/Path message
	// and resize it to be the same
	// length as the path received.
	nav_msgs::Path nav_path;
	nav_path.poses.resize(path.size());

	// Assign the timestamp, as well
	// as the frame_id of refernce
	// if the received path is not
	// empty.
	if (!path.empty())
	{
		nav_path.header.stamp = path[0].header.stamp;
		nav_path.header.frame_id = path[0].header.frame_id;
	}

	// Populate the nav_path message
	for (unsigned int i = 0; i < path.size(); i++)
	{
		nav_path.poses[i] = path[i];
	}

	// Publish message to topic
	m_planPub.publish(nav_path);
}

/**
 *
 */
bool Planner::PlannerSrv(hsr_navigation::PlannerService::Request &req,
									   hsr_navigation::PlannerService::Response &res)
{
	// Read the request message
	// information. This includes
	// the width of the map, its
	// height, the cell resolution,
	// and the orgin of the map (x,y)
	m_width = req.grid.info.width;
	m_height = req.grid.info.height;
	m_resolution = req.grid.info.resolution;
	m_originX = req.grid.info.origin.position.x;
	m_originY = req.grid.info.origin.position.y;

	// Compute radiuses
	m_cellRobotRadius = ceil(m_robotRadius / m_resolution);
	m_cellInflationRadius = m_cellRobotRadius + extra_inflation;//ceil(m_inflationRadius / m_resolution);

	// Clear vectors
	m_objectMap.clear();
	m_staticMap.clear();
	m_objectsOnPath.clear();
	m_cellInflationCost.clear();
	m_objectClassCostMap.clear();
	m_cellLethalInflation.clear();
	m_objectsInflationCells.clear();

	// Resize data holders
	// based on the height
	// and width of the grid map
	m_staticMap.resize(m_height,
					   std::vector<unsigned char>(m_width, costmap_2d::FREE_SPACE));
	m_objectMap.resize(m_height,
					   std::vector<std::list<long long unsigned int>>(m_width,
																	  std::list<long long unsigned int>()));
	m_objectClassCostMap.resize(m_height, std::vector<unsigned int>(m_width, 0));

	// Clear occupancy grid data
	m_objectGrid.data.clear();

	// Re-initialize nav_msgs/OccupancyGrid message
	m_objectGrid.header.stamp = ros::Time::now();
	m_objectGrid.header.frame_id = "map";
	m_objectGrid.info.map_load_time = ros::Time::now();
	m_objectGrid.info.resolution = 0.05;
	m_objectGrid.info.width = m_width;
	m_objectGrid.info.height = m_height;
	m_objectGrid.info.origin = req.grid.info.origin;
	m_objectGrid.info.origin.orientation.w = 1;
	m_objectGrid.data.resize(m_width * m_height);

	// Find highest probability of occupancy in the grid
	signed char maxValue = (*std::max_element(req.grid.data.begin(), req.grid.data.end()));

	// Assign lethal costs to lethal
	// obstacles based on previous max
	// occupancy value found
	for (int i = 0; i < m_height; ++i)
	{
		for (int j = 0; j < m_width; ++j)
		{
			signed char cBuffer = req.grid.data[i * m_width + j];

			if (cBuffer == maxValue)
			{
				m_staticMap[i][j] = costmap_2d::LETHAL_OBSTACLE;
			}
		}
	}

	// Compute inflation costs
	getCellInflationVector();
	inflateStaticMap();

	// Read obstacles found during the path planning
	std::vector<hsr_navigation::ObjectMessage> objects = req.obstacles_in;
	ROS_INFO("Planner ServerCB Objects In: %zu", objects.size());

	std::vector<hsr_navigation::ObjectMessage> objectsMsgOnPath;

	getObjectInflationCells(objects, m_objectsInflationCells);
	createObjectMap(objects, m_objectsInflationCells);

	std::vector<geometry_msgs::PoseStamped> plan;
	geometry_msgs::PoseStamped start_tmp = req.start;
	geometry_msgs::PoseStamped goal_tmp = req.goal;

	createOccSemGrid(req, objects, m_objectsInflationCells);

	m_gridPub.publish(m_objectGrid);

	if (makePlan(start_tmp, goal_tmp, plan))
	{

		for (int i = 0; i < plan.size() - 1; i++)
		{
			pointToNext(plan, i);
		}
//		std::ofstream myfile;
//		myfile.open("/home/pregier/Workspaces/testPath.csv");
//		for (int i = 0; i < plan.size(); ++i) {
//			myfile << plan[i].pose.position.x << " " << plan[i].pose.position.y << std::endl;
//		}
//		myfile.close();
		res.path = plan;
		publishPlan(plan);
	}
	else
		ROS_INFO("Service call fail!!!!!!!!!!!!!!!!!!");

	if (m_backUpPlan)
	{
		for (std::set<long long unsigned int>::iterator it = m_objectsOnPath.begin();
			 it != m_objectsOnPath.end(); ++it)
		{
			for (int j = 0; j < objects.size(); ++j)
			{
				if (*it == objects[j].uid)
				{
					objectsMsgOnPath.push_back(objects[j]);
					break;
				}
			}
		}
	}

	for (int j = 0; j < objectsMsgOnPath.size(); ++j)
	{
		ROS_INFO("UID: %lu", objectsMsgOnPath[j].uid);
	}
	res.obstacles_out = objectsMsgOnPath;

	return !plan.empty();
}

void Planner::pointToNext(std::vector<geometry_msgs::PoseStamped> &path, int &index)
{
	double x0 = path[index].pose.position.x, y0 = path[index].pose.position.y, x1 = path[index + 1].pose.position.x, y1 = path[index + 1].pose.position.y;

	double angle = atan2(y1 - y0, x1 - x0);
	path[index].pose.orientation = tf::createQuaternionMsgFromYaw(angle);
}

void Planner::constraintAStar(Cell &start, 
	Cell &goal, std::unordered_map<Cell, Cell> &came_from, 
	std::unordered_map<Cell, double> &cost_so_far, std::vector<Cell> &path)
{
	PriorityQueue frontier;
	std::unordered_set<Cell> closedList;
	frontier.put(start, 0);

	m_objectMap[start.my_][start.mx_].clear();
	m_objectMap[start.my_ + 1][start.mx_].clear();
	m_objectMap[start.my_ - 1][start.mx_].clear();
	m_objectMap[start.my_][start.mx_ + 1].clear();
	m_objectMap[start.my_][start.mx_ - 1].clear();

	cost_so_far.clear();
	came_from.clear();

	came_from[start] = start;
	cost_so_far[start] = 0.;
	path.clear();
	long long unsigned int current_object_id = 0;

	while (!frontier.empty())
	{
		Cell current = frontier.get();
		if (closedList.find(current) != closedList.end())
		{
			continue;
		}
		else
		{
			closedList.insert(current);
		}

		if (current == goal)
		{
			reconstructPath(start, goal, came_from, path);
			for (int i = 0; i < path.size(); ++i)
			{
				if (!m_objectMap[path[i].my_][path[i].mx_].empty())
				{
					m_objectsOnPath.insert(m_objectMap[path[i].my_][path[i].mx_].front());
				}
			}
			break;
		}

		std::vector<Cell> neighbors;
		getConstraintNeighbors(current, neighbors);

		if (m_objectMap[current.my_][current.mx_].empty())
		{
			current_object_id = 0;
		}
		else
		{
			current_object_id = m_objectMap[current.my_][current.mx_].front();
		}

		for (int i = 0; i < neighbors.size(); ++i)
		{
			Cell next = neighbors[i];
			double cm_costs;

			if (m_objectMap[next.my_][next.mx_].empty())
			{
				cm_costs = costmap_2d::FREE_SPACE;
			}
			else
			{
				if (current_object_id == 0)
				{
//					ROS_INFO("ENTER COST %d", m_objectClassCostMap[next.my_][next.mx_]);
					cm_costs = m_enterCost + 2 * m_objectClassCostMap[next.my_][next.mx_];
				}
				else if (current_object_id != m_objectMap[next.my_][next.mx_].front())
				{
					cm_costs = m_changeObjectCost + 2 * m_objectClassCostMap[next.my_][next.mx_];
				}
				else
				{
					cm_costs = 0; //m_objectClassCostMap[next.my_][next.mx_];
				}
			}

			double new_cost = cost_so_far[current] + cost_multplier(current, next) * m_movingCost + cm_costs;

			if (!cost_so_far.count(next) || new_cost < cost_so_far[next] - 1e-5)
			{
				cost_so_far[next] = new_cost;
				double priority = new_cost + m_movingCost * heuristic(next, goal);
				frontier.put(next, priority);
				came_from[next] = current;
			}
		}
	}
}

void Planner::getConstraintNeighbors(const Cell &current, std::vector<Cell> &neighbors)
{

	unsigned int state;
	bool extend_neighbors = true;
	bool diag_neighbors = true;

	bool xp1, xm1, yp1, ym1, xp2, xm2, yp2, ym2;

	xp1 = current.mx_ + 1 < m_width;
	xm1 = current.mx_ - 1 < m_width;
	yp1 = current.my_ + 1 < m_height;
	ym1 = current.my_ - 1 < m_height;

	xp2 = current.mx_ + 2 < m_width;
	xm2 = current.mx_ - 2 < m_width;
	yp2 = current.my_ + 2 < m_height;
	ym2 = current.my_ - 2 < m_height;

	if (m_staticMap[current.my_ - 1][current.mx_] < costmap_2d::INSCRIBED_INFLATED_OBSTACLE && m_objectMap[current.my_ - 1][current.mx_].size() < 3 && ym1)
	{
		neighbors.push_back(Cell(current.mx_, current.my_ - 1));
	}
	else
	{
		diag_neighbors = false;
	}

	if (m_staticMap[current.my_ + 1][current.mx_] < costmap_2d::INSCRIBED_INFLATED_OBSTACLE && m_objectMap[current.my_ + 1][current.mx_].size() < 3 && yp1)
	{
		neighbors.push_back(Cell(current.mx_, current.my_ + 1));
	}
	else
	{
		diag_neighbors = false;
	}
	if (m_staticMap[current.my_][current.mx_ - 1] < costmap_2d::INSCRIBED_INFLATED_OBSTACLE && m_objectMap[current.my_][current.mx_ - 1].size() < 3 && xm1)
	{
		neighbors.push_back(Cell(current.mx_ - 1, current.my_));
	}
	else
	{
		diag_neighbors = false;
	}
	if (m_staticMap[current.my_][current.mx_ + 1] < costmap_2d::INSCRIBED_INFLATED_OBSTACLE && m_objectMap[current.my_][current.mx_ + 1].size() < 3 && xp1)
	{
		neighbors.push_back(Cell(current.mx_ + 1, current.my_));
	}
	else
	{
		diag_neighbors = false;
	}

	if (false /*diag_neighbors*/)
	{
		if (m_staticMap[current.my_ - 1][current.mx_ - 1] == costmap_2d::FREE_SPACE && m_objectMap[current.my_ - 1][current.mx_ - 1].size() < 1 && xm1 && ym1)
		{
			neighbors.push_back(Cell(current.mx_ - 1, current.my_ - 1));
		}
		else
		{
			extend_neighbors = false;
		}
		if (m_staticMap[current.my_ + 1][current.mx_ + 1] == costmap_2d::FREE_SPACE && m_objectMap[current.my_ + 1][current.mx_ + 1].size() < 1 && xp1 && yp1)
		{
			neighbors.push_back(Cell(current.mx_ + 1, current.my_ + 1));
		}
		else
		{
			extend_neighbors = false;
		}

		if (m_staticMap[current.my_ - 1][current.mx_ + 1] == costmap_2d::FREE_SPACE && m_objectMap[current.my_ - 1][current.mx_ + 1].size() < 1 && xp1 && ym1)
		{
			neighbors.push_back(Cell(current.mx_ + 1, current.my_ - 1));
		}
		else
		{
			extend_neighbors = false;
		}
		if (m_staticMap[current.my_ + 1][current.mx_ - 1] == costmap_2d::FREE_SPACE && m_objectMap[current.my_ + 1][current.mx_ - 1].size() < 1 && xm1 && yp1)
		{
			neighbors.push_back(Cell(current.mx_ - 1, current.my_ + 1));
		}
		else
		{
			extend_neighbors = false;
		}
	}
	bool isExtendTrue = false;
	if (m_objectMap[current.my_ + 1][current.mx_ - 1].size() < 1 && m_objectMap[current.my_ - 1][current.mx_ - 1].size() < 1 && m_objectMap[current.my_ + 1][current.mx_ + 1].size() < 1 && m_objectMap[current.my_ - 1][current.mx_ + 1].size() < 1 && m_objectMap[current.my_][current.mx_ + 1].size() < 1 && m_objectMap[current.my_][current.mx_ - 1].size() < 1 && m_objectMap[current.my_ - 1][current.mx_].size() < 1 && m_objectMap[current.my_ + 1][current.mx_].size() < 1)
	{
		isExtendTrue = true;
	}
	if (isExtendTrue && (extend_neighbors && diag_neighbors))
	{
		if (m_staticMap[current.my_ - 1][current.mx_ - 2] == costmap_2d::FREE_SPACE && m_objectMap[current.my_ - 1][current.mx_ - 2].size() < 1 && xm2 && ym1)
		{
			neighbors.push_back(Cell(current.mx_ - 2, current.my_ - 1));
		}
		if (m_staticMap[current.my_ + 1][current.mx_ - 2] == costmap_2d::FREE_SPACE && m_objectMap[current.my_ + 1][current.mx_ - 2].size() < 1 && xm2 && yp1)
		{
			neighbors.push_back(Cell(current.mx_ - 2, current.my_ + 1));
		}
		if (m_staticMap[current.my_ + 2][current.mx_ - 1] == costmap_2d::FREE_SPACE && m_objectMap[current.my_ + 2][current.mx_ - 1].size() < 1 && xm1 && yp2)
		{
			neighbors.push_back(Cell(current.mx_ - 1, current.my_ + 2));
		}
		if (m_staticMap[current.my_ + 2][current.mx_ + 1] == costmap_2d::FREE_SPACE && m_objectMap[current.my_ + 2][current.mx_ + 1].size() < 1 && xp1 && yp2)
		{
			neighbors.push_back(Cell(current.mx_ + 1, current.my_ + 2));
		}
		if (m_staticMap[current.my_ + 1][current.mx_ + 2] == costmap_2d::FREE_SPACE && m_objectMap[current.my_ + 1][current.mx_ + 2].size() < 1 && xp2 && yp1)
		{
			neighbors.push_back(Cell(current.mx_ + 2, current.my_ + 1));
		}
		if (m_staticMap[current.my_ - 1][current.mx_ + 2] == costmap_2d::FREE_SPACE && m_objectMap[current.my_ - 1][current.mx_ + 2].size() < 1 && xp2 && ym1)
		{
			neighbors.push_back(Cell(current.mx_ + 2, current.my_ - 1));
		}
		if (m_staticMap[current.my_ - 2][current.mx_ + 1] == costmap_2d::FREE_SPACE && m_objectMap[current.my_ - 2][current.mx_ + 1].size() < 1 && xp1 && ym2)
		{
			neighbors.push_back(Cell(current.mx_ + 1, current.my_ - 2));
		}
		if (m_staticMap[current.my_ - 2][current.mx_ - 1] == costmap_2d::FREE_SPACE && m_objectMap[current.my_ - 2][current.mx_ - 1].size() < 1 && xm1 && ym2)
		{
			neighbors.push_back(Cell(current.mx_ - 1, current.my_ - 2));
		}
	}
}

void Planner::createOccSemGrid(const hsr_navigation::PlannerService::Request &req,
									  std::vector<hsr_navigation::ObjectMessage> &objects,
									  std::vector<std::set<std::pair<unsigned int, unsigned int>>> &objectsInflationCells)
{
	if (!objects.empty())
	{
		int increasCostStep = 100 / objects.size();
		for (int i = 0; i < objects.size(); ++i)
		{
			for (std::set<std::pair<unsigned int, unsigned int>>::iterator k =
					 objectsInflationCells[i].begin();
				 k != objectsInflationCells[i].end(); ++k)
			{
				unsigned int mx, my;
				mx = k->first;
				my = k->second;
				m_objectGrid.data[my * m_width + mx] = 10 + i * increasCostStep;
			}
		}
	}

	for (int i = 0; i < m_height; ++i)
	{
		for (int j = 0; j < m_width; ++j)
		{
			if (m_objectMap[i][j].size() > 1)
			{
				m_objectGrid.data[i * m_width + j] = 200;
			}
			if (m_staticMap[i][j] == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
			{
				m_objectGrid.data[i * m_width + j] = 254;
			}
			else if (m_staticMap[i][j] > costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
			{
				m_objectGrid.data[i * m_width + j] = 255;
			}
		}
	}
}

/**
 * Updates the map costs based on
 * which object the cell contains.
 */
void Planner::createObjectMap(std::vector<hsr_navigation::ObjectMessage> &objects,
									 std::vector<std::set<std::pair<unsigned int, unsigned int>>> &objectsInflationCells)
{
	// Warning
	if (objects.size() != objectsInflationCells.size())
	{
		ROS_WARN(
			"Object inflation vector size differs from the objects vector size. Object %zu; Inflation %zu",
			objects.size(), objectsInflationCells.size());
	}
	// Iterate over objects
	for (int i = 0; i < objects.size(); ++i)
	{
		ROS_INFO("OBJECT CLASS ASSIGN %d", objects[i].object_class);
		for (std::set<std::pair<unsigned int, unsigned int>>::iterator k =
				 objectsInflationCells[i].begin();
			 k != objectsInflationCells[i].end(); ++k)
		{
			unsigned int mx = k->first;
			unsigned int my = k->second;

			m_objectMap[my][mx].push_back(objects[i].uid);
			// m_objectClassCostMap[my][mx].push_back(objects[i].object_class

			// Update cost map
			// Factor 5 for average cell size of object.
			
			if (objects[i].object_class == BALL)
				m_objectClassCostMap[my][mx] = 5*(2.1 * m_movingCost);
			if (objects[i].object_class == BOOKS)
				m_objectClassCostMap[my][mx] = 5*(5.1 * m_movingCost);
			if (objects[i].object_class == BOXES)
				m_objectClassCostMap[my][mx] = 5*(5.1 * m_movingCost);
			if (objects[i].object_class == CARS)
				m_objectClassCostMap[my][mx] = 5*(3.5 * m_movingCost);
			if (objects[i].object_class == DOLL)
				m_objectClassCostMap[my][mx] = 5*(4.5 * m_movingCost);
			if (objects[i].object_class == STAFFED)
				m_objectClassCostMap[my][mx] = 5*(4.5 * m_movingCost);
			if (objects[i].object_class == BLOCKS)
				m_objectClassCostMap[my][mx] = 5*(3.5 * m_movingCost);
		}
	}
}

/**
 * The function retrieves the position
 * of the inflated cells where objects
 * resides.
 */
void Planner::getObjectInflationCells(std::vector<hsr_navigation::ObjectMessage> &objects,
											 std::vector<std::set<std::pair<unsigned int, unsigned int>>> &objectsInflationCells)
{
	// Loop over objects found on the path
	for (int i = 0; i < objects.size(); ++i)
	{
		// Set containing the (x,y) position of the object cells
		std::set<std::pair<unsigned int, unsigned int>> inflationCells;

		// Iterate over (x,y) position of the cells
		for (int j = 0; j < objects[i].cell_vector.size(); ++j)
		{
			for (int k = 0; k < m_cellLethalInflation.size(); ++k)
			{
				int tmp_mx;
				int tmp_my;
				tmp_my = (int)(objects[i].cell_vector[j].my) + (int)(m_cellLethalInflation[k].my_);
				tmp_mx = (int)(objects[i].cell_vector[j].mx) + (int)(m_cellLethalInflation[k].mx_);
				if (tmp_my < 0 || tmp_mx < 0 || tmp_mx > m_width || tmp_my > m_height)
				{
					continue;
				}

				inflationCells.insert(std::make_pair(tmp_mx, tmp_my));
			}
		}

		objectsInflationCells.push_back(inflationCells);
	}
	//    ROS_INFO("OBJECT SIZE %zu", objectsInflationCells.size());
}

/**
 * Compute inflation costs for
 * the static map.
 */
void Planner::inflateStaticMap()
{
	for (int i = 0; i < m_height; ++i)
	{
		for (int j = 0; j < m_width; ++j)
		{
			if (m_staticMap[i][j] > costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
			{
				for (int k = 0; k < m_cellInflationCost.size(); ++k)
//				for (int k = 0; k < m_cellLethalInflation.size(); ++k)
				{
					int tmp_mx, tmp_my;
					tmp_my = i + (int)(m_cellInflationCost[k].my_);
					tmp_mx = j + (int)(m_cellInflationCost[k].mx_);
					if (tmp_my < 0 || tmp_mx < 0 || tmp_mx >= m_width || tmp_my >= m_height)
					{
						continue;
					}
					unsigned int oldCost = m_staticMap[tmp_my][tmp_mx];
					unsigned char cost = m_cellInflationCost[k].cost;
					if (cost > oldCost)
					{
						m_staticMap[tmp_my][tmp_mx] = cost; //costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
					}
				}
			}
		}
	}
}

/**
 * The function computes the new cost
 * of the cells after the inflation.
 */
void Planner::getCellInflationVector()
{
	// Clear inflation vectors
	m_cellInflationCost.clear();
	m_cellLethalInflation.clear();

	// Store cell inflation radius
	int intCIR = m_cellInflationRadius;

	for (int m = -intCIR; m <= intCIR; ++m)
	{
		for (int n = -intCIR; n <= intCIR; ++n)
		{
			// Compute distance
			double distance = sqrt((double)(m * m) + (double)(n * n));

			if (distance > intCIR)
				continue;

			// Compute inflation cost based on distance
			unsigned char cost = computeInflationCost(distance);

			// Populate vectors
			if (cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
			{
				m_cellInflationCost.push_back(InflationCell(n,
															m,
															costmap_2d::INSCRIBED_INFLATED_OBSTACLE));
				m_cellLethalInflation.push_back(InflationCell(n,
															  m,
															  costmap_2d::INSCRIBED_INFLATED_OBSTACLE));
			}
			else
			{
				m_cellInflationCost.push_back(InflationCell(n, m, cost));
			}
		}
	}
}

bool Planner::makePlan(geometry_msgs::PoseStamped &start,
							  geometry_msgs::PoseStamped &goal,
							  std::vector<geometry_msgs::PoseStamped> &plan)
{

	// Clear the plan, just in case
	plan.clear();

	// Temporary variables
	unsigned int cgoalx;
	unsigned int cgoaly;
	unsigned int cstartx;
	unsigned int cstarty;

	// Reset flag
	m_backUpPlan = false;

	// Store world (x,y) position for
	// the starting pose of the planning
	double wx = start.pose.position.x;
	double wy = start.pose.position.y;

	if (!worldToMap(wx, wy, cstartx, cstarty))
	{
		ROS_WARN(
			"The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
		ROS_WARN("START POSITION: wx %f; wy %f; cx %d cy %d", wx, wy, cstartx, cstarty);
		return false;
	}

	wx = goal.pose.position.x;
	wy = goal.pose.position.y;
	if (!worldToMap(wx, wy, cgoalx, cgoaly))
	{
		ROS_WARN(
			"The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
		return false;
	}

	std::vector<Cell> cell_path;
	Cell startCell(cstartx, cstarty), goalCell(cgoalx, cgoaly);

	std::unordered_map<Cell, Cell> came_from;
	std::unordered_map<Cell, double> cost_so_far;

	constraintAStar(startCell, goalCell, came_from, cost_so_far, cell_path);
	m_backUpPlan = true;

	cellPathToOutput(cell_path, plan);

	if (!plan.empty())
	{
		//        publishPlan(plan);
		ROS_INFO("PLAN SIZE IN SERVER %zu", plan.size());
	}
	else
	{
		ROS_ERROR("No plan found");
	}
	return !plan.empty();
}

void Planner::reconstructPath(const Cell &start, const Cell &goal,
									 std::unordered_map<Cell, Cell> &came_from, std::vector<Cell> &path)
{
	Cell current = goal;
	path.push_back(current);

	do
	{
		current = came_from.at(current);
		path.push_back(current);
	} while (current != start);
	std::reverse(path.begin(), path.end());
}

inline double Planner::heuristic(const Cell &cell1, const Cell &cell2)
{
	double a = fabs((double)cell1.mx_ - (double)cell2.mx_);
	double b = fabs((double)cell1.my_ - (double)cell2.my_);
	return sqrt(a * a + b * b);
}

double Planner::cost_multplier(const Cell &current, const Cell &next) const
{
	if (current.mx_ == next.mx_ || current.my_ == next.my_)
		return 1.0;
	else if (std::abs((int)current.mx_ - (int)next.mx_) == 2 || std::abs((int)current.my_ - (int)next.my_) == 2)
	{
		return 2.2360679775;
	}
	else
		return M_SQRT2; //sqrt(2)
}

void Planner::mapToWorld(unsigned int &mx, unsigned int &my, double &wx, double &wy)
{
	wx = m_originX + (mx + 0.5) * m_resolution;
	wy = m_originY + (my + 0.5) * m_resolution;
}

bool Planner::worldToMap(double &wx, double &wy, unsigned int &mx, unsigned int &my)
{
	if (wx < m_originX || wy < m_originY)
		return false;

	mx = (int)((wx - m_originX) / m_resolution);
	my = (int)((wy - m_originY) / m_resolution);

	if (mx < m_width && my < m_height)
		return true;

	return false;
}

void Planner::cellPathToOutput(std::vector<Cell> &path,
									  std::vector<geometry_msgs::PoseStamped> &plan)
{
	ros::Time plan_time = ros::Time::now();

	if (!path.empty())
	{
		for (int i = 0; i < path.size(); ++i)
		{
			double wx, wy;
			mapToWorld(path[i].mx_, path[i].my_, wx, wy);
			geometry_msgs::PoseStamped pose;
			pose.header.stamp = plan_time;
			pose.header.frame_id = "map";
			pose.pose.position.x = wx;
			pose.pose.position.y = wy;
			pose.pose.position.z = 0.0;
			pose.pose.orientation.x = 0.0;
			pose.pose.orientation.y = 0.0;
			pose.pose.orientation.z = 0.0;
			pose.pose.orientation.w = 1.0;
			plan.push_back(pose);
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "planner");
	ros::NodeHandle l_nh;

	// Class object
	Planner l_pl;

	// Create ROS service
	ros::ServiceServer l_service;
	l_service = l_nh.advertiseService("planner_service", &Planner::PlannerSrv, &l_pl);
	
	ROS_INFO("Clutter Planner Service: RUNNING...");
	ros::spin();

	return 0;
}

