#ifndef HYBRID_ASTAR_H
#define HYBRID_ASTAR_H

#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "State.h"
#include "Grid_map.h"
#include "CarData.h"
#include "dubins_path.h"
#include "Node.h"

#include <tuple>
#include <vector>
#include <queue>
#include <cmath>
#include <iostream>
#include <memory>

using namespace std;

#define PI 3.14159
const double EPSILON = 1e-9;

struct NodeComparator
{
    bool operator()(const std::pair<double, std::shared_ptr<planner::Node>> &lhs,
                    const std::pair<double, std::shared_ptr<planner::Node>> &rhs) const
    {
        return lhs.first > rhs.first;
    }
};

class HybridAstar
{
public:
    HybridAstar(Grid_map grid_map, CarData car_data, double simulationLength, double step_car);
    vector<State> run(State start_state, State goal_state);

    // The obstacle heuristic
    void motionCommands();
    vector<std::shared_ptr<planner::Node>> GetnextNeighbours(const std::shared_ptr<planner::Node> &current_node);
    double PathCost(const std::shared_ptr<planner::Node> &current_node, const vector<double> &motionCommand);

    // dubins path planning
    std::shared_ptr<planner::Node> dubins_Path(const std::shared_ptr<planner::Node> &current_Node, const std::shared_ptr<planner::Node> &goal_Node);
    double dubins_Path_Cost(const std::shared_ptr<planner::Node> &currentNode, const Path *path);
    // holonomic path planning
    vector<vector<int>> holonomicMotionCommands();
    vector<std::shared_ptr<planner::HolonomicNode>> holonomicCostsWithObstacles_planning(const std::shared_ptr<planner::Node> &GoalNode);
    std::shared_ptr<planner::HolonomicNode> getNode(int index, vector<std::shared_ptr<planner::HolonomicNode>> &goal_map_);
    double eucledianCost(const vector<int> &holonomicMotionCommand);

private:
    Grid_map grid_map_;
    CarData car_data_;
    dubins_path Dubins_path;

    State start_state_;
    State goal_state_;

    vector<vector<double>> motionCommand;

    // parameter to change to get a better offordable path
    double simulationLength;
    double step_car;

    double reverse = 10.0;
    double directionChange = 200.0;
    double steerAngle = 15.0;
    double steerAngleChange = 20.0;
    double hybridCost = 30.0;

    // colors for the terminal
    string green = "\033[1;32m";
    string red = "\033[1;31m";
    string blue = "\033[1;34m";
    string reset = "\033[0m";

    const double HEADING_TOLERANCE = 0.05;
};

#endif // HYBRID_ASTAR_H
