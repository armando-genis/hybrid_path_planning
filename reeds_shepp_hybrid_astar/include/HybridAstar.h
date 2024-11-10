#ifndef HYBRID_ASTAR_H
#define HYBRID_ASTAR_H

#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "State.h"
#include "Grid_map.h"
#include "CarData.h"
#include "Node.h"
#include "reeds_shepp_path.h"

#include <tuple>
#include <vector>
#include <queue>
#include <cmath>
#include <iostream>
#include <memory>
#include <omp.h>

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

struct PathCostCompare
{
    bool operator()(const std::pair<double, PATH> &lhs, const std::pair<double, PATH> &rhs) const
    {
        return lhs.first > rhs.first; // Compare by cost, for min-heap
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

    // reeds path planning
    std::shared_ptr<planner::Node> reeds_shepp_Path(const std::shared_ptr<planner::Node> &current_Node, const std::shared_ptr<planner::Node> &goal_Node);
    std::shared_ptr<planner::Node> reeds_shepp_Path_priority_queue(const std::shared_ptr<planner::Node> &current_Node, std::vector<PATH> reedsSheppPaths);
    std::shared_ptr<planner::Node> reeds_shepp_Path_iterative(const std::shared_ptr<planner::Node> &current_Node, std::vector<PATH> reedsSheppPaths);
    double reeds_Path_Cost(const std::shared_ptr<planner::Node> &currentNode, const PATH *path);
    // holonomic path planning
    vector<vector<int>> holonomicMotionCommands();
    std::vector<double> holonomicCostsWithObstacles_planning(const std::shared_ptr<planner::Node> &GoalNode);
    std::shared_ptr<planner::HolonomicNode> getNode(int index, vector<std::shared_ptr<planner::HolonomicNode>> &goal_map_);
    inline double eucledianCost(const std::vector<int> &command, const int current_x, const int current_y);

private:
    Grid_map grid_map_;
    CarData car_data_;

    reeds_shepp_path Reeds_Shepp_Path_;

    State start_state_;
    State goal_state_;

    vector<vector<double>> motionCommand;

    // parameter to change to get a better offordable path
    double simulationLength;
    double step_car;

    double reverse = 10.0;
    double directionChange = 250.0;
    double steerAngle = 15.0;
    double steerAngleChange = 20.0;
    double hybridCost = 30.0;

    // colors for the terminal
    string green = "\033[1;32m";
    string red = "\033[1;31m";
    string blue = "\033[1;34m";
    string yellow = "\033[1;33m";
    string purple = "\033[1;35m";
    string reset = "\033[0m";

    const double HEADING_TOLERANCE = 0.05;
};

#endif // HYBRID_ASTAR_H
