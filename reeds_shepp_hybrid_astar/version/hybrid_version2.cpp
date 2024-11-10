#include "HybridAstar.h"

HybridAstar::HybridAstar(Grid_map grid_map, CarData car_data, double simulationLength, double step_car)
    : grid_map_(grid_map), car_data_(car_data), simulationLength(simulationLength), step_car(step_car)
{
    cout << purple << "--Hybrid Astar Initialized--" << reset << endl;

    // Step 2: Generate motion commands
    motionCommands();

} // Initialize car_data_

// ===================== The obstacle heuristic =============================
void HybridAstar::motionCommands()
{
    int direction = 1;
    for (double i = car_data_.maxSteerAngle; i >= -car_data_.maxSteerAngle; i -= car_data_.steer_step)
    {
        // fist data: steering angle, second data: direction
        motionCommand.push_back({i, static_cast<double>(direction)});
        motionCommand.push_back({i, -static_cast<double>(direction)});
    }
}

vector<std::shared_ptr<planner::Node>> HybridAstar::GetnextNeighbours(const std::shared_ptr<planner::Node> &current_node)
{
    vector<std::shared_ptr<planner::Node>> Neighbour_nodes;
    State current_state = current_node->Current_state;
    State initial_state = current_state;

    for (const auto &command : motionCommand)
    {
        vector<State> traj;
        for (int i = 0; i < simulationLength; ++i)
        {
            State next_state = car_data_.getVehicleStep(current_state, command[0], command[1], step_car);
            // to grid cell ints the next state
            auto next_cell = grid_map_.toCellID(next_state);
            next_state.gridx = get<0>(next_cell);
            next_state.gridy = get<1>(next_cell);

            traj.push_back(next_state);
            current_state = next_state;
        }

        // Check for collision for each point in the trajectory
        bool has_collision = false;
        for (const auto &state : traj)
        {
            geometry_msgs::msg::Polygon next_state_vehicle_poly = car_data_.getVehicleGeometry_state(state);

            // ===============================> importa add a function to cheack if the state is inside the boundary of the map
            // if (grid_map_.checkCollision(state, next_state_vehicle_poly) || !grid_map_.isPointInBounds(state.gridx, state.gridy))
            if (grid_map_.isSingleStateCollisionFree(state))
            {
                // If any point in the trajectory collides, mark the collision and break out of the loop
                has_collision = true;
                break; // Stop checking this trajectory
            }
        }

        // If no collision was detected, add the entire trajectory to next_states
        if (!has_collision)
        {
            // cost calculation for the node
            double cost = PathCost(current_node, command);
            // cout << blue << "--->motion Command: " << command[0] << " cost: " << cost << " command: " << command[1] << reset << endl;
            auto next_node = std::make_shared<planner::Node>(traj.back(), traj, cost, command[0], command[1], current_node);
            Neighbour_nodes.push_back(next_node);
        }

        traj.clear();
        current_state = initial_state;
    }

    // cout << "Neighbour_nodes size: " << Neighbour_nodes.size() << endl;

    return Neighbour_nodes;
}

double HybridAstar::PathCost(const std::shared_ptr<planner::Node> &current_node, const vector<double> &motionCommand)
{
    double cost = current_node->Cost_path;

    // Add cost based on movement direction
    if (motionCommand[1] == 1) // Forward
    {
        cost += simulationLength; // Add the simulation length as cost
    }
    else // Reverse
    {
        cost += simulationLength * reverse; // Multiply by reverse cost factor
    }

    // Add cost for direction changes
    if (current_node->direction != motionCommand[1])
    {
        cost += directionChange; // Add a cost for changing direction
    }

    // The larger the steering angle, the higher the cost
    cost += fabs(motionCommand[0]) * steerAngle;

    cost += fabs(motionCommand[0] - current_node->steeringAngle) * (steerAngleChange);

    return cost;
}

// ===================== The reeds path calculation =============================
std::shared_ptr<planner::Node> HybridAstar::reeds_shepp_Path(const std::shared_ptr<planner::Node> &current_Node, const std::shared_ptr<planner::Node> &goal_Node)
{
    // Get the current and goal states
    double currentX = current_Node->Current_state.x;
    double currentY = current_Node->Current_state.y;
    double currentHeading = current_Node->Current_state.heading;

    double goalX = goal_Node->Current_state.x;
    double goalY = goal_Node->Current_state.y;
    double goalHeading = goal_Node->Current_state.heading;

    double radius = tan(car_data_.maxSteerAngle) / car_data_.wheelBase;

    double max_c = 1; // max curvature
    std::vector<PATH> reedsSheppPaths = Reeds_Shepp_Path_.calc_all_paths(currentX, currentY, currentHeading, goalX, goalY, goalHeading, radius, max_c);

    if (reedsSheppPaths.empty())
    {
        return nullptr; // No paths found, return early
    }

    const std::size_t path_threshold = 5;
    // if the path is more than 5 use priority queue because it is  more optimal  wigh large paths
    if (reedsSheppPaths.size() > path_threshold)
    {
        // cout << yellow << "Using priority queue" << reset << endl;
        return reeds_shepp_Path_priority_queue(current_Node, reedsSheppPaths);
    }
    else
    {
        // cout << yellow << "Using iterative" << reset << endl;
        return reeds_shepp_Path_iterative(current_Node, reedsSheppPaths);
    }
}

std::shared_ptr<planner::Node> HybridAstar::reeds_shepp_Path_iterative(const std::shared_ptr<planner::Node> &current_Node, std::vector<PATH> reedsSheppPaths)
{

    // Initialize variables for tracking the best (shortest and valid) path
    double minCost = std::numeric_limits<double>::infinity();
    std::shared_ptr<planner::Node> bestNode = nullptr;

    // Loop through each Reeds-Shepp path to calculate cost and check for collisions
    for (const auto &path : reedsSheppPaths)
    {
        // Convert the path to a trajectory of states
        std::vector<State> traj;
        for (size_t i = 0; i < path.x.size(); ++i)
        {
            State state;
            state.x = path.x[i];
            state.y = path.y[i];
            state.heading = path.yaw[i];

            // Convert to grid cell
            auto next_cell = grid_map_.toCellID(state);
            state.gridx = std::get<0>(next_cell);
            state.gridy = std::get<1>(next_cell);

            traj.push_back(state);
        }

        // Check for collisions along the trajectory
        bool has_collision = false;
        for (const auto &state : traj)
        {
            geometry_msgs::msg::Polygon next_state_vehicle_poly = car_data_.getVehicleGeometry_state(state);

            // Check for boundary and collision in the grid map
            // if (grid_map_.checkCollision(state, next_state_vehicle_poly) || !grid_map_.isPointInBounds(state.gridx, state.gridy))
            if (grid_map_.isSingleStateCollisionFree(state))
            {
                has_collision = true;
                break;
            }
        }

        // If no collision is found, calculate the cost
        if (!has_collision)
        {
            double cost = reeds_Path_Cost(current_Node, &path);

            // If the cost is lower than the current minimum, store this path as the best path
            if (cost < minCost)
            {
                minCost = cost;

                // Create the new planner::Node with the valid trajectory and cost
                bestNode = std::make_shared<planner::Node>(traj.back(), traj, cost, 0, 1, current_Node);
            }
        }
    }

    // Return the best node if found, otherwise return nullptr
    if (bestNode)
    {
        return bestNode;
    }

    return nullptr;
}

std::shared_ptr<planner::Node> HybridAstar::reeds_shepp_Path_priority_queue(const std::shared_ptr<planner::Node> &current_Node, std::vector<PATH> reedsSheppPaths)
{

    // Priority queue (min-heap) to store paths by cost
    std::priority_queue<std::pair<double, PATH>, std::vector<std::pair<double, PATH>>, PathCostCompare> costQueue;

    // Calculate costs for all paths and add to priority queue
    for (const auto &path : reedsSheppPaths)
    {
        double cost = reeds_Path_Cost(current_Node, &path);
        costQueue.push(std::make_pair(cost, path));
    }

    // Check paths in order of lowest cost
    while (!costQueue.empty())
    {
        auto currentPair = costQueue.top(); // Get the lowest cost path
        costQueue.pop();

        const PATH &bestPath = currentPair.second;
        std::vector<State> traj;

        // Convert path to trajectory
        for (size_t i = 0; i < bestPath.x.size(); i++)
        {
            State state;
            state.x = bestPath.x[i];
            state.y = bestPath.y[i];
            state.heading = bestPath.yaw[i];

            // Convert to grid cells
            auto next_cell = grid_map_.toCellID(state);
            state.gridx = std::get<0>(next_cell);
            state.gridy = std::get<1>(next_cell);

            traj.push_back(state);
        }

        // Check for collisions
        bool has_collision = false;
        for (const auto &state : traj)
        {
            geometry_msgs::msg::Polygon next_state_vehicle_poly = car_data_.getVehicleGeometry_state(state);

            // Check if the state is inside the boundary and has no collisions
            // if (grid_map_.checkCollision(state, next_state_vehicle_poly) || !grid_map_.isPointInBounds(state.gridx, state.gridy))
            if (grid_map_.isSingleStateCollisionFree(state))
            {
                has_collision = true;
                break; // Exit if collision is found
            }
        }

        // If no collision is found, return the node with the valid path
        if (!has_collision)
        {
            double cost = currentPair.first;
            return std::make_shared<planner::Node>(traj.back(), traj, cost, 0, 1, current_Node);
        }
    }

    return nullptr; // No valid path found
}

double HybridAstar::reeds_Path_Cost(const std::shared_ptr<planner::Node> &currentNode, const PATH *path)
{
    double cost = currentNode->Cost_path;

    // Distance cost: iterate over each length in the path
    for (const auto &length : path->lengths)
    {
        if (length >= 0)
        {
            cost += 1.0; // Add a constant for positive lengths
        }
        else
        {
            cost += std::fabs(length) * reverse; // Use reverse cost for negative lengths
        }
    }

    // Direction change cost: check if consecutive segments have a direction change
    for (size_t i = 0; i < path->lengths.size() - 1; ++i)
    {
        if (path->lengths[i] * path->lengths[i + 1] < 0)
        {
            cost += directionChange; // Add cost for direction change
        }
    }

    // Steering Angle Cost: iterate over the ctypes (curve types)
    for (const auto &mode : path->ctypes)
    {
        // Check types which are not straight line ('S')
        if (mode != "S")
        {
            cost += car_data_.maxSteerAngle * steerAngle; // Add steering cost for non-straight segments
        }
    }

    return cost;
}

// ===============holonomic path planning =============================
vector<vector<int>> HybridAstar::holonomicMotionCommands()
{
    return {{-1, 0}, {-1, 1}, {0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}};
}

std::shared_ptr<planner::HolonomicNode> HybridAstar::getNode(int index, vector<std::shared_ptr<planner::HolonomicNode>> &goal_map_)
{
    return goal_map_[index];
}

// ============================= Hybrid A* ============================
inline double HybridAstar::eucledianCost(const std::vector<int> &command, const int current_x, const int current_y)
{
    // Since the movement is in 8 directions, the cost is either 1.0 or sqrt(2)
    // if (abs(command[0]) + abs(command[1]) == 1)
    //     return 1.0; // Orthogonal movement
    // else
    //     return 1.41421356237; // Diagonal movement (sqrt(2))

    double distance_cost = std::hypot(command[0], command[1]);

    double obstacle_penalty = 0.0;

    for (int dx = -1; dx <= 1; ++dx)
    {
        for (int dy = -1; dy <= 1; ++dy)
        {
            int neighbor_x = current_x + dx;
            int neighbor_y = current_y + dy;

            // Increase penalty if neighbor cell is in collision
            if (grid_map_.isInCollision(neighbor_x, neighbor_y))
            {
                obstacle_penalty += 10.0;
            }
        }
    }

    // Total cost is the distance cost plus the obstacle penalty
    return distance_cost + obstacle_penalty;
}

std::vector<double> HybridAstar::holonomicCostsWithObstacles_planning(const std::shared_ptr<planner::Node> &GoalNode)
{
    auto init_time = std::chrono::system_clock::now();

    size_t width = grid_map_.getWidth();
    size_t height = grid_map_.getHeight();
    size_t map_size = width * height;

    // Initialize the cost map and closed set
    std::vector<double> cost_map_(map_size, std::numeric_limits<double>::infinity());
    std::vector<bool> closed_set_(map_size, false);

    State goal_state = GoalNode->Current_state;
    int goal_index = grid_map_.toCellIndex(goal_state.gridx, goal_state.gridy);

    // Set the goal node cost to zero
    cost_map_[goal_index] = 0.0;

    // Generate holonomic motion commands (8-directional movement)
    const std::vector<std::vector<int>> &holonomicMotionCommand = holonomicMotionCommands();

    // Create a priority queue for the open set
    // The priority queue stores pairs of (cost, index)
    using PQElement = std::pair<double, int>;
    std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> openset_;

    // Push the goal node into the open set
    openset_.push(std::make_pair(0.0, goal_index));

    int count = 0;

    while (!openset_.empty())
    {
        count++;

        // Get the node with the lowest cost
        auto [current_cost, current_index] = openset_.top();
        openset_.pop();

        // If this node has already been processed, skip it
        if (closed_set_[current_index])
            continue;

        // Mark the node as closed
        closed_set_[current_index] = true;

        // Get the current node's grid coordinates
        int current_x = current_index % width;
        int current_y = current_index / width;

        // For each holonomic motion command
        for (const auto &command : holonomicMotionCommand)
        {
            int cell_x = current_x + command[0];
            int cell_y = current_y + command[1];

            // Check if the neighbor is within bounds
            if (cell_x < 0 || cell_x >= static_cast<int>(width) || cell_y < 0 || cell_y >= static_cast<int>(height))
                continue;

            // Get the index of the neighboring cell
            int neighbor_index = grid_map_.toCellIndex(cell_x, cell_y);

            // If the neighbor is in collision or already closed, skip it
            if (grid_map_.isInCollision(cell_x, cell_y) || closed_set_[neighbor_index])
                continue;

            // Calculate the new cost to reach the neighbor
            double movement_cost = eucledianCost(command, current_x, current_y);
            double newCost = current_cost + movement_cost;


            // If the new cost is lower, update the cost map and push into the open set
            if (newCost < cost_map_[neighbor_index])
            {
                cost_map_[neighbor_index] = newCost;
                openset_.push(std::make_pair(newCost, neighbor_index));
            }
        }
    }

    auto end_time = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - init_time).count();
    std::cout << "Execution time holonomic: " << duration << " ms" << std::endl;

    return cost_map_;
}

vector<State> HybridAstar::run(State start_state, State goal_state)
{
    // Initialize the time
    auto init_time = std::chrono::system_clock::now();

    vector<State> goal_trajectory;

    start_state_ = start_state;
    goal_state_ = goal_state;

    // Step 1: Compute Grid Index for Start and Goal Node
    auto start_cell = grid_map_.toCellID(start_state);
    auto goal_cell = grid_map_.toCellID(goal_state);

    start_state_.gridx = get<0>(start_cell);
    start_state_.gridy = get<1>(start_cell);
    int start_index = grid_map_.toCellIndex(start_state_.gridx, start_state_.gridy);

    goal_state_.gridx = get<0>(goal_cell);
    goal_state_.gridy = get<1>(goal_cell);

    // Step 3: Create start and goal Node and Trajectories
    vector<State> start_traj = {start_state_};
    vector<State> goal_traj = {goal_state_};

    auto startNode = std::make_shared<planner::Node>(start_state_, start_traj, 0, 0, 1, std::weak_ptr<planner::Node>());
    auto goalNode = std::make_shared<planner::Node>(goal_state_, goal_traj, 0, 0, 1, std::weak_ptr<planner::Node>());

    // Step 4: Find Holonomic Heuristic
    auto goal_map_ = holonomicCostsWithObstacles_planning(goalNode);

    // Step 5: Create Open and Closed sets using State as key
    std::unordered_map<State, std::shared_ptr<planner::Node>, StateHash> openSet;
    std::unordered_map<State, std::shared_ptr<planner::Node>, StateHash> closedSet;

    // Priority queue for managing the nodes (C++ priority queue)
    std::priority_queue<
        std::pair<double, std::shared_ptr<planner::Node>>,
        std::vector<std::pair<double, std::shared_ptr<planner::Node>>>,
        NodeComparator>
        costQueue;

    // Step 6: Add the start node to the open set
    openSet[start_state_] = startNode;
    costQueue.push({startNode->Cost_path + hybridCost * goal_map_[start_index], startNode});


    // Step 6.5: Initialize a counter for the number of iterations
    int iterations = 0;

    // Step 7: Loop until the open set is empty
    while (!costQueue.empty())
    {
        iterations++;
        if (iterations > 3500)
        {
            cout << red << "Max iterations reached!" << reset << endl;
            break;
        }

        // Get the node with the lowest cost from the priority queue
        State currentState = costQueue.top().second->Current_state;
        costQueue.pop();

        // Get the current node from the open set
        auto currentNode = openSet[currentState];

        // Check if currentNode is in the closed set
        if (closedSet.find(currentState) != closedSet.end())
        {
            continue;
        }

        // Move currentNode from openSet to closedSet
        openSet.erase(currentState);
        closedSet[currentState] = currentNode;

        // Try to find a Reeds-Shepp path to the goal from the current node
        auto dNode = reeds_shepp_Path(currentNode, goalNode);

        // If a valid Reeds-Shepp path is found, exit the loop
        if (dNode)
        {
            cout << green << "Reeds-Shepp path found" << reset << endl;
            closedSet[dNode->Current_state] = dNode;
            goalNode = dNode;
            break;
        }

        double heading_difference = fabs(currentState.heading - goal_state_.heading);

        if (heading_difference > PI)
        {
            heading_difference = fabs(heading_difference - 2 * PI);
        }

        // Goal check: If we've reached the goal
        if (currentState.gridx == goal_state_.gridx &&
            currentState.gridy == goal_state_.gridy &&
            heading_difference <= HEADING_TOLERANCE)
        {
            goalNode = currentNode;
            break;
        }

        // Get neighboring nodes by simulating motion
        auto neighbors = GetnextNeighbours(currentNode);


        for (auto &neighbor : neighbors)
        {
            // Get the neighbor's state
            State neighborState = neighbor->Current_state;
            // Skip if this neighbor is already in the closed set
            if (closedSet.find(neighborState) != closedSet.end())
            {
                continue;
            }

            // Calculate the neighbor's cost
            int neighbor_index = grid_map_.toCellIndex(neighborState.gridx, neighborState.gridy);

            // Ensure that the neighbor_index is within bounds
            if (neighbor_index < 0 || neighbor_index >= static_cast<int>(goal_map_.size()))
            {
                continue; // Skip if the index is out of bounds
            }

            double neighborCost = neighbor->Cost_path + hybridCost * goal_map_[neighbor_index];

            // If neighbor is not in the open set, or the new cost is lower, add/update it
            if (openSet.find(neighborState) == openSet.end() || neighbor->Cost_path < openSet[neighborState]->Cost_path)
            {
                openSet[neighborState] = neighbor;
                costQueue.push({neighborCost, neighbor});
            }
        }
    }

    // Backtrack the path
    goal_trajectory.clear(); // Clear any existing trajectory
    cout << blue << "Iterations: " << iterations << reset << endl;

    // Use goalNode for backtracking, which now includes Reeds-Shepp if found
    auto backtrackNode = goalNode;
    while (backtrackNode)
    {
        goal_trajectory.insert(goal_trajectory.end(), backtrackNode->Trajectory.rbegin(), backtrackNode->Trajectory.rend());
        if (auto parentPtr = backtrackNode->Parent.lock())
        {
            backtrackNode = parentPtr;
        }
        else
        {
            break;
        }
    }

    // Reverse the trajectory to start from the beginning (start node)
    std::reverse(goal_trajectory.begin(), goal_trajectory.end());

    auto end_time = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - init_time).count();
    cout << blue << "Execution time: " << duration << " ms" << reset << endl;

    // Memory release: clear openSet and closedSet
    openSet.clear();
    closedSet.clear();
    while (!costQueue.empty())
    {
        costQueue.pop();
    }

    cout << blue << " ----> Finish <---- " << reset << endl;

    return goal_trajectory;
}







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
