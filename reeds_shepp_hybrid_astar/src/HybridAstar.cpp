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
double HybridAstar::eucledianCost(const vector<int> &holonomicMotionCommand, const State &current_state)
{
    double distance_cost = std::hypot(holonomicMotionCommand[0], holonomicMotionCommand[1]);

    double obstacle_penalty = 0.0;

    for (int dx = -1; dx <= 1; ++dx)
    {
        for (int dy = -1; dy <= 1; ++dy)
        {
            int neighbor_x = current_state.gridx + dx;
            int neighbor_y = current_state.gridy + dy;

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

vector<std::shared_ptr<planner::HolonomicNode>> HybridAstar::holonomicCostsWithObstacles_planning(const std::shared_ptr<planner::Node> &GoalNode)
{

    auto init_time = std::chrono::system_clock::now();
    // Get the goal state. This already has the grid coordinates as gridx and gridy
    std::vector<std::shared_ptr<planner::HolonomicNode>> goal_map_;

    State goal_state = GoalNode->Current_state;
    int goal_index = grid_map_.toCellIndex(goal_state.gridx, goal_state.gridy);

    auto gNode = std::make_shared<planner::HolonomicNode>(goal_state, 0, std::weak_ptr<planner::HolonomicNode>());

    // resize goal map as map
    goal_map_.resize(grid_map_.getWidth() * grid_map_.getHeight());

    // Assign shared_ptr instances to all elements
    for (size_t y = 0; y < grid_map_.getHeight(); y++)
    {
        for (size_t x = 0; x < grid_map_.getWidth(); x++)
        {
            State state;
            state.gridx = x;
            state.gridy = y;
            int index = grid_map_.toCellIndex(x, y);
            goal_map_[index] = std::make_shared<planner::HolonomicNode>(state, 1000, std::weak_ptr<planner::HolonomicNode>());
        }
    }

    // Generate holonomic motion commands (8-directional movement)
    std::vector<std::vector<int>> holonomicMotionCommand = holonomicMotionCommands();

    int goal_cost_max_ = 0;

    // set the goal node to the goal map
    goal_map_[goal_index] = gNode;

    // Create a priority queue for the open set
    std::priority_queue<std::shared_ptr<planner::HolonomicNode>, std::vector<std::shared_ptr<planner::HolonomicNode>>, planner::NodeGreater> openset_;

    // push start node into open set
    openset_.push(getNode(goal_index, goal_map_));

    int count = 0;
    // int max_iterations = grid_map_.getWidth() * grid_map_.getHeight();
    // cout << "max iterations: " << max_iterations << endl;

    while (!openset_.empty())
    {
        count++;

        // get the node ptr with lowest cost
        auto current_Node = openset_.top();
        // remove the node ptr from openset
        openset_.pop();
        // get the cost of the current node
        double current_cost = current_Node->cost;
        // for to get the neighbor
        for (const auto &command : holonomicMotionCommand)
        {
            int cell_x = current_Node->Current_state.gridx + command[0];
            int cell_y = current_Node->Current_state.gridy + command[1];

            // check if the neighbor is inside the boundary of the map and not in collision
            if (!grid_map_.isInCollision(cell_x, cell_y))
            {
                // Calculate the new cost to reach the neighbor
                double newCost = current_cost + eucledianCost(command, current_Node->Current_state);
                // Get the index of the neighboring cell
                int current_index = grid_map_.toCellIndex(cell_x, cell_y);
                auto neigh_node_ptr = getNode(current_index, goal_map_);
                // Update the neighbor's cost if the new cost is lower
                if (neigh_node_ptr->cost > newCost)
                {
                    neigh_node_ptr->cost = newCost;
                    openset_.push(neigh_node_ptr);

                    // Update the max goal cost if necessary
                    goal_cost_max_ = std::max(goal_cost_max_, static_cast<int>(neigh_node_ptr->cost));
                }
            }
        }
    }
    // cout << blue << "goal cost max: " << goal_cost_max_ << reset << endl;
    // cout << blue << "count: " << count << reset << endl;

    auto end_time = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - init_time).count();
    cout << blue << "Execution time holomonic: " << duration << " ms" << reset << endl;

    return goal_map_;
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
    costQueue.push({startNode->Cost_path + hybridCost * goal_map_[start_index]->cost, startNode});

    // Use max cost calculation can not be efficient
    // costQueue.push({std::max(startNode->Cost_path, hybridCost * goal_map_[start_index]->cost), startNode});

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

        // Try to find a Dubins path to the goal from the current node
        auto dNode = reeds_shepp_Path(currentNode, goalNode);

        // If a valid Dubins path is found, exit the loop
        if (dNode)
        {
            cout << green << "Reeds shepp path found" << reset << endl;
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

        // cout << red << "===============================" << reset << endl;

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

            double neighborCost = neighbor->Cost_path + hybridCost * goal_map_[neighbor_index]->cost;

            // Use max cost calculation can not be efficient
            // double neighborCost = std::max(neighbor->Cost_path, hybridCost * goal_map_[neighbor_index]->cost);

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

    // Use goalNode for backtracking, which now includes Dubins if found
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