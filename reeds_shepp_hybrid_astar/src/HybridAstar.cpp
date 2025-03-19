#include "HybridAstar.h"

HybridAstar::HybridAstar(Grid_map grid_map, CarData car_data, double simulationLength, double step_car)
    : grid_map_(grid_map), car_data_(car_data), simulationLength(simulationLength), step_car(step_car)
{
    cout << purple << " ----> Hybrid Astar Initialized <----" << reset << endl;

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

    const size_t width = grid_map_.getWidth();
    const size_t height = grid_map_.getHeight();
    const size_t map_size = width * height;

    static std::vector<double> cost_map_;
    static std::vector<bool> closed_set_;

    // Resize only if needed (first call or map size changed)
    if (cost_map_.size() != map_size)
    {
        cost_map_.resize(map_size);
        closed_set_.resize(map_size);
    }

    std::fill(cost_map_.begin(), cost_map_.end(), std::numeric_limits<double>::infinity());
    std::fill(closed_set_.begin(), closed_set_.end(), false);

    State goal_state = GoalNode->Current_state;
    int goal_index = grid_map_.toCellIndex(goal_state.gridx, goal_state.gridy);

    // Set the goal node cost to zero
    cost_map_[goal_index] = 0.0;

    // Define the 8-connected grid directions
    static const std::vector<std::pair<int, int>> directions = {
        {-1, 0}, {-1, 1}, {0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}};

    // Define a lambda function to calculate movement cost
    // Use direct calculation rather than a predefined array for flexibility
    auto getCost = [](int dx, int dy) -> double
    {
        // Use faster integer comparison for diagonal check
        return (dx == 0 || dy == 0) ? 1.0 : 1.414;
    };

    // Use a bucket queue instead of priority queue
    //  For a 2D grid, costs are typically small integers, making a bucket queue much faster
    //  Scale MAX_COST based on grid size to handle large maps
    const int MAX_COST = std::max(5000, static_cast<int>(width + height));
    std::vector<std::vector<int>> buckets(MAX_COST);

    // Start with bucket 0
    buckets[0].push_back(goal_index);
    int current_bucket = 0;

    // Process the entire map if necessary, with periodic checks
    int processed_cells = 0;
    // Don't artificially limit the number of cells, but check progress periodically
    const int PROGRESS_CHECK_INTERVAL = 70000;

    static int last_goal_x = -1;
    static int last_goal_y = -1;
    static std::vector<double> cached_cost_map_;

    if (last_goal_x == goal_state.gridx && last_goal_y == goal_state.gridy &&
        !cached_cost_map_.empty())
    {
        // Return cached result if goal position hasn't changed
        return cached_cost_map_;
    }

    last_goal_x = goal_state.gridx;
    last_goal_y = goal_state.gridy;

    // OPTIMIZATION 8: Process until completion with periodic progress checks
    while (true)
    {
        // Find the next non-empty bucket
        while (current_bucket < MAX_COST && buckets[current_bucket].empty())
        {
            current_bucket++;
        }

        if (current_bucket >= MAX_COST)
        {
            break; // No more cells to process
        }

        // Process all cells in the current bucket
        std::vector<int> current_cells;
        std::swap(current_cells, buckets[current_bucket]);

        for (int current_index : current_cells)
        {
            // Skip if already processed
            if (closed_set_[current_index])
            {
                continue;
            }

            double current_cost = cost_map_[current_index];
            closed_set_[current_index] = true;
            processed_cells++;

            // Check progress periodically for very large maps
            if (processed_cells % PROGRESS_CHECK_INTERVAL == 0)
            {
                // If we've processed a large number of cells, check if we're reaching diminishing returns
                // This helps with extremely large maps without artificially cutting off processing
                bool sufficient_coverage = true;

                if (grid_map_.isPointInBounds(start_state_.gridx, start_state_.gridy))
                {
                    int start_index = grid_map_.toCellIndex(start_state_.gridx, start_state_.gridy);
                    if (!closed_set_[start_index] && cost_map_[start_index] == std::numeric_limits<double>::infinity())
                    {
                        sufficient_coverage = false;
                    }
                }

                // If we have sufficient coverage or have processed a very large portion, we can stop
                if (sufficient_coverage || processed_cells > map_size * 0.75)
                {
                    break;
                }
            }

            // Compute coordinates only once
            int current_x = current_index % width;
            int current_y = current_index / width;

            for (int dir = 0; dir < 8; dir++)
            {
                int cell_x = current_x + directions[dir].first;
                int cell_y = current_y + directions[dir].second;

                // Skip if out of bounds - combined check
                if (cell_x < 0 || cell_x >= static_cast<int>(width) ||
                    cell_y < 0 || cell_y >= static_cast<int>(height))
                {
                    continue;
                }

                // Compute index directly instead of calling function
                int neighbor_index = cell_y * width + cell_x;

                // Skip if closed or collision
                if (closed_set_[neighbor_index] || grid_map_.isInCollision(cell_x, cell_y))
                {
                    continue;
                }

                // Calculate movement cost dynamically
                double movement_cost = getCost(directions[dir].first, directions[dir].second);

                bool near_obstacle = false;
                // Only check 4 cardinal directions for obstacles (faster than checking all 8)
                static const int dx[4] = {-1, 0, 1, 0};
                static const int dy[4] = {0, -1, 0, 1};

                for (int i = 0; i < 4; i++)
                {
                    int nx = cell_x + dx[i];
                    int ny = cell_y + dy[i];

                    if (nx >= 0 && nx < static_cast<int>(width) &&
                        ny >= 0 && ny < static_cast<int>(height) &&
                        grid_map_.isInCollision(nx, ny))
                    {
                        near_obstacle = true;
                        break;
                    }
                }

                if (near_obstacle)
                {
                    movement_cost += 5.0; // Penalty for being near obstacles
                }

                double new_cost = current_cost + movement_cost;

                // Update if better cost found
                if (new_cost < cost_map_[neighbor_index])
                {
                    cost_map_[neighbor_index] = new_cost;

                    // Add to appropriate bucket (rounded to integer)
                    int bucket_index = std::min(static_cast<int>(new_cost), MAX_COST - 1);
                    buckets[bucket_index].push_back(neighbor_index);

                    // Update current bucket if needed
                    current_bucket = std::min(current_bucket, bucket_index);
                }
            }
        }
    }

    cached_cost_map_ = cost_map_;

    auto end_time = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - init_time).count();
    std::cout << blue << "Execution time holonomic: " << duration << " ms" << reset << std::endl;

    return cost_map_;
}
// main function
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

    goal_map_.clear();

    cout << purple << " ----> Finish <---- " << reset << endl;

    return goal_trajectory;
}
