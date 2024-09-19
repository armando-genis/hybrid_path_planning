std::shared_ptr<planner::Node> HybridAstar::reeds_shepp_Path(const std::shared_ptr<planner::Node> &current_Node, const std::shared_ptr<planner::Node> &goal_Node)
{
    auto init_time = std::chrono::system_clock::now();

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
            if (grid_map_.checkCollision(state, next_state_vehicle_poly) || !grid_map_.isPointInBounds(state.gridx, state.gridy))
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

    auto execution_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                              std::chrono::system_clock::now() - init_time)
                              .count();

    cout << "Execution time in reeds shepp: " << execution_time << " ms" << endl;

    return nullptr;
}

//

std::shared_ptr<planner::Node> HybridAstar::reeds_shepp_Path(const std::shared_ptr<planner::Node> &current_Node, const std::shared_ptr<planner::Node> &goal_Node)
{

    auto init_time = std::chrono::system_clock::now();

    double currentX = current_Node->Current_state.x;
    double currentY = current_Node->Current_state.y;
    double currentHeading = current_Node->Current_state.heading;

    double goalX = goal_Node->Current_state.x;
    double goalY = goal_Node->Current_state.y;
    double goalHeading = goal_Node->Current_state.heading;

    double radius = tan(car_data_.maxSteerAngle) / car_data_.wheelBase;

    // Fixing the initialization syntax
    double max_c = 1.0; // Max curvature
    std::vector<PATH> reedsSheppPaths = Reeds_Shepp_Path_.calc_all_paths(currentX, currentY, currentHeading, goalX, goalY, goalHeading, radius, max_c);

    if (reedsSheppPaths.empty())
    {
        return nullptr; // No paths found, return early
    }

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
            if (grid_map_.checkCollision(state, next_state_vehicle_poly) || !grid_map_.isPointInBounds(state.gridx, state.gridy))
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

    auto execution_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                              std::chrono::system_clock::now() - init_time)
                              .count();

    cout << "Execution time in reeds shepp: " << execution_time << " ms" << endl;

    return nullptr; // No valid path found
}

//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//

std::shared_ptr<planner::Node> HybridAstar::reeds_shepp_Path(const std::shared_ptr<planner::Node> &current_Node, const std::shared_ptr<planner::Node> &goal_Node)
{
    auto init_time = std::chrono::system_clock::now();

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
    if (reedsSheppPaths.size() > path_threshold)
    {
        return reeds_shepp_Path_iterative(current_Node, reedsSheppPaths);
    }
    else
    {
        return reeds_shepp_Path_priority_queue(current_Node, reedsSheppPaths);
    }

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
            if (grid_map_.checkCollision(state, next_state_vehicle_poly) || !grid_map_.isPointInBounds(state.gridx, state.gridy))
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

    auto execution_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                              std::chrono::system_clock::now() - init_time)
                              .count();

    cout << "Execution time in reeds shepp: " << execution_time << " ms" << endl;

    return nullptr;
}

// std::shared_ptr<planner::Node> HybridAstar::reeds_shepp_Path(const std::shared_ptr<planner::Node> &current_Node, const std::shared_ptr<planner::Node> &goal_Node)
// {

//     auto init_time = std::chrono::system_clock::now();

//     double currentX = current_Node->Current_state.x;
//     double currentY = current_Node->Current_state.y;
//     double currentHeading = current_Node->Current_state.heading;

//     double goalX = goal_Node->Current_state.x;
//     double goalY = goal_Node->Current_state.y;
//     double goalHeading = goal_Node->Current_state.heading;

//     double radius = tan(car_data_.maxSteerAngle) / car_data_.wheelBase;

//     // Fixing the initialization syntax
//     double max_c = 1.0; // Max curvature
//     std::vector<PATH> reedsSheppPaths = Reeds_Shepp_Path_.calc_all_paths(currentX, currentY, currentHeading, goalX, goalY, goalHeading, radius, max_c);

//     if (reedsSheppPaths.empty())
//     {
//         return nullptr; // No paths found, return early
//     }

//     // Priority queue (min-heap) to store paths by cost
//     std::priority_queue<std::pair<double, PATH>, std::vector<std::pair<double, PATH>>, PathCostCompare> costQueue;

//     // Calculate costs for all paths and add to priority queue
//     for (const auto &path : reedsSheppPaths)
//     {
//         double cost = reeds_Path_Cost(current_Node, &path);
//         costQueue.push(std::make_pair(cost, path));
//     }

//     // Check paths in order of lowest cost
//     while (!costQueue.empty())
//     {
//         auto currentPair = costQueue.top(); // Get the lowest cost path
//         costQueue.pop();

//         const PATH &bestPath = currentPair.second;
//         std::vector<State> traj;

//         // Convert path to trajectory
//         for (size_t i = 0; i < bestPath.x.size(); i++)
//         {
//             State state;
//             state.x = bestPath.x[i];
//             state.y = bestPath.y[i];
//             state.heading = bestPath.yaw[i];

//             // Convert to grid cells
//             auto next_cell = grid_map_.toCellID(state);
//             state.gridx = std::get<0>(next_cell);
//             state.gridy = std::get<1>(next_cell);

//             traj.push_back(state);
//         }

//         // Check for collisions
//         bool has_collision = false;
//         for (const auto &state : traj)
//         {
//             geometry_msgs::msg::Polygon next_state_vehicle_poly = car_data_.getVehicleGeometry_state(state);

//             // Check if the state is inside the boundary and has no collisions
//             if (grid_map_.checkCollision(state, next_state_vehicle_poly) || !grid_map_.isPointInBounds(state.gridx, state.gridy))
//             {
//                 has_collision = true;
//                 break; // Exit if collision is found
//             }
//         }

//         // If no collision is found, return the node with the valid path
//         if (!has_collision)
//         {
//             double cost = currentPair.first;
//             return std::make_shared<planner::Node>(traj.back(), traj, cost, 0, 1, current_Node);
//         }
//     }

//     auto execution_time = std::chrono::duration_cast<std::chrono::milliseconds>(
//                               std::chrono::system_clock::now() - init_time)
//                               .count();

//     cout << "Execution time in reeds shepp: " << execution_time << " ms" << endl;

//     return nullptr; // No valid path found
// }