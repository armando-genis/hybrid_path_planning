#include "Grid_map.h"

Grid_map::Grid_map(const nav_msgs::msg::OccupancyGrid &map_data)
{
    map_data_ = map_data;
    resolution = map_data.info.resolution;
    originX = map_data.info.origin.position.x;
    originY = map_data.info.origin.position.y;
    width = map_data.info.width;
    height = map_data.info.height;
    Max_origin_x = originX + width * resolution;
    Max_origin_y = originY + height * resolution;

    // // Compute the length of the map in meters
    // double length_x = width * resolution;
    // double length_y = height * resolution;

    // // Compute the center position of the map
    // double center_x = originX + length_x / 2.0;
    // double center_y = originY + length_y / 2.0;

    // // Initialize the GridMap object with "obstacle" and "distance" layers
    // map_ = grid_map::GridMap(std::vector<std::string>{"obstacle", "distance"});
    // map_.setFrameId(map_data.header.frame_id);

    // map_.setGeometry(grid_map::Length(length_x, length_y), resolution, grid_map::Position(center_x, center_y));

    // // Copy data from the occupancy grid to the "obstacle" layer
    // grid_map::Matrix &obstacleData = map_["obstacle"];
    // obstacleData.setConstant(height, width, 0.0); // Initialize with free space (0 = free)

    // // Create OpenCV matrix for visualization (optional)
    // cv::Mat obstacle_map(height, width, CV_8UC1, cv::Scalar(255)); // All cells initialized as free (255)

    // // Loop through the entire occupancy grid to populate the obstacle map
    // for (int i = 0; i < map_data_.data.size(); ++i)
    // {
    //     // Invert row index to flip the image
    //     int row = height - 1 - (i / width); // Flipping the row index
    //     int col = i % width;                // Column index remains the same

    //     // Check if the grid is occupied (based on the threshold)
    //     if (map_data_.data[i] > free_thres_ || map_data_.data[i] < 0)
    //     {
    //         // Mark the cell as occupied in the GridMap (1 = obstacle)
    //         obstacleData(row, col) = 1.0;

    //         // Mark the cell as occupied in OpenCV image (0 = obstacle)
    //         obstacle_map.at<uchar>(row, col) = 0;
    //     }
    //     else
    //     {
    //         // Mark free space (0 = free)
    //         obstacleData(row, col) = 0.0;

    //         // Mark the cell as free in OpenCV image (255 = free)
    //         obstacle_map.at<uchar>(row, col) = 255;
    //     }
    // }

    // // Save the obstacle map as a PNG image using OpenCV
    // cv::imwrite("obstacle_map.png", obstacle_map);

    // Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> binary =
    //     map_.get("obstacle").cast<unsigned char>();

    // // Convert Eigen matrix to OpenCV Mat for saving as an image
    // cv::Mat binary_map(binary.rows(), binary.cols(), CV_8UC1);

    // double resolution_map = map_.getResolution();
    // double half_res = resolution_map / 2.0;

    // // cout in green the resolution
    // cout << green << "Resolution: " << resolution_map << reset << endl;

    // // Populate the OpenCV Mat from Eigen matrix (scaling 0 to 255 for visualization)
    // for (int i = 0; i < binary.rows(); ++i)
    // {
    //     for (int j = 0; j < binary.cols(); ++j)
    //     {
    //         // Align the obstacles similarly as you did in createObstaclePolygon by adjusting to grid center
    //         double x = j * resolution_map + half_res;
    //         double y = i * resolution_map + half_res;

    //         // Set binary map values for distanceTransform, 0 for obstacles and 255 for free space
    //         binary_map.at<uchar>(i, j) = (binary(i, j) == 1) ? 0 : 255;
    //     }
    // }

    // // Step 2: Apply distance transform
    // cv::Mat distance_map_cv;
    // cv::distanceTransform(binary_map, distance_map_cv, cv::DIST_L2, cv::DIST_MASK_PRECISE);

    // // Step 3: Scale the distance map using resolution (optional)
    // grid_map::Matrix &distanceData = map_.get("distance");
    // for (int row = 0; row < distance_map_cv.rows; ++row)
    // {
    //     for (int col = 0; col < distance_map_cv.cols; ++col)
    //     {
    //         distanceData(row, col) = distance_map_cv.at<float>(row, col) * resolution_map; // Apply scaling by resolution
    //     }
    // }

    // // Step 4: Normalize the distance map for visualization
    // cv::Mat distance_map_vis;
    // cv::normalize(distance_map_cv, distance_map_vis, 0, 255, cv::NORM_MINMAX);
    // distance_map_vis.convertTo(distance_map_vis, CV_8UC1); // Convert to 8-bit for saving

    // // Step 5: Save the distance map as an image
    // cv::imwrite("distance_map.png", distance_map_vis);

    // // Debugging: Print out the maximum and minimum values of the distance map
    // double minVal, maxVal;
    // cv::minMaxLoc(distance_map_cv, &minVal, &maxVal);
    // // std::cout << "Raw Distance Transform: Min distance = " << minVal << ", Max distance = " << maxVal << std::endl;
    // // std::cout << "After scaling: Min distance = " << minVal * resolution << ", Max distance = " << maxVal * resolution << std::endl;
}

Grid_map::~Grid_map()
{
}

void Grid_map::updateMap()
{
    // Loop through the entire map data
    for (int i = 0; i < static_map_orig_.size(); ++i)
    {
        // Check if the grid is occupied (based on the threshold) and was previously unoccupied
        if ((map_data_.data[i] > free_thres_ || map_data_.data[i] < 0) && static_map_orig_[i] == 0)
        {
            // Mark the grid as occupied and add it to the obstacle list
            static_map_orig_[i] = 1;
            new_obstacle_list_.emplace_back(i);
        }
        // Check if the grid is now free but was previously occupied
        else if (map_data_.data[i] < free_thres_ && static_map_orig_[i] == 1)
        {
            // Mark the grid as free and add it to the vacancy list
            static_map_orig_[i] = 0;
            new_vacancy_list_.emplace_back(i);
        }
        // Continue if the grid's state remains unchanged
    }
}

bool Grid_map::checkCollision(const State &state, const geometry_msgs::msg::Polygon &vehicle_poly_state)
{
    auto init_time = std::chrono::system_clock::now();

    obstacle_polys.clear(); // Clear the obstacle polygons
    // Define the 5 meter offset
    double offset = 5.0;

    // Define the bounds for the region to check
    double min_x = std::max(originX, state.x - offset);
    double max_x = std::min(originX + width * resolution, state.x + offset);
    double min_y = std::max(originY, state.y - offset);
    double max_y = std::min(originY + height * resolution, state.y + offset);

    // Convert bounds to grid indices
    uint32_t min_i = std::max(0, static_cast<int>((min_y - originY) / resolution));
    uint32_t max_i = std::min(static_cast<int>(height) - 1, static_cast<int>((max_y - originY) / resolution));
    uint32_t min_j = std::max(0, static_cast<int>((min_x - originX) / resolution));
    uint32_t max_j = std::min(static_cast<int>(width) - 1, static_cast<int>((max_x - originX) / resolution));

    // Check each relevant cell in the occupancy grid
    for (uint32_t i = min_i; i <= max_i; ++i)
    {
        int base_index = i * width;
        for (uint32_t j = min_j; j <= max_j; ++j)
        {
            if (map_data_.data[base_index + j] > 0)
            {
                double x = originX + j * resolution;
                double y = originY + i * resolution;
                geometry_msgs::msg::Polygon poly = createObstaclePolygon(x, y, resolution);
                // log the obstacle polygon
                // for (const auto &point : poly.points)
                // {
                //     cout << "x: " << point.x << " y: " << point.y << endl;
                // }
                obstacle_polys.push_back(poly);

                if (collision_checker.check_collision(vehicle_poly_state, poly))
                {
                    auto end_time = std::chrono::system_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - init_time).count();
                    cout << purple << "--> Execution time checl collision: " << duration << " ms" << reset << endl;
                    return true; // Collision detected
                }
            }
        }
    }

    auto end_time = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - init_time).count();
    cout << purple << "--> Execution time checl collision: " << duration << " ms" << reset << endl;

    return false; // No collision detected
}

geometry_msgs::msg::Polygon Grid_map::createObstaclePolygon(double x, double y, double resolution) const
{
    geometry_msgs::msg::Polygon obstacle_poly;
    double half_res = resolution / 2.0;

    // Adjust (x, y) to ensure the center of the polygon aligns with the grid cell
    x += resolution / 2;
    y += resolution / 2;

    geometry_msgs::msg::Point32 p1, p2, p3, p4;
    p1.x = x - half_res;
    p1.y = y - half_res;
    p2.x = x + half_res;
    p2.y = y - half_res;
    p3.x = x + half_res;
    p3.y = y + half_res;
    p4.x = x - half_res;
    p4.y = y + half_res;

    obstacle_poly.points.push_back(p1);
    obstacle_poly.points.push_back(p2);
    obstacle_poly.points.push_back(p3);
    obstacle_poly.points.push_back(p4);
    obstacle_poly.points.push_back(p1); // Close the polygon

    return obstacle_poly;
}

std::tuple<int, int> Grid_map::toCellID(State state_)
{
    int cell_x = static_cast<int>((state_.x - originX) / resolution);
    int cell_y = static_cast<int>((state_.y - originY) / resolution);

    return std::make_tuple(cell_x, cell_y);
}

int Grid_map::toCellIndex(int x, int y)
{
    // where i is the row (y) and j is the column (x)
    return y * width + x;
}

bool Grid_map::isPointInBounds(int x, int y)
{
    return x >= 0 && x < width && y >= 0 && y < height;
}

bool Grid_map::isInCollision(int x, int y)
{
    if (!isPointInBounds(x, y))
    {
        return true; // If out of bounds, consider it as a collision.
    }
    int index = toCellIndex(x, y);
    return map_data_.data[index] > 0; // A value greater than 0 indicates an obstacle.
}

// ===================== NEW METHOD =====================

void Grid_map::setcarData(CarData car_data)
{
    car_data_ = car_data;
}

double Grid_map::getObstacleDistance(const Eigen::Vector2d &pos) const
{
    if (isInside(pos))
    {
        return map_.atPosition("distance", pos, grid_map::InterpolationMethods::INTER_LINEAR);
    }
    else
    {
        return 0.0; // Return 0.0 if the point is outside the grid map bounds
    }
}

bool Grid_map::isInside(const Eigen::Vector2d &pos) const
{
    return map_.isInside(pos); // Check if the position is inside the map bounds
}

bool Grid_map::isSingleStateCollisionFree(const State &current)
{
    auto init_time = std::chrono::system_clock::now();

    // Get the vehicle footprint as circles in global coordinates
    std::vector<Circle> footprint = car_data_.getCircles(current);

    // Loop through each circle in the footprint
    for (const auto &circle_itr : footprint)
    {
        // Create a position based on the circle's center
        Eigen::Vector2d pos(circle_itr.x, circle_itr.y);

        // cout the position in blue
        cout << blue << "Position: " << pos.transpose() << reset << endl;
        // cout the radius in blue
        cout << blue << "Radius: " << circle_itr.r << reset << endl;

        // Check if the circle is inside the map bounds
        if (isInside(pos))
        {
            // Get the clearance (distance to nearest obstacle)
            double clearance = getObstacleDistance(pos);

            // cout the clearance in yellow
            cout << yellow << "Clearance: " << clearance << reset << endl;

            // If the clearance is less than the circle's radius, it means a collision
            if (clearance < circle_itr.r)
            {
                cout << red << "Collision detected by clearance < circle_itr.r" << reset << endl;
                auto end_time = std::chrono::system_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - init_time).count();
                cout << purple << "--> Execution time check collision new version: " << duration << " ms" << reset << endl;

                return false; // Collision detected
            }
        }
        else
        {
            // If out of bounds, consider it a collision
            cout << red << "Collision detected by out of bounds" << reset << endl;
            auto end_time = std::chrono::system_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - init_time).count();
            cout << purple << "--> Execution time check collision new version: " << duration << " ms" << reset << endl;

            return false;
        }
    }

    auto end_time = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - init_time).count();
    cout << purple << "--> Execution time check collision new version: " << duration << " ms" << reset << endl;

    // No collision detected after checking all circles
    return true;
}

bool Grid_map::isSingleStateCollisionFreeImproved(const State &current)
{
    // Get the bounding circle for the vehicle in global coordinates
    Circle bounding_circle = car_data_.getBoundingCircle(current);

    // Create a position based on the bounding circle's center
    Eigen::Vector2d pos(bounding_circle.x, bounding_circle.y);

    // Check if the bounding circle is inside the map bounds
    if (isInside(pos))
    {
        // Get the clearance (distance to nearest obstacle)
        double clearance = getObstacleDistance(pos);

        // If the clearance is less than the radius of the bounding circle, we need to do detailed checks
        if (clearance < bounding_circle.r)
        {
            // Perform detailed collision checking with individual footprint circles
            return isSingleStateCollisionFree(current);
        }
        else
        {
            // No collision if clearance is larger than the bounding circle's radius
            return true;
        }
    }
    else
    {
        // If out of bounds, consider it a collision
        return false;
    }
}
