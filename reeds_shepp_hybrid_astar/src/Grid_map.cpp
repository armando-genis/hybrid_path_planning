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

    // static_map_orig_.resize(width * height, 0);
    // updateMap();
    // // cout size the new obstacle list
    // cout << "new_obstacle_list_ size: " << new_obstacle_list_.size() << endl;

    // // cout size the new vacancy list
    // cout << "new_vacancy_list_ size: " << new_vacancy_list_.size() << endl;
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
        for (uint32_t j = min_j; j <= max_j; ++j)
        {
            if (map_data_.data[i * width + j] > 0)
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
                    return true; // Collision detected
                }
            }
        }
    }

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