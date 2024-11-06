#ifndef GRID_MAP_H
#define GRID_MAP_H

#include <grid_map_core/grid_map_core.hpp>
#include <opencv2/opencv.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include "CarData.h"
#include <Eigen/Dense>
#include <iostream>
// #include "eigen2cv.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include "sat_collision_checker.h"
#include "State.h"
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <vector>
#include <iostream>
#include <chrono>

using namespace std;

class Grid_map
{
private:
    /* data */
    nav_msgs::msg::OccupancyGrid map_data_;
    double resolution;
    double originX;
    double originY;
    unsigned int width;
    unsigned int height;
    double Max_origin_x;
    double Max_origin_y;
    double free_thres_ = 20;

    // static map layer
    vector<int8_t> static_map_orig_;
    // downsampled map layer
    vector<int8_t> static_map_;

    // list of index of new occupied grid
    vector<int> new_obstacle_list_;
    // list of index of new vacant grid
    vector<int> new_vacancy_list_;

    CarData car_data_;

    // colors for the terminal
    string green = "\033[1;32m";
    string red = "\033[1;31m";
    string blue = "\033[1;34m";
    string yellow = "\033[1;33m";
    string purple = "\033[1;35m";
    string reset = "\033[0m";

    grid_map::GridMap map_;

    geometry_msgs::msg::Polygon obstacle_poly; // Obstacle polygon

    fop::SATCollisionChecker collision_checker; // Collision checker

    std::vector<geometry_msgs::msg::Polygon> obstacle_polys; // Store multiple obstacle polygons

    geometry_msgs::msg::Polygon createObstaclePolygon(double x, double y, double resolution) const;
    void updateMap();

    cv::Mat eigen2cv(const Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> &matrix)
    {
        cv::Mat mat(matrix.rows(), matrix.cols(), CV_8UC1, (void*)matrix.data(), matrix.outerStride());
        return mat.clone(); // Clone to ensure the data is copied
    }

    template <typename T>
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> cv2eigen(const cv::Mat &mat)
    {
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> matrix;
        cv::cv2eigen(mat, matrix);
        return matrix;
    }

public:
    Grid_map(const nav_msgs::msg::OccupancyGrid &map_data);
    ~Grid_map();

    double getResolution() { return resolution; }
    double getOriginX() { return originX; }
    double getOriginY() { return originY; }
    unsigned int getWidth() { return width; }
    unsigned int getHeight() { return height; }
    double getMaxOriginX() { return Max_origin_x; }
    double getMaxOriginY() { return Max_origin_y; }
    std::vector<geometry_msgs::msg::Polygon> getObstaclePolys() { return obstacle_polys; }
    bool checkCollision(const State &state, const geometry_msgs::msg::Polygon &vehicle_poly_state);

    // ===================== NEW METHOD =====================
    void setcarData(CarData car_data);
    double getObstacleDistance(const Eigen::Vector2d &pos) const;
    bool isInside(const Eigen::Vector2d &pos) const;
    bool isSingleStateCollisionFree(const State &current);
    bool isSingleStateCollisionFreeImproved(const State &current);
    cv::Mat eigenToCvMat(const Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> &eigen_matrix);

    std::tuple<int, int> toCellID(State start_state);
    int toCellIndex(int x, int y);
    bool isPointInBounds(int x, int y);
    bool isInCollision(int x, int y);
};

#endif // GRID_MAP_H