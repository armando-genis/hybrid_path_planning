#ifndef MAIN_PATH_PLANNING_H
#define MAIN_PATH_PLANNING_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <vector>
#include <cmath>
#include <iostream>

#include "State.h"
#include "Grid_map.h"
#include "CarData.h"
#include "HybridAstar.h"
#include "Node.h"

class main_path_planning : public rclcpp::Node
{
private:
    // Simulation parameters
    double pathLength; // 4
    double step_car;   // 1.15

    // Car Data
    CarData car_data_;
    double maxSteerAngle;
    double wheelBase;
    double axleToFront;
    double axleToBack;
    double width;

    std::string grid_map_topic;

    // Grid Map
    std::shared_ptr<Grid_map> grid_map_;

    // state
    State start_state_;
    State goal_state_;

    // bool to check if the start and goal point is received
    bool start_point_received_ = false;
    bool goal_point_received_ = false;

    // main functions
    void start_point(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr start);
    void goal_point(const geometry_msgs::msg::PoseStamped::SharedPtr goal);
    void gridMapdata(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
    void hybridAstarPathPlanning();

    // subscriber
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_point_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_point_sub_;

    // publisher
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hybrid_astar_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr polygon_car_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr arrow_pub_;

public:
    main_path_planning(/* args */);
    ~main_path_planning();
};

#endif // MAIN_PATH_PLANNING_H
