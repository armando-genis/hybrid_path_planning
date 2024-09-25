#ifndef MAIN_GENERATOR_H
#define MAIN_GENERATOR_H

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <vector>
#include <cmath>
#include <iostream>

// Eigen
#include <Eigen/Dense>

#include "State.h"
#include "CubicSpline1D.h"

class main_generator : public rclcpp::Node
{
private:
    /* data */
    std::vector<State> path;
    double interpolate_step = 0.15;
    Eigen::MatrixXd car_steering_zone;

    // function
    void goal_point(const geometry_msgs::msg::PoseStamped::SharedPtr goal);
    geometry_msgs::msg::Point rotate_point(double x, double y, double yaw, const geometry_msgs::msg::Point &center);
    std::vector<State> interpolate_path();
    void publish_interpolated_path(const std::vector<State> &interpolated_path);
    void generate_vehicle_path_polygon(const std::vector<State> &interpolated_path, double track_car, geometry_msgs::msg::Polygon &vehicle_path);
    void publish_street(const geometry_msgs::msg::Polygon &vehicle_path);

    double normalize_angle(double angle)
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }

    // subscriber
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_point_sub_;

    // publisher
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pah_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_interpolated_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr street_pub_;

public:
    main_generator(/* args */);
    ~main_generator();
};

#endif // MAIN_GENERATOR_H