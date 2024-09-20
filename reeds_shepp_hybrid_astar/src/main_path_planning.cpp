#include "main_path_planning.h"

main_path_planning::main_path_planning(/* args */) : Node("main_planner_node")
{
    this->declare_parameter<double>("maxSteerAngle", 0.0);
    this->declare_parameter<double>("wheelBase", 0.0);
    this->declare_parameter<double>("axleToFront", 0.0);
    this->declare_parameter<double>("axleToBack", 0.0);
    this->declare_parameter<double>("width", 0.0);
    this->declare_parameter<double>("pathLength", 0.0);
    this->declare_parameter<double>("step_car", 0.0);
    this->declare_parameter<std::string>("grid_map_topic", "/grid_map");

    this->get_parameter("maxSteerAngle", maxSteerAngle);
    this->get_parameter("wheelBase", wheelBase);
    this->get_parameter("axleToFront", axleToFront);
    this->get_parameter("axleToBack", axleToBack);
    this->get_parameter("width", width);
    this->get_parameter("pathLength", pathLength);
    this->get_parameter("step_car", step_car);
    this->get_parameter("grid_map_topic", grid_map_topic);

    // Initialize subscribers
    grid_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(grid_map_topic, 10, std::bind(&main_path_planning::gridMapdata, this, std::placeholders::_1));
    start_point_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10, std::bind(&main_path_planning::start_point, this, std::placeholders::_1));
    goal_point_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&main_path_planning::goal_point, this, std::placeholders::_1));

    // Initialize publishers
    hybrid_astar_path_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/hybrid_astar_path", 10);
    polygon_car_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/car_polygon", 10);
    arrow_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/start_and_end_arrows", 10);

    // Create the vehicle geometry
    car_data_ = CarData(maxSteerAngle, wheelBase, axleToFront, axleToBack, width);
    car_data_.createVehicleGeometry();

    // log out parameters in blue color
    RCLCPP_INFO(this->get_logger(), "\033[1;34mmaxSteerAngle: %f\033[0m", maxSteerAngle);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mwheelBase: %f\033[0m", wheelBase);
    RCLCPP_INFO(this->get_logger(), "\033[1;34maxleToFront: %f\033[0m", axleToFront);
    RCLCPP_INFO(this->get_logger(), "\033[1;34maxleToBack: %f\033[0m", axleToBack);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mwidth: %f\033[0m", width);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mpathLength: %f\033[0m", pathLength);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mstep_car: %f\033[0m", step_car);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mgrid_map_topic: %s\033[0m", grid_map_topic.c_str());

    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> optimal_planner_node initialized.\033[0m");
}

main_path_planning::~main_path_planning()
{
}

// ###################################################
//  MAP
// ###################################################
void main_path_planning::gridMapdata(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
    // Create the grid map
    grid_map_ = std::make_shared<Grid_map>(*map);
}

// ###################################################
//  START POINT
// ###################################################
void main_path_planning::start_point(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr start)
{

    // Extract yaw from the quaternion
    tf2::Quaternion q(
        start->pose.pose.orientation.x,
        start->pose.pose.orientation.y,
        start->pose.pose.orientation.z,
        start->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Set the start point
    start_state_.x = start->pose.pose.position.x;
    start_state_.y = start->pose.pose.position.y;
    start_state_.heading = yaw;

    auto rotated_vehicle_poly = car_data_.getVehicleGeometry_state(start_state_);

    // Check if the start point is within the map
    if (start_state_.x < grid_map_->getOriginX() || start_state_.x > grid_map_->getMaxOriginX() || start_state_.y < grid_map_->getOriginY() || start_state_.y > grid_map_->getMaxOriginY())
    {
        RCLCPP_ERROR(this->get_logger(), "\033[1;31mStart point is out of the map\033[0m");
        return;
    }

    if (grid_map_->checkCollision(start_state_, rotated_vehicle_poly))
    {
        RCLCPP_ERROR(this->get_logger(), "\033[1;31mStart point is in collision\033[0m");
        return;
    }

    visualization_msgs::msg::Marker car_polygon_marker;
    car_polygon_marker.header.frame_id = "map";
    car_polygon_marker.header.stamp = this->now();
    car_polygon_marker.ns = "vehicle_polygon";
    car_polygon_marker.action = visualization_msgs::msg::Marker::ADD;
    car_polygon_marker.id = 30;
    car_polygon_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    car_polygon_marker.scale.x = 0.07;
    car_polygon_marker.color.r = 0.77;
    car_polygon_marker.color.g = 0.45;
    car_polygon_marker.color.b = 1.0;
    car_polygon_marker.color.a = 1.0;

    for (const auto &point : rotated_vehicle_poly.points)
    {
        geometry_msgs::msg::Point p_point;
        p_point.x = point.x;
        p_point.y = point.y;
        p_point.z = 0.0;
        car_polygon_marker.points.push_back(p_point);
    }

    polygon_car_pub_->publish(car_polygon_marker);

    start_point_received_ = true;
}

// ###################################################
//  GOAL POINT
// ###################################################
void main_path_planning::goal_point(const geometry_msgs::msg::PoseStamped::SharedPtr goal)
{
    // Extract yaw from the quaternion
    tf2::Quaternion q(
        goal->pose.orientation.x,
        goal->pose.orientation.y,
        goal->pose.orientation.z,
        goal->pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Set the goal point
    goal_state_.x = goal->pose.position.x;
    goal_state_.y = goal->pose.position.y;
    goal_state_.heading = yaw;

    auto rotated_vehicle_poly = car_data_.getVehicleGeometry_state(goal_state_);

    // Check if the start point is within the map
    if (goal_state_.x < grid_map_->getOriginX() || goal_state_.x > grid_map_->getMaxOriginX() || goal_state_.y < grid_map_->getOriginY() || goal_state_.y > grid_map_->getMaxOriginY())
    {
        RCLCPP_ERROR(this->get_logger(), "\033[1;31mStart point is out of the map\033[0m");
        return;
    }

    if (grid_map_->checkCollision(goal_state_, rotated_vehicle_poly))
    {
        RCLCPP_ERROR(this->get_logger(), "\033[1;31mGoal point is in collision\033[0m");
        return;
    }

    visualization_msgs::msg::Marker car_polygon_marker;
    car_polygon_marker.header.frame_id = "map";
    car_polygon_marker.header.stamp = this->now();
    car_polygon_marker.ns = "vehicle_polygon";
    car_polygon_marker.action = visualization_msgs::msg::Marker::ADD;
    car_polygon_marker.id = 31;
    car_polygon_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    car_polygon_marker.scale.x = 0.07;
    car_polygon_marker.color.r = 0.77;
    car_polygon_marker.color.g = 0.45;
    car_polygon_marker.color.b = 1.0;
    car_polygon_marker.color.a = 1.0;

    for (const auto &point : rotated_vehicle_poly.points)
    {
        geometry_msgs::msg::Point p_point;
        p_point.x = point.x;
        p_point.y = point.y;
        p_point.z = 0.0;
        car_polygon_marker.points.push_back(p_point);
    }

    polygon_car_pub_->publish(car_polygon_marker);

    goal_point_received_ = true;

    hybridAstarPathPlanning();
}

// ###################################################
// HYBRID A* PATH PLANNING
// ###################################################
void main_path_planning::hybridAstarPathPlanning()
{

    if (!start_point_received_ || !goal_point_received_)
    {
        RCLCPP_ERROR(this->get_logger(), "\033[1;31mStart or Goal point is not received\033[0m");
        return;
    }

    // push the start and goal points to vector start_goal_points_
    start_goal_points_.push_back(start_state_);
    start_goal_points_.push_back(goal_state_);

    Star_End_point_visualization();

    HybridAstar hybrid_astar(*grid_map_, car_data_, pathLength, step_car);
    auto goal_trajectory = hybrid_astar.run(start_state_, goal_state_);

    visualization_msgs::msg::MarkerArray marker_array_next;
    int id = 4000; // Unique ID for each marker
    for (const auto &state : goal_trajectory)
    {

        auto rotated_vehicle_poly = car_data_.getVehicleGeometry_state(state);

        visualization_msgs::msg::Marker car_polygon_marker;
        car_polygon_marker.header.frame_id = "map";
        car_polygon_marker.header.stamp = this->now();
        car_polygon_marker.ns = "next_state_polygons";
        car_polygon_marker.action = visualization_msgs::msg::Marker::ADD;
        car_polygon_marker.id = id++;
        car_polygon_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        car_polygon_marker.scale.x = 0.07;
        car_polygon_marker.color.r = 0.77;
        car_polygon_marker.color.g = 0.45;
        car_polygon_marker.color.b = 1.0;
        car_polygon_marker.color.a = 1.0;

        std::vector<geometry_msgs::msg::Point> car_data_points;

        for (const auto &point : rotated_vehicle_poly.points)
        {
            geometry_msgs::msg::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0.0;
            car_data_points.push_back(p);
        }

        car_data_points.push_back(car_data_points.front());

        car_polygon_marker.points = car_data_points;

        // Add the marker to the marker array
        marker_array_next.markers.push_back(car_polygon_marker);
    }

    // Publish the MarkerArray containing the arrows
    hybrid_astar_path_pub_->publish(marker_array_next);

    // memory clean up
    goal_trajectory.clear();
    marker_array_next.markers.clear();

    goal_point_received_ = false;
    start_point_received_ = false;
}

void main_path_planning::Star_End_point_visualization()
{

    visualization_msgs::msg::MarkerArray arrow_states;
    int id = 7000; // Unique ID for each marker
    for (const auto &state : start_goal_points_)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map"; // Assuming map frame
        marker.header.stamp = this->now();
        marker.ns = "next_state_arrows";
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::ARROW;

        marker.pose.position.x = state.x;
        marker.pose.position.y = state.y;
        marker.pose.position.z = 0.0;

        // Set the orientation based on the yaw angle (heading)
        tf2::Quaternion quat;
        quat.setRPY(0, 0, state.heading);
        marker.pose.orientation.x = quat.x();
        marker.pose.orientation.y = quat.y();
        marker.pose.orientation.z = quat.z();
        marker.pose.orientation.w = quat.w();

        marker.scale.x = 2.0; // Length of the arrow
        marker.scale.y = 0.3; // Width of the arrow shaft
        marker.scale.z = 0.3; // Height of the arrow shaft

        marker.color.r = 0.11;
        marker.color.g = 0.54;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        arrow_states.markers.push_back(marker);
    }

    // Publish the MarkerArray containing the arrows
    arrow_pub_->publish(arrow_states);

    start_goal_points_.clear();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<main_path_planning>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
