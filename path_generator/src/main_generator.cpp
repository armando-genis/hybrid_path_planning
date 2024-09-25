#include "main_generator.h"

main_generator::main_generator(/* args */) : Node("path_planner_node")
{

    // Initialize subscribers
    goal_point_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&main_generator::goal_point, this, std::placeholders::_1));

    // Initialize publishers
    pah_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/start_and_end_arrows", 10);
    path_interpolated_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/path_interpolated", 10);
    street_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/vehicle_path", 10); // Street publisher

    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> path_generator_node initialized.\033[0m");
}

main_generator::~main_generator()
{
}

geometry_msgs::msg::Point main_generator::rotate_point(double x, double y, double yaw, const geometry_msgs::msg::Point &center)
{
    geometry_msgs::msg::Point rotated_point;
    rotated_point.x = center.x + (x * cos(yaw) - y * sin(yaw));
    rotated_point.y = center.y + (x * sin(yaw) + y * cos(yaw));
    rotated_point.z = center.z;
    return rotated_point;
}

// ###################################################
//  POINTS FOR PATH GENERATION
// ###################################################
void main_generator::goal_point(const geometry_msgs::msg::PoseStamped::SharedPtr goal)
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

    // Create a new state with the goal position and yaw
    State new_state(goal->pose.position.x, goal->pose.position.y, yaw);

    // Add the new state to the path
    path.push_back(new_state);

    // Initialize marker array for path visualization
    visualization_msgs::msg::MarkerArray path_generator;
    double triangle_size = 1.0;

    // Create a marker
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "path_planner";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set marker scale and color
    marker.scale.x = 1.0; // x scaling
    marker.scale.y = 1.0; // y scaling
    marker.scale.z = 1.0; // z scaling

    marker.color.r = 1.0; // Red
    marker.color.g = 1.0; // Green
    marker.color.b = 1.0; // Blue
    marker.color.a = 1.0; // transparency

    // Loop through the path and create points for the TRIANGLE_LIST
    for (const auto &state : path)
    {
        // Define triangle vertices relative to the center point (state.x, state.y)
        geometry_msgs::msg::Point center;
        center.x = state.x;
        center.y = state.y;
        center.z = 0.0;

        // Define the local coordinates of the triangle vertices (before rotation)
        geometry_msgs::msg::Point p1, p2, p3;
        p1.x = 0.0;
        p1.y = -triangle_size / 2;
        p1.z = 0.0;

        p2.x = triangle_size;
        p2.y = 0.0;
        p2.z = 0.0;

        p3.x = 0.0;
        p3.y = triangle_size / 2;
        p3.z = 0.0;

        // Rotate the points based on the state's yaw (heading) and move them to the state's position
        p1 = rotate_point(p1.x, p1.y, state.heading, center);
        p2 = rotate_point(p2.x, p2.y, state.heading, center);
        p3 = rotate_point(p3.x, p3.y, state.heading, center);

        // Add the rotated points to the marker's points field
        marker.points.push_back(p1);
        marker.points.push_back(p2);
        marker.points.push_back(p3);
    }

    // Add the marker to the marker array
    path_generator.markers.push_back(marker);

    // Publish the marker array
    pah_pub_->publish(path_generator);

    // Interpolate the path
    auto interpolated_path = interpolate_path();
    publish_interpolated_path(interpolated_path);

    // street creation
    Eigen::MatrixXd car_steering_zone(interpolated_path.size() * 2, 2);
    geometry_msgs::msg::Polygon vehicle_path;
    generate_vehicle_path_polygon(interpolated_path, 1.5, vehicle_path);
    publish_street(vehicle_path);
}

std::vector<State> main_generator::interpolate_path()
{
    if (path.size() < 2)
    {
        RCLCPP_ERROR(this->get_logger(), "Path has insufficient points to interpolate.");
        return std::vector<State>(); // Return an empty vector
    }

    std::vector<double> x, y, yaw;

    // Populate x, y, and yaw based on the path
    for (const auto &state : path)
    {
        x.push_back(state.x);
        y.push_back(state.y);
        yaw.push_back(state.heading);
    }

    // Create t_values after populating x, y, and yaw with the correct size
    std::vector<double> t_values(path.size());
    std::iota(t_values.begin(), t_values.end(), 0); // Simple linear time steps

    // Create splines for x and y
    CubicSpline1D x_spline(t_values, x);
    CubicSpline1D y_spline(t_values, y);

    std::vector<State> interpolated_path;

    for (double t = 0; t < t_values.size() - 1; t += interpolate_step)
    {
        // Interpolate x and y using cubic splines
        double interp_x = x_spline.calc_der0(t);
        double interp_y = y_spline.calc_der0(t);

        // Interpolate yaw using linear interpolation, handling wrap-around
        size_t idx = static_cast<size_t>(t); // Integer part of t
        double t_fraction = t - idx;         // Fractional part of t

        double yaw_start = yaw[idx];
        double yaw_end = yaw[idx + 1];
        double yaw_diff = normalize_angle(yaw_end - yaw_start);

        // Interpolate yaw while respecting the wrap-around
        double interp_yaw = normalize_angle(yaw_start + t_fraction * yaw_diff);

        // Create a new state with the interpolated x, y, and yaw
        State new_state(interp_x, interp_y, interp_yaw);
        interpolated_path.push_back(new_state);
    }

    // Push the last state to ensure the final goal point is included
    State last_state(x.back(), y.back(), yaw.back());
    interpolated_path.push_back(last_state);

    return interpolated_path;
}

void main_generator::publish_interpolated_path(const std::vector<State> &interpolated_path)
{
    // Initialize marker array for path interpolation visualization
    visualization_msgs::msg::MarkerArray path_interpolated_marker;

    // Create a marker for the interpolated path as a line strip
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "path_interpolated";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set marker scale and color
    marker.scale.x = 0.1; // Line width
    marker.color.r = 0.0; // Red
    marker.color.g = 1.0; // Green
    marker.color.b = 0.0; // Blue
    marker.color.a = 1.0; // Transparency

    // Add interpolated points to the marker
    for (const auto &state : interpolated_path)
    {
        geometry_msgs::msg::Point p;
        p.x = state.x;
        p.y = state.y;
        p.z = 0.0; // Assuming a 2D plane
        marker.points.push_back(p);
    }

    // Add the marker to the marker array
    path_interpolated_marker.markers.push_back(marker);

    // Publish the marker array
    path_interpolated_pub_->publish(path_interpolated_marker);
}

void main_generator::generate_vehicle_path_polygon(const std::vector<State> &interpolated_path, double track_car, geometry_msgs::msg::Polygon &vehicle_path)
{

    vehicle_path.points.clear();
    // Condition for interpolated_path is empty return a empty polygon
    if (interpolated_path.empty())
    {
        return;
    }

    std::vector<geometry_msgs::msg::Point32> right_side_points;
    std::vector<geometry_msgs::msg::Point32> left_side_points;

    for (size_t i = 0; i < interpolated_path.size(); ++i)
    {
        const State &state = interpolated_path[i];

        // Extract the vehicle's heading (yaw) from the state
        double yaw = state.heading;

        // Calculate the unit vectors perpendicular to the vehicle's direction
        double dx = cos(yaw);
        double dy = sin(yaw);

        // Calculate the left and right points offset from the center by half the track width
        geometry_msgs::msg::Point32 left_point, right_point;

        right_point.x = state.x + track_car * dy; // Right point (perpendicular to the direction)
        right_point.y = state.y - track_car * dx;
        right_point.z = 0.0;

        left_point.x = state.x - track_car * dy; // Left point (perpendicular to the direction)
        left_point.y = state.y + track_car * dx;
        left_point.z = 0.0;

        // Add the right and left points to their respective vectors
        right_side_points.push_back(right_point);
        left_side_points.push_back(left_point);
    }

    // Add the right-side points in order
    for (const auto &right_point : right_side_points)
    {
        vehicle_path.points.push_back(right_point);
    }

    // Add the left-side points in reverse order to close the polygon
    for (auto it = left_side_points.rbegin(); it != left_side_points.rend(); ++it)
    {
        vehicle_path.points.push_back(*it);
    }

    // close the loop
    vehicle_path.points.push_back(vehicle_path.points.front());
}

void main_generator::publish_street(const geometry_msgs::msg::Polygon &vehicle_path)
{
    visualization_msgs::msg::Marker street_marker;
    street_marker.header.frame_id = "map";
    street_marker.header.stamp = this->get_clock()->now();
    street_marker.ns = "vehicle_path";
    street_marker.id = 0;
    street_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    street_marker.action = visualization_msgs::msg::Marker::ADD;

    street_marker.scale.x = 0.1; // Line width
    street_marker.color.r = 1.0;
    street_marker.color.g = 1.0;
    street_marker.color.b = 1.0;
    street_marker.color.a = 1.0;

    for (const auto &point : vehicle_path.points)
    {
        geometry_msgs::msg::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0.0;
        street_marker.points.push_back(p);
    }

    street_pub_->publish(street_marker);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<main_generator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}