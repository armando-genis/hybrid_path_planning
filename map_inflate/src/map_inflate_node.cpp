#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"

using namespace cv;
using namespace std;

class map_inflate_node : public rclcpp::Node
{
private:
    double inflation_radius;
    string grid_map_topic_IN;
    string grid_map_topic_OUT;

    void MapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void InflateMap(const Mat &input_map, Mat &output_map, double resolution);
    nav_msgs::msg::OccupancyGrid CreateInflatedOccupancyGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr &original_grid, const Mat &inflated_map);

    Mat LabelMap; // Label map that marks obstacles and free space

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_map_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr inflated_map_pub_;

public:
    map_inflate_node();
    ~map_inflate_node();
};

map_inflate_node::map_inflate_node() : Node("map_inflate_node")
{
    this->declare_parameter<double>("inflation_radius", 0.25);
    this->declare_parameter<std::string>("grid_map_topic_IN", "/grid_map");
    this->declare_parameter<std::string>("grid_map_topic_OUT", "/grid_map_inflated");

    this->get_parameter("inflation_radius", inflation_radius);
    this->get_parameter("grid_map_topic_IN", grid_map_topic_IN);
    this->get_parameter("grid_map_topic_OUT", grid_map_topic_OUT);

    grid_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        grid_map_topic_IN, 10, std::bind(&map_inflate_node::MapCallback, this, std::placeholders::_1));
    inflated_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(grid_map_topic_OUT, 10);

    RCLCPP_INFO(this->get_logger(), "Inflated map Node initialized with inflation_radius: %.2f meters", inflation_radius);
}

map_inflate_node::~map_inflate_node() {}

void map_inflate_node::MapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    int height = msg->info.height;
    int width = msg->info.width;
    double resolution = msg->info.resolution; // Get the map resolution in meters per pixel

    Mat map(height, width, CV_8UC1);

    // Convert the occupancy grid to OpenCV Mat format (from ROS to OpenCV)
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            int occ_prob = msg->data[i * width + j];
            occ_prob = (occ_prob < 0) ? 100 : occ_prob;                               // Set unknown to 100
            map.at<uchar>(height - i - 1, j) = 255 - round(occ_prob * 255.0 / 100.0); // Convert to grayscale
        }
    }

    // Inflate the map using the map resolution
    Mat inflated_map;
    InflateMap(map, inflated_map, resolution);

    // Convert the inflated map back to OccupancyGrid and publish it
    nav_msgs::msg::OccupancyGrid inflated_occ_grid = CreateInflatedOccupancyGrid(msg, inflated_map);
    inflated_map_pub_->publish(inflated_occ_grid);

    // Optionally, print a success message
    RCLCPP_INFO(this->get_logger(), "Inflated map published.");
}

void map_inflate_node::InflateMap(const Mat &input_map, Mat &output_map, double resolution)
{
    // Convert inflation radius from meters to pixels using the map's resolution
    int inflate_radius = static_cast<int>(round(inflation_radius / resolution));

    if (inflate_radius < 1)
    {
        inflate_radius = 1; // Ensure at least a small inflation
    }

    RCLCPP_INFO(this->get_logger(), "Inflation radius in pixels: %d", inflate_radius);

    // Threshold the map for binarization (use Otsu's method to handle automatic binarization)
    Mat binarized_map;
    threshold(input_map, binarized_map, 0, 255, THRESH_BINARY | THRESH_OTSU);

    // Invert the binarized map so that obstacles are white and can be inflated
    Mat inverted_map;
    bitwise_not(binarized_map, inverted_map);

    // Inflate obstacles based on the inflate_radius (using dilation)
    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(2 * inflate_radius + 1, 2 * inflate_radius + 1), Point(inflate_radius, inflate_radius));
    dilate(inverted_map, output_map, element);

    // Invert the map back to its original state (obstacles black)
    bitwise_not(output_map, output_map);
}

nav_msgs::msg::OccupancyGrid map_inflate_node::CreateInflatedOccupancyGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr &original_grid, const Mat &inflated_map)
{
    // Create a new OccupancyGrid message for the inflated map
    nav_msgs::msg::OccupancyGrid inflated_occ_grid;
    inflated_occ_grid.header.stamp = this->now();
    inflated_occ_grid.header.frame_id = original_grid->header.frame_id;
    inflated_occ_grid.info = original_grid->info; // Copy the map info (resolution, size, etc.)
    inflated_occ_grid.data.resize(original_grid->info.height * original_grid->info.width);

    // Convert OpenCV Mat back to occupancy grid values (from 0-255 grayscale to 0-100 occupancy)
    for (int i = 0; i < original_grid->info.height; i++)
    {
        for (int j = 0; j < original_grid->info.width; j++)
        {
            uchar occ_prob = inflated_map.at<uchar>(original_grid->info.height - i - 1, j);
            inflated_occ_grid.data[i * original_grid->info.width + j] = 100 - round(occ_prob * 100.0 / 255.0); // Convert back to occupancy grid values
        }
    }

    return inflated_occ_grid;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<map_inflate_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
