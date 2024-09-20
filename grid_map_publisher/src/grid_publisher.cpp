#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class GridMapPublisher : public rclcpp::Node
{
public:
  GridMapPublisher()
      : Node("grid_map_publisher")
  {
    // Initialize grid map from image.
    std::string image_dir = "/home/genis/personal/ros2_ws/src/hybrid_a_start/grid_map_publisher";
    std::string image_file = "map_maze.png";
    image_dir.append("/" + image_file);
    cv::Mat img_src = cv::imread(image_dir, CV_8UC1);
    if (img_src.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to load image: %s", image_dir.c_str());
      return;
    }

    // Invert the image colors (black <-> white)
    cv::Mat img_inverted;
    cv::bitwise_not(img_src, img_inverted);

    double resolution = 1.0; // in meter
    grid_map_ = std::make_shared<grid_map::GridMap>(std::initializer_list<std::string>{"obstacle", "distance"});
    grid_map::GridMapCvConverter::initializeFromImage(
        img_inverted, resolution, *grid_map_, grid_map::Position::Zero());

    // Add obstacle layer.
    unsigned char OCCUPY = 255; // Adjust this if necessary
    unsigned char FREE = 0;

    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(
        img_src, "obstacle", *grid_map_, OCCUPY, FREE, 1);

    // Update distance layer.
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> binary =
        grid_map_->get("obstacle").cast<unsigned char>();
    cv::Mat binary_cv = eigen2cv(binary);
    cv::Mat distance_cv;
    cv::distanceTransform(binary_cv, distance_cv, CV_DIST_L2, CV_DIST_MASK_PRECISE);
    Eigen::MatrixXf distance = cv2eigen<float>(distance_cv);
    grid_map_->get("distance") = distance * resolution;
    grid_map_->setFrameId("map");

    // Set publisher.
    publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("grid_map", 10);
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&GridMapPublisher::publishGridMap, this));

    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> Grid map initialized.\033[0m");
  }

private:
  void publishGridMap()
  {
    auto occupancy_grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    grid_map::GridMapRosConverter::toOccupancyGrid(
        *grid_map_, "obstacle", 0, 255, *occupancy_grid);

    publisher_->publish(*occupancy_grid);
  }

  cv::Mat eigen2cv(const Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> &matrix)
  {
    cv::Mat mat(matrix.rows(), matrix.cols(), CV_8UC1);
    for (int i = 0; i < matrix.rows(); ++i)
      for (int j = 0; j < matrix.cols(); ++j)
        mat.at<unsigned char>(i, j) = matrix(i, j);
    return mat;
  }

  template <typename T>
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> cv2eigen(const cv::Mat &mat)
  {
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> matrix(mat.rows, mat.cols);
    for (int i = 0; i < mat.rows; ++i)
      for (int j = 0; j < mat.cols; ++j)
        matrix(i, j) = mat.at<T>(i, j);
    return matrix;
  }

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<grid_map::GridMap> grid_map_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridMapPublisher>());
  rclcpp::shutdown();
  return 0;
}
