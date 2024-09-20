cmake_minimum_required(VERSION 3.8)
project(grid_map_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(grid_map_ros REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(grid_map_publisher src/grid_publisher.cpp)
ament_target_dependencies(grid_map_publisher rclcpp cv_bridge grid_map_cv grid_map_ros)
target_link_libraries(grid_map_publisher ${OpenCV_LIBS})



install(TARGETS
  grid_map_publisher
  DESTINATION lib/${PROJECT_NAME})


install(
  DIRECTORY src
  DESTINATION share/${PROJECT_NAME}
)

message(=============================================================)
message("-- ROS Not Found. ROS Support is turned Off.")
message(=============================================================)


ament_package()
