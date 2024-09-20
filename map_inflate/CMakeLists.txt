cmake_minimum_required(VERSION 3.8)
project(map_inflate)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# External dependencies
find_package(OpenCV REQUIRED)

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

add_executable(map_inflate_node 
                src/map_inflate_node.cpp
              )

ament_target_dependencies(map_inflate_node rclcpp nav_msgs cv_bridge)

target_link_libraries(map_inflate_node ${OpenCV_LIBS})

# Install executables and libraries
install(TARGETS
  map_inflate_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and src files
install(
  DIRECTORY launch src config
  DESTINATION share/${PROJECT_NAME}
)

message(=============================================================)
message("-- ROS Found. ROS Support is turned ON.")
message(=============================================================)

ament_package()
