cmake_minimum_required(VERSION 3.8)
project(reeds_shepp_hybrid_astar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

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

add_executable(reeds_shepp_path_planning_node 
                src/main_path_planning.cpp # main file
                src/CarData.cpp
                src/Grid_map.cpp
                src/sat_collision_checker.cc
                src/HybridAstar.cpp
                src/dubins_path.cpp
                src/reeds_shepp_path.cpp
              )

ament_target_dependencies(reeds_shepp_path_planning_node rclcpp std_msgs geometry_msgs visualization_msgs tf2 tf2_geometry_msgs nav_msgs)

target_include_directories(reeds_shepp_path_planning_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(reeds_shepp_path_planning_node Eigen3::Eigen)

# Install executables and libraries
install(TARGETS
  reeds_shepp_path_planning_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include/
)

file(GLOB SOURCES "src/*.cpp")

# Install launch files and src files
install(
  DIRECTORY launch src config
  DESTINATION share/${PROJECT_NAME}
)

message(=============================================================)
message("-- ROS Found. ROS Support is turned ON.")
message(=============================================================)

ament_package()
