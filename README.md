# hybrid_path_planning
 This repository contains ROS2 Humble packages implementing hybrid path planning algorithms for vehicle that can move both forward and backward, including reeds_shepp and dubins motion

## 🔥 Optimized
- The `reeds_shepp_hybrid_astar` package leverages `grid_map_ro`s to create optimized "obstacle" and "distance" layers, reducing the time needed to calculate obstacle checks with the vehicle's geometry. This setup minimizes collision risks and enhances efficiency for faster, more accurate path planning.

## 📥 Install dependencies
```bash
sudo apt-get install libeigen3-dev
sudo apt install ros-$ROS_DISTRO-joint-state-publisher-gui
sudo apt install ros-$ROS_DISTRO-xacro
sudo apt install ros-$ROS_DISTRO-vision-opencv
sudo apt install ros-$ROS_DISTRO-grid-map-ros
sudo apt install ros-$ROS_DISTRO-grid-map-cv
sudo apt install ros-$ROS_DISTRO-cv-bridge
sudo apt install libopencv-dev
```
## 📋 Distribution

**The Reeds-Shepp Hybrid A algorithm**: incorporates both forward and reverse motion to generate optimal paths for non-holonomic vehicles. It computes a set of possible steering angles and directions and simulates vehicle movement to evaluate potential trajectories, checking for collisions and boundary conditions in a grid map. Costs are calculated based on movement direction (forward or reverse), steering angle, and changes in direction, with higher penalties for reversing and sharp turns. Additionally, the algorithm includes a heuristic function for holonomic motion planning, where obstacles are considered when calculating costs, and it uses the Reeds-Shepp curve to optimize the path when no valid solution is found in a straightforward trajectory.

![Reeds-Shepp-Curves Screenshot](https://github.com/armando-genis/hybrid_path_planning/raw/main/imgs/reeds_shepp.png)


**The Dubins Hybrid A algorithm**: generates paths using only forward motion, leveraging Dubins curves for optimal path planning with non-holonomic constraints. The algorithm simulates vehicle motion based on a set of forward-only steering commands, checking for collisions and computing the cost of each path based on steering angles and direction changes. A Dubins path is calculated between the current and goal nodes, and the cost is based on the length of the path and steering complexity. Additionally, a holonomic heuristic is used to account for obstacles and to optimize the path when standard Dubins paths are not feasible.

![Dubins-Curves Screenshot](https://github.com/armando-genis/hybrid_path_planning/raw/main/imgs/dubins.png)


## ⚙️ Configuration

This project includes various configuration options for different nodes, allowing for customization of vehicle parameters, map handling, and path planning settings. These configurations are located in `the global_launcher at the config file`. Below is an explanation of the configuration parameters for each node.

**Dubins Path Planning Node**:

- `maxSteerAngle`: The maximum steering angle of the vehicle in radians. This value controls how sharp the vehicle can turn. (Default: 0.6)
- `wheelBase`: The distance between the front and rear axles of the vehicle. This affects the vehicle's turning radius. (Default: 2.75)
- `axleToFront`: The distance from the front axle to the front of the vehicle. This is important for calculating the vehicle's geometry. (Default: 1.1)
-  `axleToBack`: The distance from the rear axle to the back of the vehicle. This is used for vehicle geometry calculation. (Default: 1.2)
- `width`: The width of the vehicle. This parameter affects collision checking and how the vehicle fits in the environment. (Default: 1.9)
- `pathLength`: The maximum length of the planned path in meters. (Default: 4.0)
- `step_car`: The step size for moving the vehicle forward in the simulation. `A smaller value results in finer control but more computation`. (Default: 1.0)
- `grid_map_topic`: The ROS topic that provides the grid map used for path planning. (Default: "/grid_map_inflated")

**Reeds-Shepp Path Planning Node**:

- `maxSteerAngle`: Same as above, the maximum steering angle in radians. (Default: 0.6)
- `wheelBase`: The distance between the front and rear axles. (Default: 2.75)
- `axleToFront`: The distance from the front axle to the front of the vehicle. (Default: 1.1)
- `axleToBack`: The distance from the rear axle to the back of the vehicle. (Default: 1.2)
- `width`: The width of the vehicle for collision checking. (Default: 1.9)
- `pathLength`: The maximum length of the planned path in meters. (Default: 4.0)
- `step_car`: Step size for moving the vehicle in the simulation. (Default: 1.0)
- `grid_map_topic`: The ROS topic for the grid map used in path planning. (Default: "/grid_map_inflated")

**Grid Map Publisher**:

- `resolution`: The resolution of the grid map, specifying how much area each cell represents in meters. A lower value provides higher detail. (Default: 0.2 or 1.0)
- `grid_map_topic`: The topic on which the grid map will be published. (Default: "/grid_map")
- `image_dir`: The directory path where the image file is stored. This image is converted into a grid map. (Default: "/home/genis/personal/ros2_ws/src/hybrid_path_planning/grid_map_publisher")
- `image_file`: The name of the image file used to create the grid map. (Default: "gridmap.png")

** Map Inflate Node**:

- `inflation_radius`: The radius in meters around obstacles that should be inflated to account for the vehicle’s size. (Default: 0.2)
- `grid_map_topic_IN`: The input topic where the original grid map is published. (Default: "/grid_map")
- `grid_map_topic_OUT`: The output topic where the inflated grid map will be published. (Default: "/grid_map_inflated")

## 📍 Build

```bash
 colcon build --packages-select dubins_hybrid_star
 colcon build --packages-select reeds_shepp_hybrid_astar
 colcon build --packages-select map_inflate
 colcon build --packages-select grid_map_publisher
 colcon build --packages-select global_launcher
```

## ▶️ Run
- Launch Reeds-Shepp Hybrid Path Planner:
```bash
ros2 launch global_launcher reeds_shepp_hybrid.launch.py
```
- Launch Dubins Hybrid Path Planner:
```bash
ros2 launch global_launcher dubins_hybrid.launch.py
```

## 🔴 Notes:
- ❗ Important: If you are publishing a map or using a specific grid map for planning, make sure to update the paths in the configuration file. This includes the `image_dir` and `image_file` paths for the grid_map_publisher. ❗
- Make sure your ROS2 environment is properly sourced before launching the files (source ~/ros2_ws/install/setup.bash).
- You can modify the parameters in the config/global_launcher.yaml file to suit your vehicle and environment.
- The planners require a grid map to operate, so ensure that a valid grid map is published to the specified topics (/grid_map_inflated).

