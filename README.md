# hybrid_path_planning
 This repository contains ROS2 Humble packages implementing hybrid path planning algorithms for vehicle that can move both forward and backward, including reeds_shepp and dubins motion

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

![Reeds-Shepp-Curves Screenshot](https://raw.githubusercontent.com/henrywoo/images/main/live.png)


**The Dubins Hybrid A algorithm**: generates paths using only forward motion, leveraging Dubins curves for optimal path planning with non-holonomic constraints. The algorithm simulates vehicle motion based on a set of forward-only steering commands, checking for collisions and computing the cost of each path based on steering angles and direction changes. A Dubins path is calculated between the current and goal nodes, and the cost is based on the length of the path and steering complexity. Additionally, a holonomic heuristic is used to account for obstacles and to optimize the path when standard Dubins paths are not feasible.

![Dubins-Curves  Screenshot](https://raw.githubusercontent.com/henrywoo/images/main/live.png)

 colcon build --packages-select dubins_hybrid_star
 colcon build --packages-select reeds_shepp_hybrid_astar
 colcon build --packages-select map_inflate



ros2 launch dubins_hybrid_star dubins_hybrid.launch.py
ros2 launch reeds_shepp_hybrid_astar reeds_hybrid.launch.py
ros2 launch map_inflate map.launch.py



Global 
ros2 launch global_launcher reeds_shepp_hybrid.launch.py
ros2 launch global_launcher dubins_hybrid.launch.py