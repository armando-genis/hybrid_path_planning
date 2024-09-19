#ifndef SAT_COLLISION_CHECKER_H_
#define SAT_COLLISION_CHECKER_H_

#include <math.h>
#include <vector>

#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point32.hpp>

namespace fop
{
class SATCollisionChecker
{
public:
  SATCollisionChecker(){};
  virtual ~SATCollisionChecker(){};
  
  bool check_collision(geometry_msgs::msg::Polygon rect, geometry_msgs::msg::Polygon obstacle_poly);
  
  geometry_msgs::msg::Polygon construct_rectangle(const double centre_x, const double centre_y, const double yaw, 
                                             const double length, const double width, 
                                             const double margin_lon, const double margin_lat);

  geometry_msgs::msg::Polygon construct_straight_bumper(double centre_x, double centre_y, double yaw,
                                                   double length, double width, double margin);
  
  geometry_msgs::msg::Polygon remove_top_layer(geometry_msgs::msg::Polygon poly);

private:
  std::vector<geometry_msgs::msg::Vector3> vertices_to_edges(geometry_msgs::msg::Polygon poly);
  geometry_msgs::msg::Vector3 get_edge_direction(geometry_msgs::msg::Point32 p0, geometry_msgs::msg::Point32 p1);
  geometry_msgs::msg::Vector3 get_orthogonal(geometry_msgs::msg::Vector3 vec);
  geometry_msgs::msg::Vector3 normalize(geometry_msgs::msg::Vector3 vec);
  std::vector<double> project(geometry_msgs::msg::Polygon poly, geometry_msgs::msg::Vector3 axis);
  double get_dot_product(geometry_msgs::msg::Point32 point, geometry_msgs::msg::Vector3 axis);
  bool is_overlapping(const std::vector<double>& projection1, const std::vector<double>& projection2);
  geometry_msgs::msg::Polygon rotate_and_translate_rect(geometry_msgs::msg::Polygon rect, double centre_x, double centre_y, double yaw);
};
}  // namespace fop

#endif  // SAT_COLLISION_CHECKER_H_

