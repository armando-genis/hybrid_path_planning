
#include "sat_collision_checker.h"

namespace fop
{
/**
 * @brief Get the directions of the edges of a polygon
 * 
 * @param poly Polygon to get the edge directions from
 * @return std::vector<geometry_msgs::Vector3> 
 */
std::vector<geometry_msgs::msg::Vector3> SATCollisionChecker::vertices_to_edges(geometry_msgs::msg::Polygon poly)
{
    std::vector<geometry_msgs::msg::Vector3> edges;
    int num_vertices = poly.points.size();

    for (int i = 0; i < num_vertices - 1; i++)
    {
        edges.push_back(get_edge_direction(poly.points[i], poly.points[i + 1]));
    }

    return edges;
}


/**
 * @brief Get the direction of a single edge
 * 
 * @param p0 Start point of the edge
 * @param p1 End point of the edge
 * @return geometry_msgs::Vector3 
 */
geometry_msgs::msg::Vector3 SATCollisionChecker::get_edge_direction(geometry_msgs::msg::Point32 p0, geometry_msgs::msg::Point32 p1)
{
    geometry_msgs::msg::Vector3 direction;
    direction.x = p1.x - p0.x;
    direction.y = p1.y - p0.y;

    return direction;
}

/**
 * @brief Get the normal of a vector
 * 
 * @param vec Vector to get the normal of
 * @return geometry_msgs::Vector3 
 */
geometry_msgs::msg::Vector3 SATCollisionChecker::get_orthogonal(geometry_msgs::msg::Vector3 vec)
{
    geometry_msgs::msg::Vector3 orthogonal;
    orthogonal.x = vec.y;
    orthogonal.y = -vec.x;

    return orthogonal;
}

/**
 * @brief Normalize a vector to get a unit vector
 * 
 * @param vec Vector to be normalized
 * @return geometry_msgs::Vector3 
 */
geometry_msgs::msg::Vector3 SATCollisionChecker::normalize(geometry_msgs::msg::Vector3 vec)
{
    geometry_msgs::msg::Vector3 normalized;
    double norm = sqrt(pow(vec.x, 2) + pow(vec.y, 2));
    normalized.x = vec.x / norm;
    normalized.y = vec.y / norm;

    return normalized;
}

/**
 * @brief Project a polygon onto an axis
 * 
 * @param poly Polygon to be projected
 * @param axis Axis the polygon is to be projected onto
 * @return std::vector<double> 
 */
std::vector<double> SATCollisionChecker::project(geometry_msgs::msg::Polygon poly, geometry_msgs::msg::Vector3 axis)
{
    std::vector<double> products;
    for (int i = 0; i < poly.points.size(); i++)
    {
        products.push_back(get_dot_product(poly.points[i], axis));
    }
    double min = *min_element(products.begin(), products.end());
    double max = *max_element(products.begin(), products.end());

    return std::vector<double>{min, max};
}

/**
 * @brief Get the dot product between a point and an axis
 * 
 * @param point
 * @param axis
 * @return double 
 */
double SATCollisionChecker::get_dot_product(geometry_msgs::msg::Point32 point, geometry_msgs::msg::Vector3 axis)
{
    return point.x * axis.x + point.y * axis.y;
}

/**
 * @brief 
 * 
 * @param projection1 
 * @param projection2 
 * @return true 
 * @return false 
 */
bool SATCollisionChecker::is_overlapping(const std::vector<double>& projection1, const std::vector<double>& projection2)
{
    if (projection1[0] < projection2[0]) {
        return projection2[0] - projection1[1] < 0;
    } else {
        return projection1[0] - projection2[1] < 0;
    }
}


/**
 * @brief Check for collision between the object and the obstacle
 * 
 * @param rect Rectangle representing the vehicle
 * @param obstacle_poly Convex hull of the obstacle
 * @return true 
 * @return false 
 */
bool SATCollisionChecker::check_collision(geometry_msgs::msg::Polygon rect, geometry_msgs::msg::Polygon obstacle_poly)
{

    if (rect.points.empty() || obstacle_poly.points.empty()) {
        return false; // No collision possible if one polygon is empty
    }

    std::vector<geometry_msgs::msg::Vector3> edges1 = vertices_to_edges(rect);
    std::vector<geometry_msgs::msg::Vector3> edges2 = vertices_to_edges(obstacle_poly);

    // Remove the last duplicated edge if any
    if (!edges1.empty()) edges1.pop_back();
    if (!edges2.empty()) edges2.pop_back();

    std::vector<geometry_msgs::msg::Vector3> axes;
    for (auto& edge : edges1) {
        axes.push_back(get_orthogonal(edge));
    }
    for (auto& edge : edges2) {
        axes.push_back(get_orthogonal(edge));
    }

    for (auto& axis : axes) {
        std::vector<double> projection1 = project(rect, axis);
        std::vector<double> projection2 = project(obstacle_poly, axis);
        if (!is_overlapping(projection1, projection2)) {
            return false;
        }
    }
    return true;
}


/**
 * @brief Construct a rectangle with coordinates in the map frame.
 * 
 * @param centre_x x-coordinate of the rectangle center in the map frame
 * @param centre_y y-coodinate of the rectangle center in the map frame
 * @param yaw Angle (in radians) to rotate the rectangle about
 * @param length Length of the vehicle
 * @param width Width of the vehicle
 * @param margin Lateral safety margin 
 * @return Rectangle representing the position of the vehicle in the map frame. Total width of rectangle =  width + (2 * margin) 
 */
geometry_msgs::msg::Polygon SATCollisionChecker::construct_rectangle(double centre_x, double centre_y, double yaw, double length, double width, double margin_lon, double margin_lat)
{
    geometry_msgs::msg::Point32 p1, p2, p3, p4;

    double bottom_x = -length * (0.5 + margin_lon);
    double top_x = length * (0.5 + margin_lon);
    double left_y = -width * (0.5 + margin_lat);
    double right_y = width * (0.5 + margin_lat);

    p1.x = bottom_x;
    p1.y = left_y;
    p2.x = top_x;
    p2.y = left_y;
    p3.x = top_x;
    p3.y = right_y;
    p4.x = bottom_x;
    p4.y = right_y;

    geometry_msgs::msg::Polygon rectangle;
    rectangle.points.push_back(p1);
    rectangle.points.push_back(p2);
    rectangle.points.push_back(p3);
    rectangle.points.push_back(p4);

    rectangle = rotate_and_translate_rect(rectangle, centre_x, centre_y, yaw);
    rectangle.points.push_back(rectangle.points.front()); // Closing the polygon loop
    return rectangle;
}


/**
 * @brief Construct a straight bumper with coordinates in the map frame.
 * 
 * @param centre_x x-coordinate of the rectangle center in the map frame
 * @param centre_y y-coodinate of the rectangle center in the map frame
 * @param yaw Angle (in radians) to rotate the rectangle about
 * @param length Length of the straight bumper, dynamic with respect to speed of the vehicle
 * @param width Width of the vehicle
 * @param margin Lateral safety margin 
 * @return Rectangle representing the position of the dynamic straight bumper in the map frame.
 */
geometry_msgs::msg::Polygon SATCollisionChecker::construct_straight_bumper(double centre_x, double centre_y, double yaw,
                                                                           double length, double width, double margin)
{
    geometry_msgs::msg::Point32 p1, p2, p3, p4;

    double bottom_x = -1.5; // Half of true length?
    double top_x = 1.5 + length;
    double left_y = -width / 2 - margin;
    double right_y = width / 2 + margin;

    p1.x = bottom_x;
    p1.y = left_y;
    p2.x = top_x;
    p2.y = left_y;
    p3.x = top_x;
    p3.y = right_y;
    p4.x = bottom_x;
    p4.y = right_y;

    geometry_msgs::msg::Polygon rectangle;
    rectangle.points.push_back(p1);
    rectangle.points.push_back(p2);
    rectangle.points.push_back(p3);
    rectangle.points.push_back(p4);

    rectangle = rotate_and_translate_rect(rectangle, centre_x, centre_y, yaw);
    rectangle.points.push_back(rectangle.points.front()); // Close the polygon loop
    return rectangle;
}


/**
 * @brief Rotate a rectangle centered about (0, 0) by the yaw angle given and translate it to its actual position
 * 
 * @param rect The rectangle centered about (0, 0)
 * @param centre_x x-coordinate of the rectangle center in the map frame
 * @param centre_y y-coordinate of the rectangle center in the map frame
 * @param yaw Angle (in radians) to rotate the rectangle about
 * @return geometry_msgs::Polygon 
 */
geometry_msgs::msg::Polygon SATCollisionChecker::rotate_and_translate_rect(geometry_msgs::msg::Polygon rect, double centre_x, double centre_y, double yaw)
{
    geometry_msgs::msg::Polygon rotated_rect;

    for (const geometry_msgs::msg::Point32& point : rect.points)
    {
        geometry_msgs::msg::Point32 new_point;
        double x_orig = point.x;
        double y_orig = point.y;
        new_point.x = x_orig * cos(yaw) - y_orig * sin(yaw);
        new_point.y = x_orig * sin(yaw) + y_orig * cos(yaw);
        new_point.x += centre_x;
        new_point.y += centre_y;
        rotated_rect.points.push_back(new_point);
    }

    return rotated_rect;
}


/**
 * @brief Removes the top layer of a 3D convex hull, i.e., the second half of its points have the same (x,y) coordinates
 * but different z coordinates.
 *
 * @param poly the 3D polygon to be flattened
 * @return geometry_msgs::Polygon
 */

geometry_msgs::msg::Polygon SATCollisionChecker::remove_top_layer(geometry_msgs::msg::Polygon poly)
{
    geometry_msgs::msg::Polygon new_polygon;
    int num_points = poly.points.size();

    for (int i = 0; i < num_points / 2; i++) // Assuming the top half is simply duplicated in 3D space
    {
        new_polygon.points.push_back(poly.points[i]);
    }

    return new_polygon;
}


} //namespace fop