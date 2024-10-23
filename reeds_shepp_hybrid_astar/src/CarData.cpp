#include "CarData.h"

// Constructor to initialize values from main code
CarData::CarData(double maxSteerAngle, double wheelBase,
                 double axleToFront, double axleToBack, double width)
    : maxSteerAngle(maxSteerAngle),
      wheelBase(wheelBase),
      axleToFront(axleToFront),
      axleToBack(axleToBack),
      width(width)
{
    // Set the circles around the vehicle
    setCircles();
}

// Method to create the vehicle's geometry
void CarData::createVehicleGeometry()
{
    geometry_msgs::msg::Point32 p1, p2, p3, p4;

    // Define the points based on car dimensions
    p1.x = axleToFront;
    p1.y = width / 2.0;
    p2.x = axleToFront;
    p2.y = -width / 2.0;
    p3.x = -axleToBack;
    p3.y = -width / 2.0;
    p4.x = -axleToBack;
    p4.y = width / 2.0;

    // Add points to the polygon
    vehicle_geometry.points.push_back(p1);
    vehicle_geometry.points.push_back(p2);
    vehicle_geometry.points.push_back(p3);
    vehicle_geometry.points.push_back(p4);

    // Close the polygon by adding the first point again
    vehicle_geometry.points.push_back(vehicle_geometry.points.front());
}

// Method to return the vehicle geometry
geometry_msgs::msg::Polygon CarData::getVehicleGeometry_state(const State &state)
{
    geometry_msgs::msg::Polygon vehicle_poly_state;

    // Rotate and translate vehicle polygon according to the current state
    for (const auto &point : vehicle_geometry.points)
    {
        geometry_msgs::msg::Point32 p_point32;

        double x = state.x + point.x * cos(state.heading) - point.y * sin(state.heading);
        double y = state.y + point.x * sin(state.heading) + point.y * cos(state.heading);

        p_point32.x = x;
        p_point32.y = y;

        vehicle_poly_state.points.push_back(p_point32);
    }

    return vehicle_poly_state;
}

State CarData::getVehicleStep(const State &state, double phi, double m, double dt)
{
    double dx = cos(state.heading);
    double dy = sin(state.heading);
    double dtheta = tan(phi) / wheelBase;

    // Update position and heading
    return {
        state.x + (m * dt * dx),
        state.y + (m * dt * dy),
        state.heading + (m * dt * dtheta)};
}

void CarData::setCircles()
{
    // Bounding circle
    bounding_c_.x = (axleToFront - axleToBack) / 2.0;
    bounding_c_.y = 0;
    bounding_c_.r = sqrt(pow((axleToFront + axleToBack) / 2.0, 2) + pow(width / 2.0, 2));

    // Define small circles for corners
    double small_circle_shift = width / 4.0;
    double small_circle_radius = sqrt(2 * pow(small_circle_shift, 2));

    // Rear-left (rl) circle
    circles_.emplace_back(-axleToBack + small_circle_shift, width / 2.0, small_circle_radius);
    // Rear-right (rr) circle
    circles_.emplace_back(-axleToBack + small_circle_shift, -width / 2.0, small_circle_radius);
    // Front-left (fl) circle
    circles_.emplace_back(axleToFront - small_circle_shift, width / 2.0, small_circle_radius);
    // Front-right (fr) circle
    circles_.emplace_back(axleToFront - small_circle_shift, -width / 2.0, small_circle_radius);

    // cout circles seted in green
    cout << green << "---> Circles created <---" << reset << endl;
}

std::vector<Circle> CarData::getCircles(const State &pos) const
{
    std::vector<Circle> result;
    for (const auto &circle : circles_)
    {
        State state(circle.x, circle.y);
        auto global_state = local2Global(pos, state);
        result.emplace_back(global_state.x, global_state.y, circle.r);
    }
    return result;
}

Circle CarData::getBoundingCircle(const State &state) const
{
    auto global_center = local2Global(state, State(bounding_c_.x, bounding_c_.y));
    return Circle(global_center.x, global_center.y, bounding_c_.r);
}

State CarData::local2Global(const State &reference, const State &target) const
{
    double x = target.x * cos(reference.heading) - target.y * sin(reference.heading) + reference.x;
    double y = target.x * sin(reference.heading) + target.y * cos(reference.heading) + reference.y;
    double z = reference.heading + target.heading;
    return {x, y, z};
}