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

// method to return the posistion base on the Car dynamics
// State CarData::getVehicleStep(const State &state, double phi, double m, double dt)
// {
//     State new_state;

//     double x = state.x;
//     double y = state.y;
//     double heading = state.heading;

//     // car dynamics
//     double dx = cos(state.heading);
//     double dy = sin(state.heading);
//     double dtheta = tan(phi) / wheelBase;

//     // Update position and heading
//     x += m * dt * dx;
//     y += m * dt * dy;
//     heading += m * dt * dtheta;

//     new_state.x = x;
//     new_state.y = y;
//     new_state.heading = heading;

//     return new_state;
// }