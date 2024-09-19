#ifndef CARDATA_H
#define CARDATA_H

#include <cmath>
#include <iostream>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include "State.h" // Include the State class

using namespace std;

class CarData
{
public:
    double maxSteerAngle;
    double wheelBase;
    double axleToFront;
    double axleToBack;
    double width;
    double steer_ratio = 6;                          // Steering angle step or ratio
    double steer_step = maxSteerAngle / steer_ratio; // Steering angle step or ratio for the vehicle

    geometry_msgs::msg::Polygon vehicle_geometry;

    // Default constructor
    CarData() : maxSteerAngle(0.0), wheelBase(0.0), axleToFront(0.0), axleToBack(0.0), width(0.0) {}

    // Constructor with parameters
    CarData(double maxSteerAngle, double wheelBase,
            double axleToFront, double axleToBack, double width);

    // Method to create the vehicle's geometry
    void createVehicleGeometry();

    // Method to return the vehicle geometry
    geometry_msgs::msg::Polygon getVehicleGeometry_state(const State &state);

    // method to return the posistion base on the Car dynamics
    State getVehicleStep(const State &state, double phi, double m = 1, double dt = 1e-2);
};

#endif // CARDATA_H
