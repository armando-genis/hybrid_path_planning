#ifndef CARDATA_H
#define CARDATA_H

#include <cmath>
#include <iostream>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include "State.h" // Include the State class
#include "iostream"

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

    std::vector<Circle> circles_;
    Circle bounding_c_;

    // colors for the terminal
    string green = "\033[1;32m";
    string red = "\033[1;31m";
    string blue = "\033[1;34m";
    string yellow = "\033[1;33m";
    string purple = "\033[1;35m";
    string reset = "\033[0m";

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

    // Method to set the circles around the vehicle
    void setCircles();

    // Method to return the bounding circle around the vehicle
    Circle getBoundingCircle(const State &state) const;

    // Method to return the circles around the vehicle
    std::vector<Circle> getCircles(const State &pos) const;

    // local to global transformation
    State local2Global(const State &reference, const State &target) const;
};

#endif // CARDATA_H
