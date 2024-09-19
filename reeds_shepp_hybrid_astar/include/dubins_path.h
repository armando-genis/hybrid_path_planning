// dubins.h
#ifndef DUBINS_H
#define DUBINS_H

#include <iostream>
#include <vector>
#include <tuple>
#include <cmath>
#include <algorithm>
#include <numeric>  // Include this for std::accumulate

// Structure to hold the path information
struct Path {
    double length;
    std::vector<std::string> mode;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> yaw;
};

class dubins_path
{
private:
    /* data */


    // Dubins path functions
    std::tuple<double, double, double, std::vector<std::string>> LSL(double alpha, double beta, double dist);
    std::tuple<double, double, double, std::vector<std::string>> RSR(double alpha, double beta, double dist);
    std::tuple<double, double, double, std::vector<std::string>> LSR(double alpha, double beta, double dist);
    std::tuple<double, double, double, std::vector<std::string>> RSL(double alpha, double beta, double dist);
    std::tuple<double, double, double, std::vector<std::string>> RLR(double alpha, double beta, double dist);
    std::tuple<double, double, double, std::vector<std::string>> LRL(double alpha, double beta, double dist);

    // Dubins path generation

    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<int>> interpolate(
        int ind, double l, const std::string &m, double maxc, double ox, double oy, double oyaw,
        std::vector<double> &px, std::vector<double> &py, std::vector<double> &pyaw, std::vector<int> &directions);

    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<int>> generate_local_course(
        double L, const std::vector<double> &lengths, const std::vector<std::string> &mode, double maxc, double step_size);

    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<std::string>, double> planning_from_origin(
        double gx, double gy, double gyaw, double curv, double step_size);



public:
    dubins_path(/* args */);
    ~dubins_path();

    // Utility functions
    double pi_2_pi(double theta);
    double mod2pi(double theta);

    Path calc_dubins_path(double sx, double sy, double syaw, double gx, double gy, double gyaw, double curv, double step_size = 0.1) ;

};


#endif // DUBINS_H
