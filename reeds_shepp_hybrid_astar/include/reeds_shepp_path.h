#ifndef REEDS_SHEPP_PATH_H
#define REEDS_SHEPP_PATH_H

#include <cmath>
#include <iostream>
#include <vector>
#include <tuple>
#include <cassert>
#include <map>
#include <numeric>   // For std::accumulate
#include <algorithm> // For std::transform

const double STEP_SIZE = 0.2;
const double MAX_LENGTH = 1000.0;
const double PI = M_PI;

class PATH
{
public:
    std::vector<double> lengths;     // lengths of each part of path
    std::vector<std::string> ctypes; // type of each part of the path
    double L;                        // total path length
    std::vector<double> x;           // final x positions
    std::vector<double> y;           // final y positions
    std::vector<double> yaw;         // final yaw angles
    std::vector<int> directions;     // forward: 1, backward: -1

    // Constructor
    PATH(const std::vector<double> &lengths,
         const std::vector<std::string> &ctypes,
         double L,
         const std::vector<double> &x,
         const std::vector<double> &y,
         const std::vector<double> &yaw,
         const std::vector<int> &directions)
        : lengths(lengths), ctypes(ctypes), L(L), x(x), y(y), yaw(yaw), directions(directions) {}
};

class reeds_shepp_path
{
private:
    /* data */
public:
    reeds_shepp_path(/* args */);
    ~reeds_shepp_path();

    PATH calc_optimal_path(double sx, double sy, double syaw,
                           double gx, double gy, double gyaw,
                           double maxc, double step_size = STEP_SIZE);

    std::vector<PATH> calc_all_paths(double sx, double sy, double syaw,
                                     double gx, double gy, double gyaw,
                                     double maxc, double step_size = STEP_SIZE);

    std::vector<PATH> set_path(std::vector<PATH> &paths,
                               const std::vector<double> &lengths,
                               const std::vector<std::string> &ctypes);

    std::tuple<bool, double, double, double> LSL(double x, double y, double phi);

    std::tuple<bool, double, double, double> LSR(double x, double y, double phi);

    std::tuple<bool, double, double, double> LRL(double x, double y, double phi);

    std::vector<PATH> SCS(double x, double y, double phi, std::vector<PATH> paths);

    std::tuple<bool, double, double, double> SLS(double x, double y, double phi);

    std::vector<PATH> CSC(double x, double y, double phi, std::vector<PATH> paths);

    std::vector<PATH> CCC(double x, double y, double phi, std::vector<PATH> paths);

    std::tuple<double, double> calc_tauOmega(double u, double v, double xi, double eta, double phi);

    std::tuple<bool, double, double, double> LRLRn(double x, double y, double phi);

    std::tuple<bool, double, double, double> LRLRp(double x, double y, double phi);

    std::vector<PATH> CCCC(double x, double y, double phi, std::vector<PATH> paths);

    std::tuple<bool, double, double, double> LRSR(double x, double y, double phi);

    std::tuple<bool, double, double, double> LRSL(double x, double y, double phi);

    std::vector<PATH> CCSC(double x, double y, double phi, std::vector<PATH> paths);

    std::tuple<bool, double, double, double> LRSLR(double x, double y, double phi);

    std::vector<PATH> CCSCC(double x, double y, double phi, std::vector<PATH> paths);

    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<int>>
    generate_local_course(double L, const std::vector<double> &lengths, const std::vector<char> &mode,
                          double maxc, double step_size);

    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<int>>
    interpolate(int ind, double l, char m, double maxc, double ox, double oy, double oyaw,
                std::vector<double> &px, std::vector<double> &py, std::vector<double> &pyaw, std::vector<int> &directions);

    std::vector<PATH> generate_path(const std::array<double, 3> &q0, const std::array<double, 3> &q1, double maxc);

    double pi_2_pi(double theta);

    std::tuple<double, double> R(double x, double y);

    double M(double theta);

    std::string get_label(const PATH &path);

    std::tuple<std::vector<double>, std::vector<double>> calc_curvature(
        const std::vector<double> &x,
        const std::vector<double> &y,
        const std::vector<double> &yaw,
        const std::vector<int> &directions);

    void check_path(double sx, double sy, double syaw,
                    double gx, double gy, double gyaw,
                    double maxc);
};

#endif // REEDS_SHEPP_PATH_H
