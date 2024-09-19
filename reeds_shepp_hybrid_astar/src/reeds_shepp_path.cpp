#include "reeds_shepp_path.h"

reeds_shepp_path::reeds_shepp_path(/* args */)
{
}

reeds_shepp_path::~reeds_shepp_path()
{
}

PATH reeds_shepp_path::calc_optimal_path(double sx, double sy, double syaw,
                                         double gx, double gy, double gyaw,
                                         double maxc, double step_size)
{

    // Print the sx and sy values
    // std::cout << "sx: " << sx << std::endl;
    // std::cout << "sy: " << sy << std::endl;
    // std::cout << "syaw: " << syaw << std::endl;

    std::vector<PATH> paths = calc_all_paths(sx, sy, syaw, gx, gy, gyaw, maxc, step_size);

    // Initialize variables for finding the minimum path
    double minL = paths[0].L;
    std::size_t mini = 0;

    // Loop through all paths to find the one with the smallest length
    for (std::size_t i = 0; i < paths.size(); ++i)
    {
        if (paths[i].L <= minL)
        {
            minL = paths[i].L;
            mini = i;
        }
    }

    return paths[mini];
}

std::vector<PATH> reeds_shepp_path::calc_all_paths(double sx, double sy, double syaw,
                                                   double gx, double gy, double gyaw,
                                                   double maxc, double step_size)
{
    std::array<double, 3> q0 = {sx, sy, syaw};
    std::array<double, 3> q1 = {gx, gy, gyaw};

    std::vector<PATH> paths = generate_path(q0, q1, maxc);

    for (auto &path : paths)
    {
        std::vector<double> x, y, yaw;
        std::vector<int> directions;

        // Mode conversion: Convert ctypes to a vector of characters
        std::vector<char> mode;
        for (const auto &ctype : path.ctypes)
        {
            if (!ctype.empty())
            {
                mode.push_back(ctype[0]); // Take the first character of each string
            }
        }

        // Generate the local course
        std::tie(x, y, yaw, directions) = generate_local_course(path.L, path.lengths,
                                                                mode, maxc, step_size * maxc);

        // print path length
        // std::cout << "path length: " << path.L << std::endl;

        // Convert global coordinates
        path.x.resize(x.size());
        path.y.resize(y.size());
        path.yaw.resize(yaw.size());

        for (size_t i = 0; i < x.size(); ++i)
        {
            path.x[i] = cos(-q0[2]) * x[i] + sin(-q0[2]) * y[i] + q0[0];
            path.y[i] = -sin(-q0[2]) * x[i] + cos(-q0[2]) * y[i] + q0[1];
            path.yaw[i] = pi_2_pi(yaw[i] + q0[2]);
        }

        // Assign directions
        path.directions = directions;

        // Normalize lengths and total path length by maxc
        for (auto &l : path.lengths)
        {
            l /= maxc;
        }

        path.L /= maxc;
    }

    return paths;
}

std::vector<PATH> reeds_shepp_path::set_path(std::vector<PATH> &paths,
                                             const std::vector<double> &lengths,
                                             const std::vector<std::string> &ctypes)
{
    // Create a new path with provided lengths and ctypes
    PATH path({}, ctypes, 0.0, {}, {}, {}, {});
    path.ctypes = ctypes;
    path.lengths = lengths;

    // Check if the same path exists
    for (const auto &path_e : paths)
    {
        if (path_e.ctypes == path.ctypes)
        {
            double sum_diff = 0.0;
            for (size_t i = 0; i < path_e.lengths.size(); ++i)
            {
                sum_diff += path_e.lengths[i] - path.lengths[i]; // Direct subtraction, no abs
            }
            if (sum_diff <= 0.01)
            {
                return paths; // Path already exists, return without adding
            }
        }
    }

    // Calculate total length (L) of the path
    path.L = std::accumulate(lengths.begin(), lengths.end(), 0.0, [](double acc, double val)
                             { return acc + std::abs(val); });

    // Check if the total path length exceeds the maximum allowed length
    if (path.L >= MAX_LENGTH)
    {
        return paths;
    }

    // Ensure path.L is valid
    assert(path.L >= 0.01);

    // Add the new path to the list
    paths.push_back(path);

    return paths;
}

std::tuple<bool, double, double, double> reeds_shepp_path::LSL(double x, double y, double phi)
{
    auto [u, t] = R(x - std::sin(phi), y - 1.0 + std::cos(phi));

    if (t >= 0.0)
    {
        double v = M(phi - t);
        if (v >= 0.0)
        {
            return {true, t, u, v};
        }
    }

    return {false, 0.0, 0.0, 0.0};
}

std::tuple<bool, double, double, double> reeds_shepp_path::LSR(double x, double y, double phi)
{
    double u1, t1;
    std::tie(u1, t1) = R(x + std::sin(phi), y - 1.0 - std::cos(phi));
    u1 = u1 * u1;

    if (u1 >= 4.0)
    {
        double u = std::sqrt(u1 - 4.0);
        double theta = std::atan2(2.0, u);
        double t = M(t1 + theta);
        double v = M(t - phi);

        if (t >= 0.0 && v >= 0.0)
        {
            return std::make_tuple(true, t, u, v);
        }
    }

    return std::make_tuple(false, 0.0, 0.0, 0.0);
}

std::tuple<bool, double, double, double> reeds_shepp_path::LRL(double x, double y, double phi)
{
    double u1, t1;
    std::tie(u1, t1) = R(x - std::sin(phi), y - 1.0 + std::cos(phi));

    if (u1 <= 4.0)
    {
        double u = -2.0 * std::asin(0.25 * u1);
        double t = M(t1 + 0.5 * u + PI);
        double v = M(phi - t + u);

        if (t >= 0.0 && u <= 0.0)
        {
            return std::make_tuple(true, t, u, v);
        }
    }

    return std::make_tuple(false, 0.0, 0.0, 0.0);
}

std::vector<PATH> reeds_shepp_path::SCS(double x, double y, double phi, std::vector<PATH> paths)
{
    bool flag;
    double t, u, v;

    std::tie(flag, t, u, v) = SLS(x, y, phi);
    if (flag)
        paths = set_path(paths, {t, u, v}, {"S", "L", "S"});

    std::tie(flag, t, u, v) = SLS(x, -y, -phi);
    if (flag)
        paths = set_path(paths, {t, u, v}, {"S", "R", "S"});

    return paths;
}

std::tuple<bool, double, double, double> reeds_shepp_path::SLS(double x, double y, double phi)
{
    phi = M(phi);

    if (y > 0.0 && 0.0 < phi && phi < PI * 0.99)
    {
        double xd = -y / std::tan(phi) + x;
        double t = xd - std::tan(phi / 2.0);
        double u = phi;
        double v = std::sqrt((x - xd) * (x - xd) + y * y) - std::tan(phi / 2.0);
        return std::make_tuple(true, t, u, v);
    }
    else if (y < 0.0 && 0.0 < phi && phi < PI * 0.99)
    {
        double xd = -y / std::tan(phi) + x;
        double t = xd - std::tan(phi / 2.0);
        double u = phi;
        double v = -std::sqrt((x - xd) * (x - xd) + y * y) - std::tan(phi / 2.0);
        return std::make_tuple(true, t, u, v);
    }

    return std::make_tuple(false, 0.0, 0.0, 0.0);
}

std::vector<PATH> reeds_shepp_path::CSC(double x, double y, double phi, std::vector<PATH> paths)
{
    bool flag;
    double t, u, v;

    // LSL
    std::tie(flag, t, u, v) = LSL(x, y, phi);
    if (flag)
    {
        paths = set_path(paths, {t, u, v}, {"L", "S", "L"});
    }

    std::tie(flag, t, u, v) = LSL(-x, y, -phi);
    if (flag)
    {
        paths = set_path(paths, {-t, -u, -v}, {"L", "S", "L"});
    }

    std::tie(flag, t, u, v) = LSL(x, -y, -phi);
    if (flag)
    {
        paths = set_path(paths, {t, u, v}, {"R", "S", "R"});
    }

    std::tie(flag, t, u, v) = LSL(-x, -y, phi);
    if (flag)
    {
        paths = set_path(paths, {-t, -u, -v}, {"R", "S", "R"});
    }

    // LSR
    std::tie(flag, t, u, v) = LSR(x, y, phi);
    if (flag)
    {
        paths = set_path(paths, {t, u, v}, {"L", "S", "R"});
    }

    std::tie(flag, t, u, v) = LSR(-x, y, -phi);
    if (flag)
    {
        paths = set_path(paths, {-t, -u, -v}, {"L", "S", "R"});
    }

    std::tie(flag, t, u, v) = LSR(x, -y, -phi);
    if (flag)
    {
        paths = set_path(paths, {t, u, v}, {"R", "S", "L"});
    }

    std::tie(flag, t, u, v) = LSR(-x, -y, phi);
    if (flag)
    {
        paths = set_path(paths, {-t, -u, -v}, {"R", "S", "L"});
    }

    return paths;
}

std::vector<PATH> reeds_shepp_path::CCC(double x, double y, double phi, std::vector<PATH> paths)
{
    bool flag;
    double t, u, v;

    std::tie(flag, t, u, v) = LRL(x, y, phi);
    if (flag)
    {
        paths = set_path(paths, {t, u, v}, {"L", "R", "L"});
    }

    std::tie(flag, t, u, v) = LRL(-x, y, -phi);
    if (flag)
    {
        paths = set_path(paths, {-t, -u, -v}, {"L", "R", "L"});
    }

    std::tie(flag, t, u, v) = LRL(x, -y, -phi);
    if (flag)
    {
        paths = set_path(paths, {t, u, v}, {"R", "L", "R"});
    }

    std::tie(flag, t, u, v) = LRL(-x, -y, phi);
    if (flag)
    {
        paths = set_path(paths, {-t, -u, -v}, {"R", "L", "R"});
    }

    // backwards
    double xb = x * std::cos(phi) + y * std::sin(phi);
    double yb = x * std::sin(phi) - y * std::cos(phi);

    std::tie(flag, t, u, v) = LRL(xb, yb, phi);
    if (flag)
    {
        paths = set_path(paths, {v, u, t}, {"L", "R", "L"});
    }

    std::tie(flag, t, u, v) = LRL(-xb, yb, -phi);
    if (flag)
    {
        paths = set_path(paths, {-v, -u, -t}, {"L", "R", "L"});
    }

    std::tie(flag, t, u, v) = LRL(xb, -yb, -phi);
    if (flag)
    {
        paths = set_path(paths, {v, u, t}, {"R", "L", "R"});
    }

    std::tie(flag, t, u, v) = LRL(-xb, -yb, phi);
    if (flag)
    {
        paths = set_path(paths, {-v, -u, -t}, {"R", "L", "R"});
    }

    return paths;
}

std::tuple<double, double> reeds_shepp_path::calc_tauOmega(double u, double v, double xi, double eta, double phi)
{
    double delta = M(u - v);
    double A = std::sin(u) - std::sin(delta);
    double B = std::cos(u) - std::cos(delta) - 1.0;

    double t1 = std::atan2(eta * A - xi * B, xi * A + eta * B);
    double t2 = 2.0 * (std::cos(delta) - std::cos(v) - std::cos(u)) + 3.0;

    double tau = t2 < 0 ? M(t1 + PI) : M(t1);
    double omega = M(tau - u + v - phi);

    return std::make_tuple(tau, omega);
}

std::tuple<bool, double, double, double> reeds_shepp_path::LRLRn(double x, double y, double phi)
{
    double xi = x + std::sin(phi);
    double eta = y - 1.0 - std::cos(phi);
    double rho = 0.25 * (2.0 + std::sqrt(xi * xi + eta * eta));

    if (rho <= 1.0)
    {
        double u = std::acos(rho);
        double t, v;
        std::tie(t, v) = calc_tauOmega(u, -u, xi, eta, phi);

        if (t >= 0.0 && v <= 0.0)
        {
            return std::make_tuple(true, t, u, v);
        }
    }

    return std::make_tuple(false, 0.0, 0.0, 0.0);
}

std::tuple<bool, double, double, double> reeds_shepp_path::LRLRp(double x, double y, double phi)
{
    double xi = x + std::sin(phi);
    double eta = y - 1.0 - std::cos(phi);
    double rho = (20.0 - xi * xi - eta * eta) / 16.0;

    if (rho >= 0.0 && rho <= 1.0)
    {
        double u = -std::acos(rho);
        if (u >= -0.5 * PI)
        {
            double t, v;
            std::tie(t, v) = calc_tauOmega(u, u, xi, eta, phi);

            if (t >= 0.0 && v >= 0.0)
            {
                return std::make_tuple(true, t, u, v);
            }
        }
    }

    return std::make_tuple(false, 0.0, 0.0, 0.0);
}

std::vector<PATH> reeds_shepp_path::CCCC(double x, double y, double phi, std::vector<PATH> paths)
{
    bool flag;
    double t, u, v;

    std::tie(flag, t, u, v) = LRLRn(x, y, phi);
    if (flag)
    {
        paths = set_path(paths, {t, u, -u, v}, {"L", "R", "L", "R"});
    }

    std::tie(flag, t, u, v) = LRLRn(-x, y, -phi);
    if (flag)
    {
        paths = set_path(paths, {-t, -u, u, -v}, {"L", "R", "L", "R"});
    }

    std::tie(flag, t, u, v) = LRLRn(x, -y, -phi);
    if (flag)
    {
        paths = set_path(paths, {t, u, -u, v}, {"R", "L", "R", "L"});
    }

    std::tie(flag, t, u, v) = LRLRn(-x, -y, phi);
    if (flag)
    {
        paths = set_path(paths, {-t, -u, u, -v}, {"R", "L", "R", "L"});
    }

    std::tie(flag, t, u, v) = LRLRp(x, y, phi);
    if (flag)
    {
        paths = set_path(paths, {t, u, u, v}, {"L", "R", "L", "R"});
    }

    std::tie(flag, t, u, v) = LRLRp(-x, y, -phi);
    if (flag)
    {
        paths = set_path(paths, {-t, -u, -u, -v}, {"L", "R", "L", "R"});
    }

    std::tie(flag, t, u, v) = LRLRp(x, -y, -phi);
    if (flag)
    {
        paths = set_path(paths, {t, u, u, v}, {"R", "L", "R", "L"});
    }

    std::tie(flag, t, u, v) = LRLRp(-x, -y, phi);
    if (flag)
    {
        paths = set_path(paths, {-t, -u, -u, -v}, {"R", "L", "R", "L"});
    }

    return paths;
}

std::tuple<bool, double, double, double> reeds_shepp_path::LRSR(double x, double y, double phi)
{
    double xi = x + std::sin(phi);
    double eta = y - 1.0 - std::cos(phi);
    double rho, theta;
    std::tie(rho, theta) = R(-eta, xi);

    if (rho >= 2.0)
    {
        double t = theta;
        double u = 2.0 - rho;
        double v = M(t + 0.5 * PI - phi);
        if (t >= 0.0 && u <= 0.0 && v <= 0.0)
        {
            return std::make_tuple(true, t, u, v);
        }
    }

    return std::make_tuple(false, 0.0, 0.0, 0.0);
}

std::tuple<bool, double, double, double> reeds_shepp_path::LRSL(double x, double y, double phi)
{
    double xi = x - std::sin(phi);
    double eta = y - 1.0 + std::cos(phi);
    double rho, theta;
    std::tie(rho, theta) = R(xi, eta);

    if (rho >= 2.0)
    {
        double r = std::sqrt(rho * rho - 4.0);
        double u = 2.0 - r;
        double t = M(theta + std::atan2(r, -2.0));
        double v = M(phi - 0.5 * PI - t);
        if (t >= 0.0 && u <= 0.0 && v <= 0.0)
        {
            return std::make_tuple(true, t, u, v);
        }
    }

    return std::make_tuple(false, 0.0, 0.0, 0.0);
}

std::vector<PATH> reeds_shepp_path::CCSC(double x, double y, double phi, std::vector<PATH> paths)
{
    bool flag;
    double t, u, v;

    std::tie(flag, t, u, v) = LRSL(x, y, phi);
    if (flag)
    {
        paths = set_path(paths, {t, -0.5 * PI, u, v}, {"L", "R", "S", "L"});
    }

    std::tie(flag, t, u, v) = LRSL(-x, y, -phi);
    if (flag)
    {
        paths = set_path(paths, {-t, 0.5 * PI, -u, -v}, {"L", "R", "S", "L"});
    }

    std::tie(flag, t, u, v) = LRSL(x, -y, -phi);
    if (flag)
    {
        paths = set_path(paths, {t, -0.5 * PI, u, v}, {"R", "L", "S", "R"});
    }

    std::tie(flag, t, u, v) = LRSL(-x, -y, phi);
    if (flag)
    {
        paths = set_path(paths, {-t, 0.5 * PI, -u, -v}, {"R", "L", "S", "R"});
    }

    std::tie(flag, t, u, v) = LRSR(x, y, phi);
    if (flag)
    {
        paths = set_path(paths, {t, -0.5 * PI, u, v}, {"L", "R", "S", "R"});
    }

    std::tie(flag, t, u, v) = LRSR(-x, y, -phi);
    if (flag)
    {
        paths = set_path(paths, {-t, 0.5 * PI, -u, -v}, {"L", "R", "S", "R"});
    }

    std::tie(flag, t, u, v) = LRSR(x, -y, -phi);
    if (flag)
    {
        paths = set_path(paths, {t, -0.5 * PI, u, v}, {"R", "L", "S", "L"});
    }

    std::tie(flag, t, u, v) = LRSR(-x, -y, phi);
    if (flag)
    {
        paths = set_path(paths, {-t, 0.5 * PI, -u, -v}, {"R", "L", "S", "L"});
    }

    // Backwards
    double xb = x * std::cos(phi) + y * std::sin(phi);
    double yb = x * std::sin(phi) - y * std::cos(phi);

    std::tie(flag, t, u, v) = LRSL(xb, yb, phi);
    if (flag)
    {
        paths = set_path(paths, {v, u, -0.5 * PI, t}, {"L", "S", "R", "L"});
    }

    std::tie(flag, t, u, v) = LRSL(-xb, yb, -phi);
    if (flag)
    {
        paths = set_path(paths, {-v, -u, 0.5 * PI, -t}, {"L", "S", "R", "L"});
    }

    std::tie(flag, t, u, v) = LRSL(xb, -yb, -phi);
    if (flag)
    {
        paths = set_path(paths, {v, u, -0.5 * PI, t}, {"R", "S", "L", "R"});
    }

    std::tie(flag, t, u, v) = LRSL(-xb, -yb, phi);
    if (flag)
    {
        paths = set_path(paths, {-v, -u, 0.5 * PI, -t}, {"R", "S", "L", "R"});
    }

    std::tie(flag, t, u, v) = LRSR(xb, yb, phi);
    if (flag)
    {
        paths = set_path(paths, {v, u, -0.5 * PI, t}, {"R", "S", "R", "L"});
    }

    std::tie(flag, t, u, v) = LRSR(-xb, yb, -phi);
    if (flag)
    {
        paths = set_path(paths, {-v, -u, 0.5 * PI, -t}, {"R", "S", "R", "L"});
    }

    std::tie(flag, t, u, v) = LRSR(xb, -yb, -phi);
    if (flag)
    {
        paths = set_path(paths, {v, u, -0.5 * PI, t}, {"L", "S", "L", "R"});
    }

    std::tie(flag, t, u, v) = LRSR(-xb, -yb, phi);
    if (flag)
    {
        paths = set_path(paths, {-v, -u, 0.5 * PI, -t}, {"L", "S", "L", "R"});
    }

    return paths;
}

std::tuple<bool, double, double, double> reeds_shepp_path::LRSLR(double x, double y, double phi)
{
    // formula 8.11 *** TYPO IN PAPER ***
    double xi = x + std::sin(phi);
    double eta = y - 1.0 - std::cos(phi);
    double rho, theta;
    std::tie(rho, theta) = R(xi, eta);

    if (rho >= 2.0)
    {
        double u = 4.0 - std::sqrt(rho * rho - 4.0);
        if (u <= 0.0)
        {
            double t = M(std::atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta));
            double v = M(t - phi);

            if (t >= 0.0 && v >= 0.0)
            {
                return std::make_tuple(true, t, u, v);
            }
        }
    }

    return std::make_tuple(false, 0.0, 0.0, 0.0);
}

std::vector<PATH> reeds_shepp_path::CCSCC(double x, double y, double phi, std::vector<PATH> paths)
{
    bool flag;
    double t, u, v;

    std::tie(flag, t, u, v) = LRSLR(x, y, phi);
    if (flag)
    {
        paths = set_path(paths, {t, -0.5 * PI, u, -0.5 * PI, v}, {"L", "R", "S", "L", "R"});
    }

    std::tie(flag, t, u, v) = LRSLR(-x, y, -phi);
    if (flag)
    {
        paths = set_path(paths, {-t, 0.5 * PI, -u, 0.5 * PI, -v}, {"L", "R", "S", "L", "R"});
    }

    std::tie(flag, t, u, v) = LRSLR(x, -y, -phi);
    if (flag)
    {
        paths = set_path(paths, {t, -0.5 * PI, u, -0.5 * PI, v}, {"R", "L", "S", "R", "L"});
    }

    std::tie(flag, t, u, v) = LRSLR(-x, -y, phi);
    if (flag)
    {
        paths = set_path(paths, {-t, 0.5 * PI, -u, 0.5 * PI, -v}, {"R", "L", "S", "R", "L"});
    }

    return paths;
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<int>>
reeds_shepp_path::generate_local_course(double L, const std::vector<double> &lengths, const std::vector<char> &mode,
                                        double maxc, double step_size)
{

    int point_num = static_cast<int>(L / step_size) + lengths.size() + 3;

    std::vector<double> px(point_num, 0.0);
    std::vector<double> py(point_num, 0.0);
    std::vector<double> pyaw(point_num, 0.0);
    std::vector<int> directions(point_num, 0);

    int ind = 1;

    directions[0] = (lengths[0] > 0.0) ? 1 : -1;
    double d = (lengths[0] > 0.0) ? step_size : -step_size;

    double pd = d;
    double ll = 0.0;

    for (size_t i = 0; i < mode.size(); ++i)
    {
        double l = lengths[i];
        char m = mode[i];

        d = (l > 0.0) ? step_size : -step_size;

        double ox = px[ind], oy = py[ind], oyaw = pyaw[ind];
        --ind;

        if (i >= 1 && (lengths[i - 1] * lengths[i]) > 0)
        {
            pd = -d - ll;
        }
        else
        {
            pd = d - ll;
        }

        while (std::abs(pd) <= std::abs(l))
        {
            ++ind;
            std::tie(px, py, pyaw, directions) =
                interpolate(ind, pd, m, maxc, ox, oy, oyaw, px, py, pyaw, directions);
            pd += d;
        }

        ll = l - pd - d; // calc remain length

        ++ind;
        std::tie(px, py, pyaw, directions) =
            interpolate(ind, l, m, maxc, ox, oy, oyaw, px, py, pyaw, directions);
    }

    // remove unused data
    while (!px.empty() && px.back() == 0.0)
    {
        px.pop_back();
        py.pop_back();
        pyaw.pop_back();
        directions.pop_back();
    }

    return std::make_tuple(px, py, pyaw, directions);
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<int>>
reeds_shepp_path::interpolate(int ind, double l, char m, double maxc, double ox, double oy, double oyaw,
                              std::vector<double> &px, std::vector<double> &py, std::vector<double> &pyaw, std::vector<int> &directions)
{

    if (m == 'S')
    {
        px[ind] = ox + l / maxc * std::cos(oyaw);
        py[ind] = oy + l / maxc * std::sin(oyaw);
        pyaw[ind] = oyaw;
    }
    else
    {
        double ldx = std::sin(l) / maxc;
        double ldy;

        if (m == 'L')
        {
            ldy = (1.0 - std::cos(l)) / maxc;
        }
        else if (m == 'R')
        {
            ldy = (1.0 - std::cos(l)) / (-maxc);
        }

        double gdx = std::cos(-oyaw) * ldx + std::sin(-oyaw) * ldy;
        double gdy = -std::sin(-oyaw) * ldx + std::cos(-oyaw) * ldy;
        px[ind] = ox + gdx;
        py[ind] = oy + gdy;
    }

    if (m == 'L')
    {
        pyaw[ind] = oyaw + l;
    }
    else if (m == 'R')
    {
        pyaw[ind] = oyaw - l;
    }

    directions[ind] = (l > 0.0) ? 1 : -1;

    return std::make_tuple(px, py, pyaw, directions);
}

std::vector<PATH> reeds_shepp_path::generate_path(const std::array<double, 3> &q0,
                                                  const std::array<double, 3> &q1,
                                                  double maxc)
{
    double dx = q1[0] - q0[0];
    double dy = q1[1] - q0[1];
    double dth = q1[2] - q0[2];
    double c = std::cos(q0[2]);
    double s = std::sin(q0[2]);
    double x = (c * dx + s * dy) * maxc;
    double y = (-s * dx + c * dy) * maxc;

    std::vector<PATH> paths;

    paths = SCS(x, y, dth, paths);
    paths = CSC(x, y, dth, paths);
    paths = CCC(x, y, dth, paths);
    paths = CCCC(x, y, dth, paths);
    paths = CCSC(x, y, dth, paths);
    paths = CCSCC(x, y, dth, paths);

    return paths;
}

// Utils
// Function to normalize theta to the range [-PI, PI]
double reeds_shepp_path::pi_2_pi(double theta)
{
    while (theta > PI)
    {
        theta -= 2.0 * PI;
    }

    while (theta < -PI)
    {
        theta += 2.0 * PI;
    }

    return theta;
}

// Function to return the polar coordinates (r, theta) of the point (x, y)
std::tuple<double, double> reeds_shepp_path::R(double x, double y)
{
    double r = std::hypot(x, y);
    double theta = std::atan2(y, x);

    return std::make_tuple(r, theta);
}

// Function to regulate theta to the range [-PI, PI]
double reeds_shepp_path::M(double theta)
{
    double phi = std::fmod(theta, 2.0 * PI);

    if (phi < -PI)
    {
        phi += 2.0 * PI;
    }
    if (phi > PI)
    {
        phi -= 2.0 * PI;
    }

    return phi;
}

std::string reeds_shepp_path::get_label(const PATH &path)
{
    std::string label;

    for (std::size_t i = 0; i < path.ctypes.size(); ++i)
    {
        label += path.ctypes[i];
        if (path.lengths[i] > 0.0)
        {
            label += "+";
        }
        else
        {
            label += "-";
        }
    }

    return label;
}

std::tuple<std::vector<double>, std::vector<double>> reeds_shepp_path::calc_curvature(
    const std::vector<double> &x,
    const std::vector<double> &y,
    const std::vector<double> &yaw,
    const std::vector<int> &directions)
{

    std::vector<double> c, ds;

    for (std::size_t i = 1; i < x.size() - 1; ++i)
    {
        double dxn = x[i] - x[i - 1];
        double dxp = x[i + 1] - x[i];
        double dyn = y[i] - y[i - 1];
        double dyp = y[i + 1] - y[i];
        double dn = std::hypot(dxn, dyn);
        double dp = std::hypot(dxp, dyp);

        double dx = 1.0 / (dn + dp) * (dp / dn * dxn + dn / dp * dxp);
        double ddx = 2.0 / (dn + dp) * (dxp / dp - dxn / dn);
        double dy = 1.0 / (dn + dp) * (dp / dn * dyn + dn / dp * dyp);
        double ddy = 2.0 / (dn + dp) * (dyp / dp - dyn / dn);

        double curvature = (ddy * dx - ddx * dy) / (dx * dx + dy * dy);
        double d = (dn + dp) / 2.0;

        if (std::isnan(curvature))
        {
            curvature = 0.0;
        }

        if (directions[i] <= 0.0)
        {
            curvature = -curvature;
        }

        if (c.empty())
        {
            ds.push_back(d);
            c.push_back(curvature);
        }

        ds.push_back(d);
        c.push_back(curvature);
    }

    if (!ds.empty())
    {
        ds.push_back(ds.back());
        c.push_back(c.back());
    }

    return std::make_tuple(c, ds);
}

void reeds_shepp_path::check_path(double sx, double sy, double syaw,
                                  double gx, double gy, double gyaw,
                                  double maxc)
{
    std::vector<PATH> paths = calc_all_paths(sx, sy, syaw, gx, gy, gyaw, maxc);

    assert(!paths.empty());

    for (const auto &path : paths)
    {
        assert(std::abs(path.x[0] - sx) <= 0.01);
        assert(std::abs(path.y[0] - sy) <= 0.01);
        assert(std::abs(path.yaw[0] - syaw) <= 0.01);
        assert(std::abs(path.x.back() - gx) <= 0.01);
        assert(std::abs(path.y.back() - gy) <= 0.01);
        assert(std::abs(path.yaw.back() - gyaw) <= 0.01);

        std::vector<double> dx(path.x.size() - 1);
        std::vector<double> dy(path.y.size() - 1);

        // Calculate differences between consecutive points
        std::transform(path.x.begin() + 1, path.x.end(), path.x.begin(), dx.begin(), std::minus<double>());
        std::transform(path.y.begin() + 1, path.y.end(), path.y.begin(), dy.begin(), std::minus<double>());

        std::vector<double> d(dx.size());

        for (std::size_t i = 0; i < dx.size(); ++i)
        {
            d[i] = std::hypot(dx[i], dy[i]);
            assert(std::abs(d[i] - STEP_SIZE) <= 0.001);
        }
    }
}