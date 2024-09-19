#include "dubins_path.h"

dubins_path::dubins_path()
{
    // Constructor code (if needed)
}

dubins_path::~dubins_path()
{
    // Destructor code (if needed)
}

double dubins_path::pi_2_pi(double theta)
{
    while (theta > M_PI)
    {
        theta -= 2.0 * M_PI;
    }
    while (theta < -M_PI)
    {
        theta += 2.0 * M_PI;
    }
    return theta;
}

double dubins_path::mod2pi(double theta)
{
    return theta - 2.0 * M_PI * std::floor(theta / M_PI / 2.0);
}

// LSL, LSR, RSL, RSR, RLR, LRL

std::tuple<double, double, double, std::vector<std::string>> dubins_path::LSL(double alpha, double beta, double dist)
{
    double sin_a = std::sin(alpha);
    double sin_b = std::sin(beta);
    double cos_a = std::cos(alpha);
    double cos_b = std::cos(beta);
    double cos_a_b = std::cos(alpha - beta);

    double p_lsl = 2 + dist * dist - 2 * cos_a_b + 2 * dist * (sin_a - sin_b);

    if (p_lsl < 0)
    {
        return {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), {"L", "S", "L"}};
    }
    else
    {
        p_lsl = std::sqrt(p_lsl);
    }

    double denominate = dist + sin_a - sin_b;
    double t_lsl = mod2pi(-alpha + std::atan2(cos_b - cos_a, denominate));
    double q_lsl = mod2pi(beta - std::atan2(cos_b - cos_a, denominate));

    return {t_lsl, p_lsl, q_lsl, {"L", "S", "L"}};
}

std::tuple<double, double, double, std::vector<std::string>> dubins_path::RSR(double alpha, double beta, double dist)
{
    double sin_a = std::sin(alpha);
    double sin_b = std::sin(beta);
    double cos_a = std::cos(alpha);
    double cos_b = std::cos(beta);
    double cos_a_b = std::cos(alpha - beta);

    double p_rsr = 2 + dist * dist - 2 * cos_a_b + 2 * dist * (sin_b - sin_a);

    if (p_rsr < 0)
    {
        return {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), {"R", "S", "R"}};
    }
    else
    {
        p_rsr = std::sqrt(p_rsr);
    }

    double denominate = dist - sin_a + sin_b;
    double t_rsr = mod2pi(alpha - std::atan2(cos_a - cos_b, denominate));
    double q_rsr = mod2pi(-beta + std::atan2(cos_a - cos_b, denominate));

    return {t_rsr, p_rsr, q_rsr, {"R", "S", "R"}};
}

std::tuple<double, double, double, std::vector<std::string>> dubins_path::LSR(double alpha, double beta, double dist)
{
    double sin_a = std::sin(alpha);
    double sin_b = std::sin(beta);
    double cos_a = std::cos(alpha);
    double cos_b = std::cos(beta);
    double cos_a_b = std::cos(alpha - beta);

    double p_lsr = -2 + dist * dist + 2 * cos_a_b + 2 * dist * (sin_a + sin_b);

    if (p_lsr < 0)
    {
        return {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), {"L", "S", "R"}};
    }
    else
    {
        p_lsr = std::sqrt(p_lsr);
    }

    double rec = std::atan2(-cos_a - cos_b, dist + sin_a + sin_b) - std::atan2(-2.0, p_lsr);
    double t_lsr = mod2pi(-alpha + rec);
    double q_lsr = mod2pi(-mod2pi(beta) + rec);

    return {t_lsr, p_lsr, q_lsr, {"L", "S", "R"}};
}

std::tuple<double, double, double, std::vector<std::string>> dubins_path::RSL(double alpha, double beta, double dist)
{
    double sin_a = std::sin(alpha);
    double sin_b = std::sin(beta);
    double cos_a = std::cos(alpha);
    double cos_b = std::cos(beta);
    double cos_a_b = std::cos(alpha - beta);

    double p_rsl = -2 + dist * dist + 2 * cos_a_b - 2 * dist * (sin_a + sin_b);

    if (p_rsl < 0)
    {
        return {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), {"R", "S", "L"}};
    }
    else
    {
        p_rsl = std::sqrt(p_rsl);
    }

    double rec = std::atan2(cos_a + cos_b, dist - sin_a - sin_b) - std::atan2(2.0, p_rsl);
    double t_rsl = mod2pi(alpha - rec);
    double q_rsl = mod2pi(beta - rec);

    return {t_rsl, p_rsl, q_rsl, {"R", "S", "L"}};
}

std::tuple<double, double, double, std::vector<std::string>> dubins_path::RLR(double alpha, double beta, double dist)
{
    double sin_a = std::sin(alpha);
    double sin_b = std::sin(beta);
    double cos_a = std::cos(alpha);
    double cos_b = std::cos(beta);
    double cos_a_b = std::cos(alpha - beta);

    double rec = (6.0 - dist * dist + 2.0 * cos_a_b + 2.0 * dist * (sin_a - sin_b)) / 8.0;

    if (std::abs(rec) > 1.0)
    {
        return {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), {"R", "L", "R"}};
    }

    double p_rlr = mod2pi(2 * M_PI - std::acos(rec));
    double t_rlr = mod2pi(alpha - std::atan2(cos_a - cos_b, dist - sin_a + sin_b) + mod2pi(p_rlr / 2.0));
    double q_rlr = mod2pi(alpha - beta - t_rlr + mod2pi(p_rlr));

    return {t_rlr, p_rlr, q_rlr, {"R", "L", "R"}};
}

std::tuple<double, double, double, std::vector<std::string>> dubins_path::LRL(double alpha, double beta, double dist)
{
    double sin_a = std::sin(alpha);
    double sin_b = std::sin(beta);
    double cos_a = std::cos(alpha);
    double cos_b = std::cos(beta);
    double cos_a_b = std::cos(alpha - beta);

    double rec = (6.0 - dist * dist + 2.0 * cos_a_b + 2.0 * dist * (sin_b - sin_a)) / 8.0;

    if (std::abs(rec) > 1.0)
    {
        return {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), {"L", "R", "L"}};
    }

    double p_lrl = mod2pi(2 * M_PI - std::acos(rec));
    double t_lrl = mod2pi(-alpha - std::atan2(cos_a - cos_b, dist + sin_a - sin_b) + p_lrl / 2.0);
    double q_lrl = mod2pi(mod2pi(beta) - alpha - t_lrl + mod2pi(p_lrl));

    return {t_lrl, p_lrl, q_lrl, {"L", "R", "L"}};
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<int>> dubins_path::interpolate(
    int ind, double l, const std::string &m, double maxc, double ox, double oy, double oyaw,
    std::vector<double> &px, std::vector<double> &py, std::vector<double> &pyaw, std::vector<int> &directions)
{

    if (m == "S")
    {
        px[ind] = ox + l / maxc * std::cos(oyaw);
        py[ind] = oy + l / maxc * std::sin(oyaw);
        pyaw[ind] = oyaw;
    }
    else
    {
        double ldx = std::sin(l) / maxc;
        double ldy = 0.0;
        if (m == "L")
        {
            ldy = (1.0 - std::cos(l)) / maxc;
        }
        else if (m == "R")
        {
            ldy = (1.0 - std::cos(l)) / (-maxc);
        }

        double gdx = std::cos(-oyaw) * ldx + std::sin(-oyaw) * ldy;
        double gdy = -std::sin(-oyaw) * ldx + std::cos(-oyaw) * ldy;
        px[ind] = ox + gdx;
        py[ind] = oy + gdy;

        if (m == "L")
        {
            pyaw[ind] = oyaw + l;
        }
        else if (m == "R")
        {
            pyaw[ind] = oyaw - l;
        }
    }

    if (l > 0.0)
    {
        directions[ind] = 1;
    }
    else
    {
        directions[ind] = -1;
    }

    return std::make_tuple(px, py, pyaw, directions);
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<int>> dubins_path::generate_local_course(
    double L, const std::vector<double> &lengths, const std::vector<std::string> &mode, double maxc, double step_size)
{

    int point_num = static_cast<int>(L / step_size) + lengths.size() + 3;
    std::vector<double> px(point_num, 0.0);
    std::vector<double> py(point_num, 0.0);
    std::vector<double> pyaw(point_num, 0.0);
    std::vector<int> directions(point_num, 0);

    int ind = 1;
    double d = (lengths[0] > 0.0) ? step_size : -step_size;
    directions[0] = (lengths[0] > 0.0) ? 1 : -1;
    double ll = 0.0;

    for (size_t i = 0; i < mode.size(); ++i)
    {
        double l = lengths[i];
        std::string m = mode[i];

        if (l > 0.0)
        {
            d = step_size;
        }
        else
        {
            d = -step_size;
        }

        double ox = px[ind], oy = py[ind], oyaw = pyaw[ind];

        ind -= 1;
        double pd = (i >= 1 && (lengths[i - 1] * lengths[i]) > 0) ? -d - ll : d - ll;

        while (std::abs(pd) <= std::abs(l))
        {
            ind += 1;
            std::tie(px, py, pyaw, directions) = interpolate(ind, pd, m, maxc, ox, oy, oyaw, px, py, pyaw, directions);
            pd += d;
        }

        ll = l - pd - d; // calculate remaining length

        ind += 1;
        std::tie(px, py, pyaw, directions) = interpolate(ind, l, m, maxc, ox, oy, oyaw, px, py, pyaw, directions);
    }

    // Remove unused data
    while (px.size() > 1 && px.back() == 0.0)
    {
        px.pop_back();
        py.pop_back();
        pyaw.pop_back();
        directions.pop_back();
    }

    if (px.size() <= 1)
    {
        return {{}, {}, {}, {}};
    }

    return std::make_tuple(px, py, pyaw, directions);
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<std::string>, double> dubins_path::planning_from_origin(
    double gx, double gy, double gyaw, double curv, double step_size)
{

    double D = std::hypot(gx, gy);
    double d = D * curv;
    double theta = mod2pi(std::atan2(gy, gx));
    double alpha = mod2pi(-theta);
    double beta = mod2pi(gyaw - theta);

    std::vector<std::tuple<double, double, double, std::vector<std::string>>> planners = {
        LSL(alpha, beta, d),
        RSR(alpha, beta, d),
        LSR(alpha, beta, d),
        RSL(alpha, beta, d),
        RLR(alpha, beta, d),
        LRL(alpha, beta, d)};

    double best_cost = std::numeric_limits<double>::infinity();
    double bt = 0.0, bp = 0.0, bq = 0.0;
    std::vector<std::string> best_mode;

    for (const auto &planner : planners)
    {
        double t, p, q;
        std::vector<std::string> mode;
        std::tie(t, p, q, mode) = planner;

        if (std::isnan(t))
        {
            continue;
        }

        double cost = std::abs(t) + std::abs(p) + std::abs(q);
        if (best_cost > cost)
        {
            bt = t;
            bp = p;
            bq = q;
            best_mode = mode;
            best_cost = cost;
        }
    }

    std::vector<double> lengths = {bt, bp, bq};
    auto [x_list, y_list, yaw_list, directions] = generate_local_course(std::accumulate(lengths.begin(), lengths.end(), 0.0), lengths, best_mode, curv, step_size);

    return std::make_tuple(x_list, y_list, yaw_list, best_mode, best_cost); // Use std::make_tuple to create a tuple
}

Path dubins_path::calc_dubins_path(double sx, double sy, double syaw, double gx, double gy, double gyaw, double curv, double step_size)
{
    gx -= sx;
    gy -= sy;

    double cos_yaw = std::cos(syaw);
    double sin_yaw = std::sin(syaw);

    double le_x = gx * cos_yaw + gy * sin_yaw;
    double le_y = -gx * sin_yaw + gy * cos_yaw;
    double le_yaw = gyaw - syaw;

    auto [lp_x, lp_y, lp_yaw, mode, lengths] = planning_from_origin(le_x, le_y, le_yaw, curv, step_size);

    if (lp_x.empty() || lp_y.empty() || lp_yaw.empty())
    {
        std::cerr << "Error: Empty path generated." << std::endl;
    }

    std::vector<double> x_list(lp_x.size()), y_list(lp_y.size()), yaw_list(lp_yaw.size());
    for (size_t i = 0; i < lp_x.size(); ++i)
    {
        x_list[i] = lp_x[i] * cos_yaw - lp_y[i] * sin_yaw + sx;
        y_list[i] = lp_x[i] * sin_yaw + lp_y[i] * cos_yaw + sy;
        yaw_list[i] = pi_2_pi(lp_yaw[i] + syaw);
    }

    return {lengths, mode, x_list, y_list, yaw_list};
}